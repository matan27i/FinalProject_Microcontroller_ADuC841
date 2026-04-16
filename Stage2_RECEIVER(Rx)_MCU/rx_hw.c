/* 
   File: rx_hw.c

   ADuC841 PORT 1 ANALOG-MODE ANOMALY:
   On the ADuC841, all Port 1 pins power up in ANALOG INPUT mode.  In
   this mode the digital input buffer is disabled and the pin reads as
   indeterminate.  Writing a 0 to a P1 bit switches that pin to DIGITAL
   mode.  This must be done BEFORE enabling any peripheral (like SPI)
   that relies on reading the pin.

   The SPI /SS pin is P1.5.  If SPE is set while P1.5 is still in
   analog mode, the SPI slave cannot detect /SS transitions and will
   either ignore frames entirely or sample data at random phases.
   
*/

#include "rx_system.h"

/* 
   SPI RECEIVE STATE VARIABLES
    */

/* Staging registers: assembled 26-bit word from the last complete frame.
   Written atomically by the SPI ISR; read by the main loop when
   rx_spi_frame_ready == 1. */
volatile uint16_t rx_spi_data        = 0;
volatile uint16_t rx_spi_red         = 0;
volatile uint8_t  rx_spi_parity      = 0;   /* [W4] Overall parity bit */
volatile uint8_t  rx_spi_frame_ready = 0;

/* ISR-private byte accumulation buffer and counter. */
static uint8_t spi_rx_buf[4];
static uint8_t spi_byte_idx = 0;


/* 
   INTERRUPT SERVICE ROUTINES
    */

/* 
   INT0 ISR (vector 0, address 0x0003) -- /SS Frame-Sync
   
   Fires on the falling edge of /SS (Mega asserts slave select).
   Resets the byte counter to 0 so the next SPI byte is byte 0.
   
*/
void INT0_FrameSync_ISR(void) interrupt 0
{
    spi_byte_idx = 0;
}


/* 
   SPI/I2C ISR (vector 7, address 0x003B) -- Byte Received
   
   Fires after each complete 8-bit SPI transfer.

   FRAME PROTOCOL (4 bytes per frame):
     Index 0: H1 data bits  [7:0]
     Index 1: bit 7 = overall parity (W4)
              bits [6:0] = H1 data bits [14:8]
     Index 2: ECC red bits  [7:0]
     Index 3: ECC red bits  [9:8] (bits 7-2 = 0)

   After storing byte 3, the ISR assembles the 15-bit data, 10-bit
   redundancy, and 1-bit parity, then sets rx_spi_frame_ready = 1.
   
*/
sbit DEBUG_LED = P3^4;
void SPI_Receive_ISR(void) interrupt 7
{
    uint8_t rx_byte;
    DEBUG_LED = !DEBUG_LED;
    /* Read SPIDAT to get the received byte AND clear the interrupt flag. */
    rx_byte = SPIDAT;

    if (spi_byte_idx >= 4)
    {
        return;
    }

    spi_rx_buf[spi_byte_idx] = rx_byte;
    spi_byte_idx++;

    if (spi_byte_idx == 4)
    {
        /* Assemble the 15-bit data word from bytes 0 and 1.
           Byte 0 = data[7:0].
           Byte 1 bits [6:0] = data[14:8].
           Byte 1 bit 7 = overall parity (extracted separately). */
        rx_spi_data = (uint16_t)spi_rx_buf[0]
                    | (((uint16_t)(spi_rx_buf[1] & 0x7Fu)) << 8);

        /* Assemble the 10-bit redundancy word from bytes 2 and 3. */
        rx_spi_red  = (uint16_t)spi_rx_buf[2]
                    | (((uint16_t)(spi_rx_buf[3] & 0x03u)) << 8);

        /* [W4] Extract the overall parity bit from byte 1, bit 7.
           This is the TX-computed even parity of all 25 bits
           (15 data + 10 redundancy).  It rides in the previously-
           unused high bit of byte 1. */
        rx_spi_parity = (spi_rx_buf[1] >> 7) & 0x01u;

        rx_spi_frame_ready = 1;

        /* [F3] Reset byte index so the next frame can be received even
           if /SS stays asserted (low) continuously.  INT0 still provides
           resync on SS edges, but normal operation no longer depends on
           SS toggling between frames. */
        spi_byte_idx = 0;
    }
}


/* 
   PERIPHERAL INITIALISATION
    */

/* 
   RX_Timer3_Init  (unchanged)
   
*/
void RX_Timer3_Init(void)
{
    T3CON = 0x86u;
    T3FD  = 0x08u;
}


/* 
   RX_UART_Init  (unchanged)
   
*/
void RX_UART_Init(void)
{
    SM0 = 0;
    SM1 = 1;
    REN = 0;
    RI  = 0;
    TI  = 0;
    ES  = 0;

    TI  = 1;
}


/* 
   RX_SPI_Slave_Init
   
   [F2] FIX: P1.5 ANALOG-MODE BUG

   The ADuC841 Port 1 defaults to ANALOG INPUT mode on power-up.
   In analog mode, the digital input buffer is disconnected, so the
   pin cannot reliably read logic levels.  P1.5 is the SPI /SS pin.
   If SPE is enabled while P1.5 is still analog, the SPI slave
   peripheral cannot detect the master's slave-select assertion.

   FIX: Write 0 to P1.5 BEFORE enabling SPE.  On the ADuC841, writing
   a 0 to any Port 1 bit switches that pin from analog to digital mode.
   We use a bitwise AND to clear only P1.5 without disturbing other P1
   bits that may be intentionally left in analog mode for ADC use.

   After the digital-mode switch, SPI is enabled as before.

   [W4] rx_spi_parity is reset to 0 alongside the other staging registers.

   SPICON = 0x20:
     SPE=1 (SPI enable), SPIM=0 (slave), CPOL=0, CPHA=0 (Mode 0).
     [FIX] Was 0xA0; bit 7 (ISPI) is a status flag, not an enable.
   IEIP2 |= 0x01:
     [FIX] Enable ESI (SPI/I2C interrupt source) -- was never set.
   
*/
void RX_SPI_Slave_Init(void)
{
    /* Reset SPI receive state. */
    spi_byte_idx       = 0;
    rx_spi_frame_ready = 0;
    rx_spi_data        = 0;
    rx_spi_red         = 0;
    rx_spi_parity      = 0;   /* [W4] */

    /* [F2] Force P1.5 (/SS) from analog mode to digital input mode.
       On the ADuC841, SCLOCK and SDATA/MOSI are dedicated pins (not on
       P1), and MISO is on P3.3.  Only /SS (P1.5) is a Port 1 pin that
       needs the analog-to-digital switch.  We also clear P1.7-P1.4 to
       be safe, since the receiver does not use ADC channels 4-7. */
    P1 &= ~0xF0u;   /* P1.7..P1.4 = 0 -> digital mode */

    /* Pre-load transmit register. */
    SPIDAT = 0x00u;

    /* Configure SPI: slave mode, Mode 0.
       SPE=1, SPIM=0, CPOL=0, CPHA=0, SPR=00 => 0x20.
       [FIX] Was 0xA0: bit 7 (ISPI) is a hardware status flag, not an
       enable.  Pre-setting it caused a spurious interrupt at startup. */
    SPICON = 0x20u;

    /* [FIX] Enable the SPI/I2C interrupt source (ESI, bit 0 of IEIP2).
       Without this, the SPI ISR at vector 003BH never fires regardless
       of the ISPI flag state.  IEIP2 power-on default is A0H (ESI=0).
       Use ORL to preserve other bits (priority settings). */
    IEIP2 |= 0x01u;

    /* Configure INT0 (P3.2) for falling-edge trigger. */
    IT0 = 1;
    IE0 = 0;
    EX0 = 1;

    /* EA is NOT set here; caller must set EA=1 after all inits. */
}


/* 
   NON-BLOCKING UART TX BUFFER  (unchanged)
    */

#define TX_BUF_SIZE 64
static uint8_t xdata tx_buf[TX_BUF_SIZE];
static uint8_t tx_head = 0;
static uint8_t tx_tail = 0;


void rx_send_uart_byte(uint8_t data_byte)
{
    uint8_t next_head;

    next_head = (tx_head + 1) % TX_BUF_SIZE;
    if (next_head == tx_tail)
    {
        return;
    }

    tx_buf[tx_head] = data_byte;
    tx_head = next_head;
}


void rx_uart_pump(void)
{
    if (TI && (tx_tail != tx_head))
    {
        TI = 0;
        SBUF = tx_buf[tx_tail];
        tx_tail = (tx_tail + 1) % TX_BUF_SIZE;
    }
}
