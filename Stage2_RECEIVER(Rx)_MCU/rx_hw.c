/* =============================================================================
   File: rx_hw.c
   Module: Group 1 -- Hardware Abstraction Layer (SPI Slave version)
   Target: ADuC841 (8052 single-cycle core, 11.0592 MHz crystal)

   MODIFICATION SUMMARY:
     The original design read 25 parallel bits from P0, P1, P2, and P3
     with polling on a DATA_READY GPIO.  This version replaces that
     entire input path with SPI slave reception from the Arduino Mega
     2560 bridge.

   SPI SLAVE OPERATION:
     The Mega sends a 4-byte frame (data_lo, data_hi, ecc_lo, ecc_hi)
     with /SS driven low for the duration.  Two interrupt sources
     cooperate to receive and assemble the frame:

       INT0 (P3.2, falling edge):
         Fires when the Mega asserts /SS low, signalling the start of
         a new 4-byte frame.  Resets the byte counter to 0, ensuring
         frame alignment even after glitches or startup transients.

       SPI/I2C interrupt (vector 6):
         Fires after each byte is clocked in.  Stores the byte in a
         4-element buffer and increments the counter.  When the 4th
         byte is stored, it assembles the 25-bit word and sets the
         rx_spi_frame_ready flag for the main loop.

     This two-interrupt design guarantees that byte 0 is always the
     first byte after /SS falls, regardless of prior state.

   ADuC841 SPI PERIPHERAL OVERVIEW:
   ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
     SPICON (SFR F8h, bit-addressable):
       Bit 7  ISPI   -- SPI interrupt enable/flag
                         In slave mode, set to 1 to enable ISR.
                         Cleared by hardware on ISR entry or by
                         reading SPIDAT.
       Bit 6  WCOL   -- Write collision flag (read-only).
       Bit 5  SPE    -- SPI Enable.  When set, the SPI pins take
                         over their alternate functions.
       Bit 4  SPIM   -- SPI Master/Slave select.
                         0 = Slave mode (we use this).
                         1 = Master mode.
       Bit 3  CPOL   -- Clock polarity.
                         0 = SCLK idle low (Mode 0/1).
       Bit 2  CPHA   -- Clock phase.
                         0 = sample on leading edge (Mode 0).
       Bit 1-0 SPR1:SPR0 -- Clock rate divisor (Master only; ignored
                             in slave mode).

     SPIDAT (SFR F7h):
       Read:  Returns the last received byte.
       Write: Loads the transmit shift register (not used here).

     SPISTA (SFR read address):
       Contains the ISPI flag copy and WCOL.  Reading SPISTA then
       SPIDAT clears the interrupt flag.

     SPI INTERRUPT:
       The SPI/I2C interrupt is at vector 6 (address 0x0033).
       It is enabled by:
         1. Setting ISPI (SPICON.7) = 1.
         2. Setting EA = 1 (global interrupt enable).

   NOTE ON PIN VERIFICATION:
     The SPI peripheral pins (SCLOCK, MOSI, MISO, /SS) are automatically
     routed when SPE=1.  The exact physical pins depend on the ADuC841
     package variant.  Consult your specific datasheet (Table 10, "Pin
     Function Description") and ensure the Mega's SPI lines are wired
     to the correct ADuC841 pins.
   =============================================================================
*/

#include "rx_system.h"

/* =========================================================================
   SPI RECEIVE STATE VARIABLES
   ========================================================================= */

/* Staging registers: assembled 25-bit word from the last complete frame.
   Written atomically by the SPI ISR; read by the main loop when
   rx_spi_frame_ready == 1. */
volatile uint16_t rx_spi_data       = 0;
volatile uint16_t rx_spi_red        = 0;
volatile uint8_t  rx_spi_frame_ready = 0;

/* ISR-private byte accumulation buffer and counter.
   These are only accessed within interrupt context (INT0 + SPI ISR),
   and since the 8051 does not nest interrupts by default, no additional
   synchronisation is required. */
static uint8_t spi_rx_buf[4];
static uint8_t spi_byte_idx = 0;


/* =========================================================================
   INTERRUPT SERVICE ROUTINES
   ========================================================================= */

/* ---------------------------------------------------------------------------
   INT0 ISR (vector 0, address 0x0003) -- /SS Frame-Sync
   ---------------------------------------------------------------------------
   Fires on the falling edge of /SS (Mega asserts slave select).
   Resets the byte counter to 0 so the next SPI byte received is
   interpreted as byte 0 of the frame.

   This ensures frame alignment is re-established at the start of every
   SPI transfer, even if a previous transfer was interrupted or the
   system just powered up.

   TIMING: The Mega inserts a ~125 ns delay between /SS assertion and
   the first SCK edge.  The INT0 ISR entry latency on the 8051 is
   3-8 machine cycles (~0.3-0.7 us at 11.0592 MHz), which completes
   well before the first SPI byte finishes (~2 us at 4 MHz SCK).
   ---------------------------------------------------------------------------
*/
void INT0_FrameSync_ISR(void) interrupt 0
{
    spi_byte_idx = 0;
}


/* ---------------------------------------------------------------------------
   SPI/I2C ISR (vector 6, address 0x0033) -- Byte Received
   ---------------------------------------------------------------------------
   Fires after each complete 8-bit SPI transfer.  The received byte is
   in SPIDAT.

   FRAME PROTOCOL (4 bytes per frame):
     Index 0: H1 data bits  [7:0]   -> low byte of data word
     Index 1: H1 data bits [14:8]   -> high byte of data word (bit 7 = 0)
     Index 2: ECC red bits  [7:0]   -> low byte of redundancy word
     Index 3: ECC red bits  [9:8]   -> high byte of redundancy (bits 7-2 = 0)

   After storing byte 3, the ISR assembles the two 16-bit values and
   sets rx_spi_frame_ready = 1.  The main loop detects this flag and
   feeds the data into the decode pipeline.

   INTERRUPT FLAG CLEARING:
     On the ADuC841, reading SPIDAT clears the SPI interrupt flag.
     The ISPI bit in SPICON is automatically cleared on ISR entry on
     some ADuC841 revisions; reading SPIDAT ensures it is cleared on all.
   ---------------------------------------------------------------------------
*/
void SPI_Receive_ISR(void) interrupt 6
{
    uint8_t rx_byte;

    /* Read SPIDAT to get the received byte AND clear the interrupt flag. */
    rx_byte = SPIDAT;

    /* Guard: if index is out of range (should not happen with INT0 sync),
       silently ignore the byte. */
    if (spi_byte_idx >= 4)
    {
        return;
    }

    spi_rx_buf[spi_byte_idx] = rx_byte;
    spi_byte_idx++;

    /* Check if a complete 4-byte frame has been received. */
    if (spi_byte_idx == 4)
    {
        /* Assemble the 15-bit data word from bytes 0 and 1.
           Byte 0 = data[7:0], Byte 1 = data[14:8] (bits 6:0 valid). */
        rx_spi_data = (uint16_t)spi_rx_buf[0]
                    | (((uint16_t)(spi_rx_buf[1] & 0x7Fu)) << 8);

        /* Assemble the 10-bit redundancy word from bytes 2 and 3.
           Byte 2 = red[7:0], Byte 3 = red[9:8] (bits 1:0 valid). */
        rx_spi_red  = (uint16_t)spi_rx_buf[2]
                    | (((uint16_t)(spi_rx_buf[3] & 0x03u)) << 8);

        /* Signal the main loop that a new frame is available. */
        rx_spi_frame_ready = 1;

        /* Do not reset spi_byte_idx here.  The next INT0 falling edge
           (start of the next frame) will reset it.  This prevents a
           stale or spurious SPI clock from being misinterpreted as
           the start of a new frame without /SS re-assertion. */
    }
}


/* =========================================================================
   PERIPHERAL INITIALISATION
   ========================================================================= */

/* ---------------------------------------------------------------------------
   RX_Timer3_Init  (unchanged from original)
   ---------------------------------------------------------------------------
   Configures Timer 3 as the dedicated UART baud-rate generator targeting
   exactly 9600 baud with a 11.0592 MHz crystal.

     T3CON = 0x86  ->  T3BAUDEN=1, DIV=6
     T3FD  = 0x08  ->  Fractional correction
     Baud  = 22118400 / (32 * 72) = 9600.0 (exact)
   ---------------------------------------------------------------------------
*/
void RX_Timer3_Init(void)
{
    T3CON = 0x86u;
    T3FD  = 0x08u;
}


/* ---------------------------------------------------------------------------
   RX_UART_Init  (unchanged from original)
   ---------------------------------------------------------------------------
   8-bit UART, transmit-only, 9600 baud (clocked by Timer 3).
   TI pre-armed so the first SBUF write does not stall.
   ---------------------------------------------------------------------------
*/
void RX_UART_Init(void)
{
    SM0 = 0;
    SM1 = 1;
    REN = 0;
    RI  = 0;
    TI  = 0;
    ES  = 0;

    TI  = 1;   /* Pre-arm for first byte. */
}


/* ---------------------------------------------------------------------------
   RX_SPI_Slave_Init
   ---------------------------------------------------------------------------
   Configures the ADuC841 SPI peripheral as a slave and sets up INT0
   for frame synchronisation.

   SPI CONFIGURATION (SPICON = 0xA0):
     Bit 7  ISPI = 1   Enable SPI interrupt.
     Bit 6  WCOL = 0   (read-only, cannot set)
     Bit 5  SPE  = 1   Enable SPI peripheral; pins become SPI function.
     Bit 4  SPIM = 0   Slave mode.
     Bit 3  CPOL = 0   Clock idle low  (SPI Mode 0, matches Mega config).
     Bit 2  CPHA = 0   Sample on leading (rising) edge (SPI Mode 0).
     Bit 1-0 SPR = 00  (Ignored in slave mode.)

     Combined: 1_0_1_0_0_0_0_0 = 0xA0

   INT0 CONFIGURATION:
     P3.2 (INT0) is connected to the Mega's /SS output.
     Configured for falling-edge trigger (IT0 = 1).
     Enabled via EX0 = 1 in the IE register.

   Pre-load SPIDAT with 0x00 so the slave has a defined first
   response byte (the Mega ignores MISO, but this is good practice).

   GLOBAL INTERRUPT NOTE:
     This function enables the SPI and INT0 interrupt sources but does
     NOT set EA=1.  The caller (main) must set EA=1 after all
     initialisations are complete, exactly as the original design did.
   ---------------------------------------------------------------------------
*/
void RX_SPI_Slave_Init(void)
{
    /* Reset SPI receive state. */
    spi_byte_idx       = 0;
    rx_spi_frame_ready = 0;
    rx_spi_data        = 0;
    rx_spi_red         = 0;

    /* Pre-load transmit register (slave sends 0x00 -- Mega ignores it). */
    SPIDAT = 0x00u;

    /* Configure SPI: slave mode, Mode 0, interrupt enabled.
       ISPI=1, SPE=1, SPIM=0, CPOL=0, CPHA=0, SPR=00 => 0xA0. */
    SPICON = 0xA0u;

    /* Configure INT0 (P3.2) for falling-edge trigger.
       IT0 = 1: edge-triggered (not level).
       IE0 is auto-cleared by hardware on ISR entry. */
    IT0 = 1;

    /* Clear any pending INT0 flag from startup noise. */
    IE0 = 0;

    /* Enable INT0 interrupt source. */
    EX0 = 1;

    /* NOTE: EA (global interrupt enable) is NOT set here.
       The caller must set EA=1 after all init functions complete.
       This prevents ISR entry with partially-configured peripherals. */
}


/* =========================================================================
   NON-BLOCKING UART TX BUFFER  (unchanged from original)
   =========================================================================
   Bytes are enqueued by rx_send_uart_byte() and drained by rx_uart_pump()
   which the main loop calls on every iteration.
   ========================================================================= */

/* TX buffer in XRAM (2 KB internal, accessed via MOVX).
   CFG841.XRAMEN must be set in main() before first access. */
#define TX_BUF_SIZE 16
static uint8_t xdata tx_buf[TX_BUF_SIZE];
static uint8_t tx_head = 0;
static uint8_t tx_tail = 0;


void rx_send_uart_byte(uint8_t data_byte)
{
    uint8_t next_head;

    next_head = (tx_head + 1) % TX_BUF_SIZE;
    if (next_head == tx_tail)
    {
        return;  /* Buffer full, drop byte. */
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
