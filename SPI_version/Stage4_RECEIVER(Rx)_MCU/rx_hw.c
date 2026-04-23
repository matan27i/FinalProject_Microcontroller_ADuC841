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

/* Set RX_DEBUG_LED to 1 to enable in-ISR LED toggle on P3.4 for scope
   debugging of SPI byte reception.
   [FIX] Default is now 0 (OFF).  With the Arduino SPI clock dropped
   to 50 kHz the ISR has plenty of deadline margin, but the P3.4
   toggle added ~8 instruction cycles per byte plus EMI on a port pin
   adjacent to INT0 (P3.2) -- a plausible contributor to spurious
   INT0 falling edges that zero spi_byte_idx mid-frame.  Compile with
   -DRX_DEBUG_LED=1 to re-enable for scope work. */
#ifndef RX_DEBUG_LED
#define RX_DEBUG_LED 0
#endif

#if RX_DEBUG_LED
sbit DEBUG_LED = P3^4;
#endif

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

/* ISR-private byte accumulation buffer and counter.
   [W6] volatile because spi_byte_idx is written from two ISRs
   (INT0 vector 0 and SPI vector 7).  They cannot preempt each other at
   equal priority, but volatile prevents compiler reordering/caching. */
static uint8_t spi_rx_buf[4];
static volatile uint8_t spi_byte_idx = 0;


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
void SPI_Receive_ISR(void) interrupt 7
{
    uint8_t rx_byte;
    uint8_t idx;

#if RX_DEBUG_LED
    DEBUG_LED = !DEBUG_LED;
#endif

    /* Reading SPIDAT returns the just-clocked byte AND clears ISPI. */
    rx_byte = SPIDAT;

    /* Snapshot the index once so we don't re-read the volatile
       inside the hot path. */
    idx = spi_byte_idx;
    if (idx >= 4)
    {
        /* Stale byte after the frame was assembled but before the main
           loop consumed rx_spi_frame_ready.  Drop it. */
        return;
    }

    spi_rx_buf[idx] = rx_byte;
    idx++;
    spi_byte_idx = idx;

    if (idx == 4)
    {
        /* 15-bit data word from bytes 0 (low) and 1[6:0] (high).
           Byte 1 bit 7 carries the overall parity. */
        rx_spi_data = (uint16_t)spi_rx_buf[0]
                    | (((uint16_t)(spi_rx_buf[1] & 0x7Fu)) << 8);

        /* 10-bit redundancy from bytes 2 (low) and 3[1:0] (high). */
        rx_spi_red  = (uint16_t)spi_rx_buf[2]
                    | (((uint16_t)(spi_rx_buf[3] & 0x03u)) << 8);

        rx_spi_parity = (spi_rx_buf[1] >> 7) & 0x01u;

        rx_spi_frame_ready = 1;

        /* Allow the next frame to arrive without requiring INT0 resync.
           INT0 is still enabled as a belt-and-braces realignment; it
           fires on the /SS falling edge at the START of the next frame
           and zeros spi_byte_idx again, which is a no-op after this
           self-reset. */
        spi_byte_idx = 0;
    }
}


/* 
   PERIPHERAL INITIALISATION
    */

/*
   RX_Timer3_Init

   Configures Timer 3 (ADuC841 dedicated UART baud rate generator) for
   115200 baud at a core clock of 11.0592 MHz (CD=0 in PLLCON).

  
*/
void RX_Timer3_Init(void)
{
    T3FD  = 0x20u;   /* Fractional divider = 32 */
    T3CON = 0x82u;   /* Enable Timer 3 as UART baud source, DIV = 2 */
}


/*
   RX_UART_Init

   [FIX] Write SCON as a full byte instead of individual sbit writes.
   The ADuC841 Keil header aliases SCON bit 7 as SM0/FE; depending on
   SMOD0 in PCON, sbit writes to SM0/SM1 can silently target the wrong
   bit and leave SCON = 0x00 (Mode 0, synchronous shift-register at
   f_core/12).  In that mode, SBUF bytes are clocked out on TxD as raw
   8-bit shift-register pulses at ~921 kHz with no start/stop framing,
   so PuTTY at 115200 baud (or any async baud) sees nothing decodable.
   This is the same trap fixed on the TX side in tx_main.c [C4].

   SCON = 0x40 -> SM0=0, SM1=1 (Mode 1, 8-bit async UART, Timer-3 baud)
                  SM2=0, REN=0 (TX-only, receive disabled)
                  TB8=RB8=TI=RI=0
   ES remains 0: the TX path is polled via rx_uart_pump(), not ISR-driven.
*/
void RX_UART_Init(void)
{
    SCON = 0x40;
    ES   = 0;
    TI   = 1;   /* Seed TI so the first rx_uart_pump() can write SBUF */
}


/* 
   RX_SPI_Slave_Init
   
   [F2] FIX: P1.5 ANALOG-MODE BUG

   The ADuC841 Port 1 defaults to ANALOG INPUT mode on power-up.
   In analog mode, the digital input buffer is disconnected, so the
   pin cannot reliably read logic levels.  P1.5 is the SPI /SS pin.
   If SPE is enabled while P1.5 is still analog, the SPI slave
   peripheral cannot detect the master's slave-select assertion.

   After the digital-mode switch, SPI is enabled as before.

   rx_spi_parity is reset to 0 alongside the other staging registers.

   SPICON = 0x24  (Mode 1, CPHA=1):
     SPE=1 (SPI enable), SPIM=0 (slave), CPOL=0, CPHA=1.

     *** ADuC841 DATASHEET CPHA=0 QUIRK ***
     Per the ADuC841 datasheet (SPI slave section, p.54):
       "If CPHA = 1, the SS input may be permanently pulled low.
        If CPHA = 0, the SS input must be driven LOW before the
        first bit in a byte-wide transmission or reception and
        return HIGH again after the last bit in that byte-wide
        transmission or reception."
       "The end of transmission occurs after the eighth clock
        has been received if CPHA = 1, or when SS returns high
        if CPHA = 0."
     In other words, Mode 0 (CPHA=0) requires /SS to toggle
     HIGH-LOW *per byte*.  The Arduino's sendFrame() holds /SS
     LOW for all 4 bytes of a frame, which violates the CPHA=0
     protocol -- SPIDAT is not latched at proper byte boundaries,
     so received bytes are shifted/duplicated, PESEC fires on
     every frame, rx_is_bus_reset() never sees {0,0,0,0}, and
     the FSM emits the scrambled 'FdFdrFrFdFdFr!' pattern.
     Switching to Mode 1 (CPHA=1) allows /SS to remain LOW
     across the entire 4-byte frame with per-byte latching on
     the 8th SCLK edge -- which is exactly what the Arduino
     master is doing.  testspii.ino must match with SPI_MODE1.

   IEIP2 |= 0x01:
     Enable ESI (SPI/I2C interrupt source) -- was never set.

*/
void RX_SPI_Slave_Init(void)
{
    /* Reset SPI receive state. */
    spi_byte_idx       = 0;
    rx_spi_frame_ready = 0;
    rx_spi_data        = 0;
    rx_spi_red         = 0;
    rx_spi_parity      = 0;   

    /* Force P1.5 (/SS) from analog mode to digital input mode.
       On the ADuC841, SCLOCK and SDATA/MOSI are dedicated pins (not on
       P1), and MISO is on P3.3.  Only /SS (P1.5) is a Port 1 pin that
       needs the analog-to-digital switch.  We also clear P1.7-P1.4 to
       be safe, since the receiver does not use ADC channels 4-7. */
    P1 &= ~0xF0u;   /* P1.7..P1.4 = 0 -> digital mode */

    /* Pre-load transmit register. */
    SPIDAT = 0x00u;

    /* Configure SPI: slave mode, Mode 1 (CPHA=1).
       SPE=1, SPIM=0, CPOL=0, CPHA=1, SPR=00 => 0x24.
       [FIX] Was 0x20 (Mode 0): the ADuC841 slave requires /SS to
       toggle per-byte in CPHA=0 mode (see datasheet p.54 and the
       block comment above).  Arduino's sendFrame() holds /SS low
       across all 4 bytes, so we must use CPHA=1 (Mode 1) instead
       so that each byte is latched on the 8th SCLK rather than
       on the /SS rising edge.  testspii.ino uses SPI_MODE1 to
       match. */
    SPICON = 0x24u;

    /* [FIX] Enable the SPI/I2C interrupt source (ESI, bit 0 of IEIP2).
       Without this, the SPI ISR at vector 003BH never fires regardless
       of the ISPI flag state.  IEIP2 power-on default is A0H (ESI=0).
       Use ORL to preserve other bits (priority settings). */
    IEIP2 |= 0x01u;

    /* Configure INT0 (P3.2) for falling-edge trigger on /SS assertion.
     *
     * Arduino pin 10 drives BOTH P1.5 (SPI /SS) and P3.2 (INT0) per the
     * testspii.ino wiring note.  Each Arduino sendFrame() lowers pin 10,
     * which fires INT0_FrameSync_ISR and zeros spi_byte_idx -- giving a
     * hard resync at every frame start, independent of how many SPI
     * bytes the previous frame delivered.
     *
     * History: this was briefly disabled (EX0 = 0) during bring-up when
     * we were not sure whether P3.2 was wired.  The [F3] self-reset at
     * idx == 4 let frames assemble, but a single lost byte left
     * spi_byte_idx stuck mid-frame -- producing the symptom where reset
     * frames ({0,0,0,0}) never triggered rx_is_bus_reset() because the
     * received bytes were shifted by one slot against the previous
     * frame's tail.  INT0 is the correct mechanism for frame alignment
     * now that the SPI path itself is confirmed working. */
    IT0 = 1;
    IE0 = 0;
    EX0 = 1;   /* [FIX] RE-ENABLED.  Frame-alignment hardware resync.
                  The current testspiiGuCci.ino drives pin 10 as an
                  OUTPUT: HIGH between frames, LOW during a frame
                  (see sendFrame() in that sketch).  That gives P3.2
                  a clean, non-floating signal with exactly one falling
                  edge per 4-byte frame.  INT0_FrameSync_ISR zeros
                  spi_byte_idx on that edge BEFORE the first SCLK byte
                  is latched, so the SPI ISR always sees byte0 in slot 0.
                  Without this, a single dropped SPI byte (very likely
                  during ADuC boot / first-frame race) permanently
                  shifts every subsequent frame by N bytes -- which is
                  exactly the "12 consecutive FU" symptom we saw:
                  PESEC syndrome != 0 but parity OK = even-bit-flip
                  signature caused by byte-boundary misalignment, not
                  by PESEC or line noise.  The old reason to keep this
                  disabled (floating pin 10 on the legacy testspii.ino)
                  no longer applies. */

    /* EA is NOT set here; caller must set EA=1 after all inits. */
}


/* 
   NON-BLOCKING UART TX BUFFER 
    */

/* [FIX] Bumped 64 -> 128.  rx_send_uart_byte() drops on full, and
   under PuTTY flow-control pauses or back-to-back frames the early
   debug markers (F, R) were getting dropped while later FSM bytes
   (decoded chars) survived -- producing the "many A's, no F/R"
   pattern in the trace.  Bigger ring absorbs the bursts. */
#define TX_BUF_SIZE 128
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
