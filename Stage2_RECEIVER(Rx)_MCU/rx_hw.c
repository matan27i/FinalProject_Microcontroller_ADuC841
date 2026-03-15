/* =============================================================================
   File: rx_hw.c
   Module: Group 1 -- Hardware Abstraction Layer
   Target: ADuC841 (8052 single-cycle core, 11.0592 MHz crystal)

   This is the ONLY file that contains ADuC841-specific register writes.
   All MCU-specific knowledge is confined here and in the pin definitions
   inside rx_system.h.  When porting, replace only this file.

   UPGRADE: Added support for reading the full 25-bit bus word
   (15 H1 data + 10 PESEC redundancy) from the parallel port interface.
   =============================================================================
*/

#include "rx_system.h"


/* [N2] RX_GlobalINT() removed: no individual interrupts are enabled
   in this design (ES=0, no ETx bits set), so EA=1 is a no-op.
   If interrupts are added later, call EA=1 after all peripheral inits. */


/* ---------------------------------------------------------------------------
   RX_Timer3_Init
   ---------------------------------------------------------------------------
   Configures Timer 3 as the dedicated UART baud-rate generator targeting
   exactly 9600 baud with a 11.0592 MHz crystal.

   Register values:
     T3CON = 0x86  ->  T3BAUDEN=1, DIV=6
     T3FD  = 0x08  ->  Fractional correction

   Baud = (2 * fCORE) / (2^(DIV-1) * (T3FD + 64))
        = 22118400 / (32 * 72) = 9600.0 baud  (exact, zero error)
   ---------------------------------------------------------------------------
*/
void RX_Timer3_Init(void)
{
    /* [M3] Direct assignment; no need to clear first since we set all bits.
       T3BAUDEN=1, DIV2=1, DIV1=1, DIV0=0 -> DIV=6 */
    T3CON = 0x86u;     /* T3BAUDEN=1, DIV=6 for 9600 baud at 11.0592 MHz */
    T3FD  = 0x08u;     /* Fractional correction: (1 + 8/64) = 1.125 */
}


/* ---------------------------------------------------------------------------
   RX_UART_Init
   ---------------------------------------------------------------------------
   Configures the UART for 8-bit asynchronous mode (Mode 1), transmit-only,
   9600 baud (clocked by Timer 3).

   TI pre-arm: writing TI=1 at the end allows the very first call to
   rx_send_uart_byte() to proceed without blocking forever.
   ---------------------------------------------------------------------------
*/
void RX_UART_Init(void)
{
    SM0 = 0;    /* Mode 1: 8-bit UART */
    SM1 = 1;    /* Variable baud rate */
    REN = 0;    /* Receiver disabled -- TX only */
    RI  = 0;    /* Clear receive interrupt flag */
    TI  = 0;    /* Clear transmit interrupt flag */
    ES  = 0;    /* Disable serial interrupt */

    /* Pre-arm TI so the first SBUF write does not stall. */
    TI  = 1;
}


/* ---------------------------------------------------------------------------
   RX_Port_Init
   ---------------------------------------------------------------------------
   Configures all I/O pins used for the 25-bit bus interface.

   Port 0 (P0.0-P0.7) -- H1 data bits 0-7:
     Standard 8051 open-drain port.  Writing 0xFF configures all pins
     as high-impedance inputs.

   Port 1 (P1.0-P1.6) -- H1 data bits 8-14:
     ADuC841 ANOMALY: Port 1 defaults to ANALOG INPUT mode on reset.
     Writing a 0 to a P1 bit switches it to DIGITAL INPUT mode.
     P1 &= 0x80 clears bits 0-6 (digital) while leaving P1.7 analog.

   Port 2 (P2.0-P2.7) -- PESEC redundancy bits 0-7:
     Standard bidirectional port.  Writing 0xFF configures all pins
     as high-impedance inputs for reading the redundancy bus.

   Port 3 bits 3-4 (P3.3/INT1, P3.4/T0) -- PESEC redundancy bits 8-9:
     Bidirectional with internal pull-ups.  Default as inputs after
     reset; writing 1 ensures they remain high-impedance.

   Port 3 bit 2 (P3.2/INT0) -- DATA_READY:
     Defaults to input after reset; no explicit configuration needed.
   ---------------------------------------------------------------------------
*/
void RX_Port_Init(void)
{
    /* P0.0-P0.7 as digital inputs for H1 data bits 0-7. */
    P0 = 0xFFu;

    /* P1.0-P1.6 as DIGITAL inputs for H1 data bits 8-14.
       Write 0 to switch from analog to digital mode (ADuC841 anomaly). */
    P1 &= 0x80u;

    /* P2.0-P2.7 as digital inputs for PESEC redundancy bits 0-7. */
    P2 = 0xFFu;

    /* P3.3 and P3.4 as inputs for PESEC redundancy bits 8-9.
       Write 1 to keep them as high-impedance inputs.
       Do NOT disturb P3.0 (RXD), P3.1 (TXD), or P3.2 (DATA_READY). */
    P3 |= 0x18u;   /* Set bits 3 and 4: 0001_1000 */
}


/* ---------------------------------------------------------------------------
   rx_read_full_bus
   ---------------------------------------------------------------------------
   Reads the complete 25-bit word in a single call, ensuring both the
   data and redundancy are sampled as close together as possible.

   The 74HC595 latched outputs are static between DATA_READY edges, so
   there is no risk of the values changing mid-read as long as this
   function is called before DATA_READY de-asserts.
   ---------------------------------------------------------------------------
*/
void rx_read_full_bus(uint16_t *out_data, uint16_t *out_red)
{
    /* Read data bits first (15 bits from P0 + P1). */
    *out_data  = (uint16_t)P0;
    *out_data |= ((uint16_t)(P1 & 0x7Fu)) << 8;
    *out_data &= BUS_STATE_MASK;

    /* Read redundancy bits (10 bits from P2 + P3.3 + P3.4). */
    *out_red  = (uint16_t)P2;
    *out_red |= ((uint16_t)ECC_BIT8) << 8;
    *out_red |= ((uint16_t)ECC_BIT9) << 9;
    *out_red &= PESEC_RED_MASK;
}


/* ---------------------------------------------------------------------------
   [M2] Non-blocking UART TX output buffer.
   Bytes are enqueued by rx_send_uart_byte() and drained by rx_uart_pump()
   which the main loop calls on every iteration, never blocking.
   ---------------------------------------------------------------------------
*/
/* TX buffer lives in XRAM (2 KB internal, accessed via MOVX).
   CFG841.XRAMEN must be set in main() before first access.
   Head/tail indices stay in DATA for fast single-cycle access. */
#define TX_BUF_SIZE 16
static uint8_t xdata tx_buf[TX_BUF_SIZE];
static uint8_t tx_head = 0;
static uint8_t tx_tail = 0;


/* ---------------------------------------------------------------------------
   rx_send_uart_byte
   ---------------------------------------------------------------------------
   Enqueues one byte into the TX output buffer.  Non-blocking: if the
   buffer is full, the byte is silently dropped.
   ---------------------------------------------------------------------------
*/
void rx_send_uart_byte(uint8_t data_byte)
{
    uint8_t next_head;

    next_head = (tx_head + 1) % TX_BUF_SIZE;
    if (next_head == tx_tail)
    {
        return;  /* Buffer full, drop byte */
    }

    tx_buf[tx_head] = data_byte;
    tx_head = next_head;
}


/* ---------------------------------------------------------------------------
   rx_uart_pump
   ---------------------------------------------------------------------------
   Drains one byte from the TX buffer into SBUF if TI is set (previous
   byte finished transmitting).  Must be called from the main loop on
   every iteration.

   This replaces the blocking while(!TI) loop, ensuring the main loop
   never stalls and DATA_READY edges are not missed.
   ---------------------------------------------------------------------------
*/
void rx_uart_pump(void)
{
    if (TI && (tx_tail != tx_head))
    {
        TI = 0;
        SBUF = tx_buf[tx_tail];
        tx_tail = (tx_tail + 1) % TX_BUF_SIZE;
    }
}
