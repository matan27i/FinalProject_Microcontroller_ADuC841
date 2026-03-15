/* =============================================================================
   File: rx_hw.c
   Module: Group 1 — Hardware Abstraction Layer
   Target: ADuC841 (8052 single-cycle core, 11.0592 MHz crystal)

   This is the ONLY file that contains ADuC841-specific register writes.
   All MCU-specific knowledge is confined here and in the pin definitions
   inside rx_system.h.  When porting, replace only this file.
   =============================================================================
*/


#include "rx_system.h"


/* ---------------------------------------------------------------------------
   RX_GlobalINT
   ---------------------------------------------------------------------------
   Enables the global interrupt master switch.
   Must be called before any interrupt-dependent peripheral begins operating.
   ---------------------------------------------------------------------------
*/
void RX_GlobalINT(void)
{
    EA = 1;
}


/* ---------------------------------------------------------------------------
   RX_Timer3_Init
   ---------------------------------------------------------------------------
   Configures Timer 3 as the dedicated UART baud-rate generator targeting
   exactly 9600 baud with a 11.0592 MHz crystal.

   Register values and derivation:
   ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
   T3CON = 0x86
     Bit 7  (T3BAUDEN) = 1  : Timer 3 drives the UART baud rate.
     Bits 6:3           = 0  : reserved; must be zero.
     Bits 2:1:0         = 110 = 6 : DIV = 6 (binary divider exponent).

   T3FD = 0x08  (fractional correction word)

   Baud-rate formula (ADuC841 datasheet, Timer 3 section):
     DIV  = floor( log2( fCORE / (16 * Baud) ) )
          = floor( log2( 11 059 200 / 153 600 ) )
          = floor( log2( 72.0 ) )
          = floor( 6.170 )
          = 6

     T3FD = round( (2 * fCORE) / (2^(DIV-1) * Baud) - 64 )
          = round( 22 118 400  / (32 * 9600)  - 64 )
          = round( 22 118 400  / 307 200      - 64 )
          = round( 72.0 - 64 )
          = 8 = 0x08

   Actual baud rate verification (0% error):
     Baud = (2 * fCORE) / (2^(DIV-1) * (T3FD + 64))
          = 22 118 400  / (32 * 72)
          = 22 118 400  / 2304
          = 9600.0 baud  (exact integer result, zero error)
   ---------------------------------------------------------------------------
*/
void RX_Timer3_Init(void)
{
    T3CON &= 0xFEu;    /* Ensure Timer 3 is stopped before configuring */
    T3CON  = 0x86u;    /* T3BAUDEN=1, DIV=6 */
    T3FD   = 0x08u;    /* Fractional correction for exact 9600 baud */
}


/* ---------------------------------------------------------------------------
   RX_UART_Init
   ---------------------------------------------------------------------------
   Configures the UART for 8-bit asynchronous mode (Mode 1), transmit-only,
   9600 baud (clocked by Timer 3).

   Mode 1 (SM0=0, SM1=1): 10-bit frame — 1 start + 8 data + 1 stop.
   REN=0: receiver disabled because this node only transmits to the PC.
   ES=0:  UART interrupt disabled; transmissions are polled via TI.

   FIX — TI pre-arm:
     SCON.TI resets to 0 and no prior transmission has occurred, so the
     very first call to rx_send_uart_byte() would block forever in
     while(!TI) waiting for a flag that is never set.
     Writing TI=1 here acts as if a prior empty transmission just finished,
     which is the correct 8051 UART startup idiom.
   ---------------------------------------------------------------------------
*/
void RX_UART_Init(void)
{
    SM0 = 0;    /* Mode 1: 8-bit UART (bit 1 of SM0/SM1 pair) */
    SM1 = 1;    /* Mode 1: variable baud rate                  */
    REN = 0;    /* Receiver disabled — TX only                 */
    RI  = 0;    /* Clear receive interrupt flag                */
    TI  = 0;    /* Clear transmit interrupt flag               */
    ES  = 0;    /* Disable serial interrupt                    */

    /* Pre-arm: allow rx_send_uart_byte() to load SBUF on its first call
       without stalling.  See note above. */
    TI  = 1;
}


/* ---------------------------------------------------------------------------
   RX_Port_Init
   ---------------------------------------------------------------------------
   Configures the I/O pins used for the Stage 1 bus interface.

   Port 0 (P0.0-P0.7) — bus state bits 0-7:
     Standard 8051 open-drain port.  Writing 0xFF configures pins as
     high-impedance inputs, ready to be driven by the Stage 1 outputs.

   Port 1 (P1.0-P1.6) — bus state bits 8-14:
     ADuC841 ANOMALY: Port 1 defaults to ANALOG INPUT mode on reset.
     Unlike all other 8051-family ports, writing a 1 to a P1 bit KEEPS it
     in analog mode (electrically isolated from the digital core).
     Writing a 0 switches the pin to DIGITAL INPUT mode.
     P1 &= 0x80 clears bits 0-6 (digital input for bus lines 8-14)
     while leaving P1.7 (ADC7) in its default analog state.

   Port 3 bit 2 (P3.2 / INT0) — DATA_READY:
     Port 3 is bidirectional with internal pull-ups.  P3.2 defaults to an
     input after reset; no explicit configuration is required for polling.
   ---------------------------------------------------------------------------
*/
void RX_Port_Init(void)
{
    /* P0.0-P0.7 as digital inputs for bus bits 0-7. */
    P0 = 0xFFu;

    /* P1.0-P1.6 as DIGITAL inputs for bus bits 8-14.
       Write 0 to enable digital mode (see ADuC841 anomaly note above). */
    P1 &= 0x80u;

    /* P3.2 (DATA_READY / INT0) requires no initialisation for polling use.
       To switch to interrupt-driven mode, configure IT0 and EX0 in main(). */
}


/* ---------------------------------------------------------------------------
   rx_read_bus_state
   ---------------------------------------------------------------------------
   Reads one corrected 15-bit bus state from Stage 1 via the parallel port.

   Hardware mapping (default, modify as needed):
     P0 [7:0]   -> bus state bits  [7:0]
     P1 [6:0]   -> bus state bits [14:8]

   The mask ensures bit 15 is always zero even if P1.7 is floating.
   ---------------------------------------------------------------------------
*/
uint16_t rx_read_bus_state(void)
{
    uint16_t bus_state;

    bus_state  = (uint16_t)P0;                          /* Bits  7:0  */
    bus_state |= ((uint16_t)(P1 & 0x7Fu)) << 8;         /* Bits 14:8  */

    return (bus_state & BUS_STATE_MASK);
}


/* ---------------------------------------------------------------------------
   rx_send_uart_byte
   ---------------------------------------------------------------------------
   Transmits one byte to the PC.

   Protocol (Mode 1 UART, ADuC841 datasheet §UART):
     1. Spin until TI=1 (previous transmission complete or pre-armed).
     2. Clear TI.
     3. Write data_byte to SBUF; hardware begins serialisation immediately.
     4. Hardware sets TI at the start of the stop bit when done.

   Timing at 9600 baud:
     1 frame = 10 bits  ->  approx. 1.04 ms per byte.
   ---------------------------------------------------------------------------
*/
void rx_send_uart_byte(uint8_t data_byte)
{
    while (!TI)
    {
        /* Busy-wait: previous byte still being serialised. */
    }

    TI   = 0;
    SBUF = data_byte;
}
