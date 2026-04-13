/* =============================================================================
   File: rx_system.h
   Role: Master system header

   Every .c file in the project includes exactly this one header.
   It provides:
     1. The MCU-specific device header (hardware register definitions).
     2. Board-level pin assignments for the 25-bit bus interface.
     3. All three module headers, in dependency order.

   Porting checklist:
     - Replace <aduc841.h> with your target MCU's device header.
     - Redefine DATA_READY to match your hardware wiring.
     - Redefine the port mappings if you use a different bus layout.
     - No other file needs to be touched for a basic port.
   =============================================================================
*/

#ifndef RX_SYSTEM_H
#define RX_SYSTEM_H

/* ---------------------------------------------------------------------------
   Target MCU device header
   Provides: SFR declarations (P0, P1, P2, P3, SBUF, SCON bits, T3CON,
             T3FD, EA, TI, RI, SM0, SM1, REN, ES), and the 'sbit'/'bit'
             keywords.
   Replace this line when porting to a different MCU.
   ---------------------------------------------------------------------------
*/
#include <aduc841.h>

/* ---------------------------------------------------------------------------
   Board-level hardware definitions
   ---------------------------------------------------------------------------

   DATA_READY (P3.2 / INT0):
     Active-high signal driven by Stage 1 to indicate that a new 25-bit
     word (15 data + 10 redundancy) is ready on the parallel bus.
     The main loop uses rising-edge detection on this pin.

   25-BIT BUS PORT MAPPING:
   ~~~~~~~~~~~~~~~~~~~~~~~~
   The TX encoder outputs a 25-bit word through daisy-chained 74HC595
   shift registers.  The RX reads these bits via four port groups:

     Port          Bits     Bus content
     ~~~~~~~~~~~~  ~~~~~~~  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
     P0 [7:0]      8 bits   H1 data bits  [7:0]
     P1 [6:0]      7 bits   H1 data bits  [14:8]
     P2 [7:0]      8 bits   PESEC redundancy bits [7:0]
     P3.4, P3.3    2 bits   PESEC redundancy bits [9:8]

   Total: 8 + 7 + 8 + 2 = 25 bits.

   P3.3 (INT1) and P3.4 (T0) are general-purpose I/O pins that are not
   used by any other peripheral in this design.  P3.0 (RXD) and P3.1
   (TXD) are reserved for UART communication with the PC.
   ---------------------------------------------------------------------------
*/
sbit DATA_READY = P3^2;

/* Symbolic port aliases for the bus interface.
   Modify these defines (and the read logic in rx_hw.c) when rewiring. */
#define BUS_LOW_PORT     P0     /* H1 data bits  [7:0]                  */
#define BUS_HIGH_PORT    P1     /* H1 data bits [14:8] on P1.0-P1.6    */
#define ECC_LOW_PORT     P2     /* PESEC redundancy bits [7:0]          */

/* Individual bit pins for PESEC redundancy bits 8 and 9.
   Using P3.3 and P3.4 which are free on the RX board. */
sbit ECC_BIT8 = P3^3;          /* PESEC redundancy bit 8               */
sbit ECC_BIT9 = P3^4;          /* PESEC redundancy bit 9               */

/* ---------------------------------------------------------------------------
   Module headers -- include order follows the dependency chain:
     rx_types   (no deps)
     rx_hw      (uses rx_types)
     rx_ecc     (uses rx_types, calls rx_decoder for H1 syndrome)
     rx_decoder (uses rx_types, calls rx_hw)
   ---------------------------------------------------------------------------
*/
#include "rx_types.h"
#include "rx_hw.h"
#include "rx_ecc.h"
#include "rx_decoder.h"

#endif /* RX_SYSTEM_H */
