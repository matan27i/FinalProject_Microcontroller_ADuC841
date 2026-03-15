/* =============================================================================
   File: rx_system.h
   Role: Master system header

   Every .c file in the project includes exactly this one header.
   It provides:
     1. The MCU-specific device header (hardware register definitions).
     2. Board-level pin assignments.
     3. All three module headers, in dependency order.

   Porting checklist:
     - Replace <aduc841.h> with your target MCU's device header.
     - Redefine DATA_READY to match your hardware wiring.
     - Optionally redefine BUS_LOW_PORT / BUS_HIGH_PORT if you use a
       different port mapping in rx_hw.c.
     - No other file needs to be touched for a basic port.
   =============================================================================
*/

#ifndef RX_SYSTEM_H
#define RX_SYSTEM_H

/* ---------------------------------------------------------------------------
   Target MCU device header
   Provides: SFR declarations (P0, P1, P3, SBUF, SCON bits, T3CON, T3FD,
             EA, TI, RI, SM0, SM1, REN, ES), and the 'sbit' / 'bit' keywords.
   Replace this line when porting to a different MCU.
   ---------------------------------------------------------------------------
*/
#include <aduc841.h>

/* ---------------------------------------------------------------------------
   Board-level hardware definitions
   ---------------------------------------------------------------------------
   DATA_READY (P3.2 / INT0):
     Active-high signal driven by Stage 1 to indicate that a new corrected
     15-bit bus state is ready to be read from the parallel port.
     The main loop uses rising-edge detection on this pin.

   BUS_LOW_PORT / BUS_HIGH_PORT (optional aliases):
     Symbolic names for the ports carrying the 15-bit bus word.
     Used only in rx_hw.c; redefined here for easy reconfiguration.
   ---------------------------------------------------------------------------
*/
sbit DATA_READY    = P3^2;

#define BUS_LOW_PORT    P0      /* Bus state bits  [7:0]  */
#define BUS_HIGH_PORT   P1      /* Bus state bits [14:8] on P1.0-P1.6 */

/* ---------------------------------------------------------------------------
   Module headers — include order follows the dependency chain:
     rx_types  (no deps)
     rx_hw     (uses rx_types)
     rx_decoder(uses rx_types, calls rx_hw)
     rx_ecc    (uses rx_types, calls rx_decoder)
   ---------------------------------------------------------------------------
*/
#include "rx_types.h"
#include "rx_hw.h"
#include "rx_decoder.h"
#include "rx_ecc.h"

#endif /* RX_SYSTEM_H */
