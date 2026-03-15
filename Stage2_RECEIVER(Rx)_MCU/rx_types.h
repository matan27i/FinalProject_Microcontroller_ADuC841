/* =============================================================================
   File: rx_types.h
   Module: Shared types and H1-bus constants

   Purpose:
     Provides portable integer typedefs and the Hamming-bus geometry
     constants used by every module in the receiver system.

   Portability:
     This file deliberately contains NO hardware-specific includes.
     When porting to a platform that supplies <stdint.h> (ANSI C99+),
     delete the two typedefs below and replace them with:
         #include <stdint.h>
     No other change is needed.
   =============================================================================
*/

#ifndef RX_TYPES_H
#define RX_TYPES_H

/* ---------------------------------------------------------------------------
   Portable integer types (substitute for <stdint.h> on Keil C51)
   ---------------------------------------------------------------------------
*/
typedef unsigned char  uint8_t;
typedef unsigned int   uint16_t;

/* ---------------------------------------------------------------------------
   H1-type Hamming bus geometry
   ---------------------------------------------------------------------------
   The H1-type parity-check matrix H has dimensions R x N where:

     R = number of syndrome bits (rows)
     N = number of bus lines (columns) = 2^R - 1

   Column i (1-indexed, i = 1 .. N) of H is the R-bit binary
   representation of i.  This structure means that the syndrome
   S = H * x^T equals the XOR sum of all column indices for which
   the corresponding bit in the bus vector x is set.  That is the
   entire decode algorithm in one sentence.

   With R = 4:
     N           = 2^4 - 1 = 15   (15-bit physical bus)
     BUS_STATE_MASK = 0x7FFF       (isolates bits 0..14; zeros any noise
                                    on bit 15 that a wide read might produce)
   ---------------------------------------------------------------------------
*/
#define HAMMING_R        4
#define HAMMING_N        ((1 << HAMMING_R) - 1)    /* 15 */
#define BUS_STATE_MASK   ((uint16_t)0x7FFFu)

#endif /* RX_TYPES_H */
