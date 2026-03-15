/* =============================================================================
   File: rx_types.h
   Module: Shared types and system-wide constants

   Purpose:
     Provides portable integer typedefs, the H1-bus geometry constants,
     and the PESEC error-correction parameters used by every module in
     the receiver system.

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
   the corresponding bit in the bus vector x is set.

   With R = 4:
     N              = 2^4 - 1 = 15   (15-bit physical bus)
     BUS_STATE_MASK = 0x7FFF          (isolates bits 0..14)
   ---------------------------------------------------------------------------
*/
#define HAMMING_R        4
#define HAMMING_N        ((1 << HAMMING_R) - 1)    /* 15 */
#define BUS_STATE_MASK   ((uint16_t)0x7FFFu)

/* ---------------------------------------------------------------------------
   PESEC Error Correction parameters
   ---------------------------------------------------------------------------
   The TX encoder appends 10 redundancy bits to the 15 H1 data bits,
   producing a 25-bit word that is clocked out through daisy-chained
   74HC595 shift registers.

   PESEC_RED_BITS   Number of redundancy bits appended by the TX encoder.
   PESEC_RED_MASK   Isolates the valid redundancy bits (bits 0..9).
   PESEC_TOTAL_BITS Total physical bus width: 15 data + 10 redundancy.
   MAX_PESEC_BLOCKS Maximum number of H1-type sub-blocks in the PESEC
                    configuration array.  The TX uses config {3, 2},
                    giving 2 blocks, but the code supports up to 5.

   Block structure for config {m1=3, m2=2}:
     Block 0 (m=3): 2^3 - 1 = 7 redundancy columns, syndrome bits [2:0]
     Block 1 (m=2): 2^2 - 1 = 3 redundancy columns, syndrome bits [4:3]
     Total redundancy columns: 7 + 3 = 10
     Total syndrome width:     3 + 2 = 5 bits  (values 0..31)
   ---------------------------------------------------------------------------
*/
#define PESEC_RED_BITS      10
#define PESEC_RED_MASK      ((uint16_t)0x03FFu)
#define PESEC_TOTAL_BITS    (HAMMING_N + PESEC_RED_BITS)   /* 25 */
#define MAX_PESEC_BLOCKS    5

/* ---------------------------------------------------------------------------
   PESEC correction result codes (returned by rx_pesec_correct)
   ---------------------------------------------------------------------------
*/
#define PESEC_NO_ERROR          0   /* Syndrome zero, no correction needed   */
#define PESEC_CORRECTED_DATA    1   /* Single-bit error corrected in data    */
#define PESEC_CORRECTED_RED     2   /* Single-bit error corrected in redund. */
#define PESEC_UNCORRECTABLE     3   /* Multi-bit error, correction failed    */

#endif /* RX_TYPES_H */
