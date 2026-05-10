/* 
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
   
*/

#ifndef RX_TYPES_H
#define RX_TYPES_H

/*
   UART TRACE MARKERS
   ---
   Two tiers of markers are emitted on TxD alongside decoded payload:

   ALWAYS-ON ECC events (sent regardless of RX_DEBUG -- see rx_signal_error
   in rx_hw.c).  Each is a 2-byte marker '!' + kind, chosen so the leading
   '!' stands out in printable-ASCII payload streams:
       !U   PESEC uncorrectable (double error, frame dropped + resync)
       !d   PESEC corrected single data-bit error
       !r   PESEC corrected single redundancy-bit error
       !p   PESEC corrected the overall parity bit itself
       !H   H1 differential layer reconstructed the bus state

   RX_DEBUG-gated link-trace markers (single character, only when this
   macro is 1):
       RX-READY\r\n  boot OK, UART TX path alive
       F             CRC-valid frame arrived from the Mega
       C             UART CRC mismatch (bridge link glitched, ISR-side)
       R             bus-reset frame ({0,0,0,0,0}) detected

   Decoded payload bytes are interleaved in the same stream.  With debug
   on, an input "Hi!" produces a trace like "FFHFFiFF!" -- the 'F' before
   each payload byte confirms the frame pair that produced it.  Set
   RX_DEBUG to 0 to get only decoded payload + always-on ECC markers.

   In addition to the UART markers, an onboard error-indicator LED on
   P3.4 (active-low by default) is lit on any error event and cleared
   on a frame that passes entirely cleanly -- see rx_hw.c.
*/
#ifndef RX_DEBUG
#define RX_DEBUG 1
#endif

/*
   Portable integer types (substitute for <stdint.h> on Keil C51)

*/
typedef unsigned char  uint8_t;
typedef unsigned int   uint16_t;

/* 
   H1-type Hamming bus geometry
   
*/
#define HAMMING_R        4
#define HAMMING_N        ((1 << HAMMING_R) - 1)    /* 15 */
#define BUS_STATE_MASK   ((uint16_t)0x7FFFu)

/* 
   PESEC Error Correction parameters
   
   [W4] PESEC_TOTAL_BITS updated from 25 to 26 to include the overall
   parity bit for SEC+DED.
   
*/
#define PESEC_RED_BITS      10
#define PESEC_RED_MASK      ((uint16_t)0x03FFu)
#define PESEC_TOTAL_BITS    (HAMMING_N + PESEC_RED_BITS + 1)   /* 26: +1 parity */
#define MAX_PESEC_BLOCKS    5

/* 
   PESEC correction result codes (returned by rx_pesec_correct)
   
   Added PESEC_CORRECTED_PARITY for the case where only the overall
   parity bit itself flipped (syndrome == 0 but parity mismatch).
*/
#define PESEC_NO_ERROR          0   /* Syndrome zero, parity OK: no error       */
#define PESEC_CORRECTED_DATA    1   /* Single-bit error corrected in data       */
#define PESEC_CORRECTED_RED     2   /* Single-bit error corrected in redundancy */
#define PESEC_UNCORRECTABLE     3   /* Multi-bit error, correction failed       */
#define PESEC_CORRECTED_PARITY  4   /* [W4] The parity bit itself was flipped   */

#endif 
