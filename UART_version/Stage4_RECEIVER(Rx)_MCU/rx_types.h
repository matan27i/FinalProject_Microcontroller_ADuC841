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
   RX_DEBUG -- compile-time toggle for UART trace markers
   ---
   When 1 (default), the RX emits single-character status markers on
   TxD as the pipeline runs, so PuTTY shows whether the link is alive
   and which stage is accepting/rejecting frames:
       RX-READY\r\n  boot ok, UART TX path alive
       F             CRC-valid frame arrived from the Mega
       C             UART CRC mismatch (bridge link glitched)
       R             bus-reset frame ({0,0,0,0,0}) detected
       U             PESEC SEC+DED flagged an uncorrectable double error
       d             PESEC corrected a single data-bit error
       r             PESEC corrected a single redundancy-bit error
       p             PESEC corrected the overall parity bit itself
   Decoded payload bytes are emitted in the same stream, so running with
   debug on gives a trace like "FFHFFiFF!" for input "Hi!".  Set to 0 to
   get only decoded payload on TxD.
*/
#ifndef RX_DEBUG
#define RX_DEBUG 0
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
#define PESEC_TOTAL_BITS    (HAMMING_N + PESEC_RED_BITS + 1)   /* 26*/
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
#define PESEC_CORRECTED_PARITY  4   /* The parity bit itself was flipped        */

#endif 
