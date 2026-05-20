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
   All UART markers are gated by a LATCHED debug-mode flag
   (rx_debug_mode_active in rx_hw.c) that mirrors the P3.5 strap pin.
   The flag is sampled only when the host sends the '~' command -- the
   strap state at that moment is captured and held until the next '~'.

   Why latched: the main loop and UART_ISR check the flag for every
   frame and every CRC mismatch respectively; reading a cached xdata
   byte (~3 cycles) is cheaper than calling rx_debug_active() (~10
   cycles), and the flag cannot accidentally flip from line noise on
   P3.5 between explicit '~' commands.

   How to use:
     1. Tie P3.5 to GND with a jumper (or leave it floating for OFF).
     2. From the host, send '~' (followed by '\n' if you're using
        host_sender.py's batch mode).  The byte echoes to PuTTY as
        '~' and the strap is captured into rx_debug_mode_active.
     3. The very next frame's markers reflect the new mode.
     4. To change mode later, move the jumper and send '~' again.

   In normal operation the TxD stream contains ONLY decoded payload
   bytes -- the line is silent except for real data.  In debug mode:

       RX-READY\r\n  boot OK, UART TX path alive (sent unconditionally
                     during boot before the strap is first read)

   Link-trace markers (single characters, strap-gated):
       F             CRC-valid frame arrived from the Mega
       C             UART CRC mismatch (bridge link glitched, ISR-side)
       R             bus-reset frame ({0,0,0,0,0}) detected

   ECC event markers (2-byte '!' + kind, strap-gated):
       !U            PESEC uncorrectable (double error, frame dropped)
       !d            PESEC corrected single data-bit error
       !r            PESEC corrected single redundancy-bit error
       !p            PESEC corrected the overall parity bit itself

   The H1 differential layer (and its '!H' marker) were removed in a
   previous revision.

   With the strap low, input "Hi!" produces a trace like "FFHFFiFF!" --
   the 'F' before each payload byte confirms the frame pair that
   produced it; '!d' etc. appear inline when PESEC corrects something.
   The strap is sampled once per frame in the main loop (plus once per
   CRC-mismatch event in UART_ISR), so toggling the external strap
   reflects in the trace within at most one frame.

   The onboard error LED on P3.4 (active-low by default) is the ONLY
   always-on indicator: lit on any error event regardless of strap
   state, cleared on a frame that passes entirely cleanly.  This means
   the LED can be used as a "did anything go wrong?" indicator without
   running PuTTY at all.
*/

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
*/
#define PESEC_RED_BITS      10
#define PESEC_RED_MASK      ((uint16_t)0x03FFu)
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
