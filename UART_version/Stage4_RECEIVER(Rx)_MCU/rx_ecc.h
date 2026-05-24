/*
   File: rx_ecc.h
   Module: Group 3  Error Correction (PESEC SEC+DED)
   Public interface and extern declarations.

   This module provides the bus-level error-protection layer:

     PESEC (25-bit codeword):
       Single-bit error correction via syndrome column matching.

     SEC+DED (26-bit word):
       The overall parity bit (26th bit) is combined with the PESEC
       syndrome to detect double-bit errors that SEC alone would
       silently miscorrect.  Double errors cause the frame to be
       dropped by the main loop.

   The H1-type differential validation layer was removed in this
   revision.  Nibble decoding is handled separately by absolute
   syndrome computation in rx_decoder.c.

*/

#ifndef RX_ECC_H
#define RX_ECC_H

#include "rx_types.h"

/* 
   PESEC MATRIX STORAGE AND METADATA
    */
extern uint8_t xdata PESEC_MAT_D[20];
extern uint8_t xdata PESEC_MAT_A[40];

extern uint8_t xdata pesec_num_blocks;
extern uint8_t xdata pesec_bit_offsets[MAX_PESEC_BLOCKS];
extern uint8_t xdata pesec_col_offsets[MAX_PESEC_BLOCKS];
extern uint8_t xdata pesec_chunk_masks[MAX_PESEC_BLOCKS];

extern uint8_t xdata pesec_num_d_cols;
extern uint8_t xdata pesec_num_a_cols;

/*
   PESEC DIAGNOSTIC COUNTERS
   Read-only from the main loop; updated by rx_pesec_correct.
*/
extern uint16_t xdata rx_errors_detected;     /* double-bit / no-match events */
extern uint16_t xdata rx_pesec_corrections;   /* single-bit corrections      */

/* 
   PESEC INITIALISATION
    */
void rx_init_pesec_matrices(uint8_t *m_sizes, uint8_t num_blocks);

/* 
   PESEC ERROR CORRECTION  (SEC+DED UPGRADED)
   rx_pesec_correct
   
   [W4] UPGRADED PROTOTYPE: now accepts the received overall parity bit.

   Performs SEC+DED error handling on the 26-bit received word using
   the PESEC parity-check matrix [A | D] plus the overall parity bit.

   SEC+DED truth table:
     S==0, P_OK   -> PESEC_NO_ERROR        (no error)
     S!=0, P_FAIL -> PESEC_CORRECTED_DATA   or PESEC_CORRECTED_RED
                     (single error, corrected)
     S!=0, P_OK   -> PESEC_UNCORRECTABLE   (double error detected)
     S==0, P_FAIL -> PESEC_CORRECTED_PARITY (parity bit itself flipped)

   Parameters:
     data_bits       Pointer to 15-bit data word; corrected in place.
     red_bits        Pointer to 10-bit redundancy word; corrected in place.
     overall_parity  Received overall parity bit (0 or 1) from 26th bus bit.

   Returns:
     PESEC_NO_ERROR         (0)
     PESEC_CORRECTED_DATA   (1)
     PESEC_CORRECTED_RED    (2)
     PESEC_UNCORRECTABLE    (3)
     PESEC_CORRECTED_PARITY (4)
   
*/
uint8_t rx_pesec_correct(uint16_t *data_bits, uint16_t *red_bits,
                         uint8_t overall_parity);

#endif
