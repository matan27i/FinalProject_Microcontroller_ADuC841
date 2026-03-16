/* =============================================================================
   File: rx_ecc.h
   Module: Group 3 -- Error Correction (PESEC + H1)
   Public interface and extern declarations.

   This module provides two layers of error correction:

     Layer 1 -- PESEC (25-bit codeword):
       Uses dynamically-built matrices A (data columns) and D (redundancy
       columns) to compute a syndrome across the full 25-bit word, locate
       a single-bit error, and correct it.  This is the PRIMARY error
       correction mechanism matching the TX encoder's PESEC output.

     Layer 2 -- H1 differential (15-bit data):
       After PESEC correction, the clean 15-bit data passes through a
       stateful differential tracker that verifies the transition from
       the previous bus state uses at most one bit flip.  This serves
       as a secondary validation layer and maintains the decode baseline.

   The module is self-contained with no hardware dependencies.  Its only
   external call is rx_compute_syndrome() from the decoder module (for the
   H1 layer), and even that is pure arithmetic.
   =============================================================================
*/

#ifndef RX_ECC_H
#define RX_ECC_H

#include "rx_types.h"

/* =========================================================================
   PESEC MATRIX STORAGE AND METADATA
   =========================================================================
   These are the receiver-side copies of the TX encoder's PESEC matrices.
   They are built at startup by rx_init_pesec_matrices() using the same
   block configuration as the TX (e.g., {3, 2}).

   Matrix D (pesec_num_d_cols columns):
     Each column is the syndrome-space value for one redundancy bit.
     For config {3, 2}: D = {1,2,3,4,5,6,7,  8,16,24}  (10 columns).

   Matrix A (pesec_num_a_cols columns):
     Each column is a syndrome-space value NOT present in D.
     For config {3, 2}: A has 21 columns (values 9..15, 17..23, 25..31).
     Only the first 15 columns correspond to physical data bits;
     columns A[15]..A[20] map to virtual bit positions that are always 0.
   ========================================================================= */
extern uint8_t xdata PESEC_MAT_D[20];     /* Redundancy-bit columns (max 20)  */
extern uint8_t xdata PESEC_MAT_A[40];     /* Data-bit columns (max 40)        */

extern uint8_t xdata pesec_num_blocks;                        /* Active block count        */
extern uint8_t xdata pesec_bit_offsets[MAX_PESEC_BLOCKS];     /* Syndrome bit offset/block */
extern uint8_t xdata pesec_col_offsets[MAX_PESEC_BLOCKS];     /* Column offset in D/block  */
extern uint8_t xdata pesec_chunk_masks[MAX_PESEC_BLOCKS];     /* Syndrome chunk mask/block */

extern uint8_t xdata pesec_num_d_cols;    /* Number of columns in Matrix D */
extern uint8_t xdata pesec_num_a_cols;    /* Number of columns in Matrix A */

/* =========================================================================
   H1 DIFFERENTIAL STATE
   ========================================================================= */

/* Last accepted (error-corrected) 15-bit bus state.
   Initialised to 0 to match the TX encoder's starting state.
   [N1] volatile removed: no ISRs access these; main-loop only.
   [MEM] XDATA: saves 8 bytes of DATA. */
extern uint16_t xdata rx_previous_bus_state;

/* Running count of single-bit corrections applied by the H1 layer. */
extern uint16_t xdata rx_errors_corrected;

/* Placeholder for future multi-bit error accounting. */
extern uint16_t xdata rx_errors_detected;

/* Running count of PESEC corrections applied. */
extern uint16_t xdata rx_pesec_corrections;

/* =========================================================================
   PESEC INITIALISATION
   =========================================================================
   rx_init_pesec_matrices
   ---------------------------------------------------------------------------
   Builds matrices D and A from an array of block sizes, exactly mirroring
   the TX encoder's Init_PESEC_Matrices().  Must be called once at startup
   with the SAME configuration the TX uses.

   For the TX config {3, 2}:
     Block 0 (m=3): D columns = {1,2,3,4,5,6,7}, syndrome bits [2:0]
     Block 1 (m=2): D columns = {8,16,24},        syndrome bits [4:3]
     Total syndrome space: 5 bits (0..31)
     D: 10 columns, A: 21 columns

   Parameters:
     m_sizes     Array of block sizes (e.g., {3, 2}).
     num_blocks  Number of elements in m_sizes.
   ---------------------------------------------------------------------------
*/
void rx_init_pesec_matrices(uint8_t *m_sizes, uint8_t num_blocks);

/* =========================================================================
   PESEC ERROR CORRECTION
   =========================================================================
   rx_pesec_correct
   ---------------------------------------------------------------------------
   Performs single-bit error correction on a 25-bit received word using
   the PESEC parity-check matrix [A | D].

   Algorithm:
     1. Compute the PESEC syndrome:
          S = (XOR of A[i] for each set data bit i)
              XOR (XOR of D[j] for each set redundancy bit j)
     2. If S = 0: no error, the codeword is valid.
     3. If S != 0: search for the column that matches S:
        a. If S == A[i] for some i < 15: error is in data bit i.
           Flip that bit in *data_bits.
        b. If S == D[j] for some j < 10: error is in redundancy bit j.
           Flip that bit in *red_bits.
        c. If no match among physical bits: uncorrectable (multi-bit error).

   Why this works:
     The PESEC code is constructed so that every column of [A | D] is a
     unique non-zero value in the 5-bit syndrome space.  A single-bit
     error in position k adds column k to the syndrome (XOR is self-
     inverse), producing a non-zero syndrome equal to that column.
     Matching the syndrome to a column uniquely identifies the errored bit.

   Parameters:
     data_bits   Pointer to the 15-bit data word; corrected in place.
     red_bits    Pointer to the 10-bit redundancy word; corrected in place.

   Returns:
     PESEC_NO_ERROR        (0) -- syndrome was zero, no correction needed.
     PESEC_CORRECTED_DATA  (1) -- single-bit error found and fixed in data.
     PESEC_CORRECTED_RED   (2) -- single-bit error found and fixed in redund.
     PESEC_UNCORRECTABLE   (3) -- syndrome non-zero but no matching column.
   ---------------------------------------------------------------------------
*/
uint8_t rx_pesec_correct(uint16_t *data_bits, uint16_t *red_bits);

/* =========================================================================
   H1 DIFFERENTIAL ERROR CORRECTION
   =========================================================================
   find_minimal_w
   ---------------------------------------------------------------------------
   Returns the unique minimum-Hamming-weight 15-bit vector w such that
       H * w^T  =  s_target
   using the H1-type matrix structure (column i = value i).

   For s_target = 0: returns 0 (no transition).
   For s_target in {1..15}: returns (1 << (s_target - 1)).
   ---------------------------------------------------------------------------
*/
uint16_t find_minimal_w(uint8_t s_target);

/* ---------------------------------------------------------------------------
   rx_correct_bus_state
   ---------------------------------------------------------------------------
   Applies H1 differential validation to one 15-bit bus state.

   After PESEC correction, the data bits should be clean.  This function
   verifies the transition from rx_previous_bus_state and updates the
   differential baseline.  If the transition weight exceeds 1 (which
   should be rare after PESEC), it reconstructs the expected state.

   Parameters:
     corrected_data  15-bit data from PESEC correction stage.
   Returns:
     15-bit validated bus state; also updates rx_previous_bus_state.
   ---------------------------------------------------------------------------
*/
uint16_t rx_correct_bus_state(uint16_t corrected_data);

#endif /* RX_ECC_H */
