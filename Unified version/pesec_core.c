/* =============================================================================
   File: pesec_core.c
   Module: Shared PESEC Error-Correction Core (DRY refactor)
   =============================================================================

   Purpose:
     Single implementation of every PESEC algorithmic primitive shared
     between the TX encoder and the RX decoder:
       - Matrix construction (D and A sub-matrices)
       - Generic syndrome computation
       - Data-syndrome and redundancy-syndrome wrappers
       - Overall even-parity calculation (SEC+DED, W4 upgrade)

   Build notes:
     Both the TX and RX projects compile and link this file.
     The RX project should define PESEC_USE_XDATA (via compiler flag
     or a pre-include) so that the module-owned arrays are placed in
     XDATA to conserve the 8051 DATA segment.

   Portability:
     No hardware registers or vendor headers are referenced here.
   =============================================================================
*/

#include "pesec_core.h"

/* =========================================================================
   MODULE STATE VARIABLE DEFINITIONS
   =========================================================================
   These are the single, authoritative instances of the PESEC matrices
   and block-solver metadata.  Both TX and RX link against them.
   ========================================================================= */

/* Dynamic matrix storage */
uint8_t PESEC_MEM PESEC_MAT_D[20];   /* Redundancy-bit columns */
uint8_t PESEC_MEM PESEC_MAT_A[40];   /* Data-bit columns       */

/* Block solver metadata */
uint8_t PESEC_MEM pesec_num_blocks;
uint8_t PESEC_MEM pesec_bit_offsets[MAX_PESEC_BLOCKS];
uint8_t PESEC_MEM pesec_col_offsets[MAX_PESEC_BLOCKS];
uint8_t PESEC_MEM pesec_chunk_masks[MAX_PESEC_BLOCKS];

/* Column counts */
uint8_t PESEC_MEM pesec_num_d_cols;
uint8_t PESEC_MEM pesec_num_a_cols;

/* =========================================================================
   PESEC MATRIX INITIALISATION
   =========================================================================
   Builds the D (redundancy-column) and A (data-column) sub-matrices
   from the supplied block-size configuration.  The algorithm is
   identical on both TX and RX; this single copy replaces the former
   Init_PESEC_Matrices() (TX) and rx_init_pesec_matrices() (RX).

   Parameters:
     m_sizes     Array of sub-block bit widths (e.g. {3, 2}).
     num_blocks  Number of entries in m_sizes.
   ========================================================================= */
void pesec_init_matrices(uint8_t *m_sizes, uint8_t num_blocks)
{
    uint8_t i, b, val;
    uint8_t is_in_d;
    uint8_t total_m = 0;
    uint8_t current_col_offset = 0;
    uint8_t max_syndrome;

    pesec_num_blocks = num_blocks;
    pesec_num_d_cols = 0;
    pesec_num_a_cols = 0;

    for (b = 0; b < num_blocks; b++)
    {
        pesec_bit_offsets[b] = total_m;
        pesec_col_offsets[b] = current_col_offset;
        pesec_chunk_masks[b] = ((1 << m_sizes[b]) - 1) << total_m;

        for (i = 1; i < (1 << m_sizes[b]); i++)
        {
            PESEC_MAT_D[pesec_num_d_cols++] = (i << total_m);
        }

        total_m += m_sizes[b];
        current_col_offset += ((1 << m_sizes[b]) - 1);
    }

    max_syndrome = (1 << total_m) - 1;

    for (val = 1; val <= max_syndrome; val++)
    {
        is_in_d = 0;

        for (i = 0; i < pesec_num_d_cols; i++)
        {
            if (PESEC_MAT_D[i] == val)
            {
                is_in_d = 1;
                break;
            }
        }

        if (is_in_d == 0)
        {
            PESEC_MAT_A[pesec_num_a_cols++] = val;
        }
    }
}

/* =========================================================================
   GENERIC SYNDROME COMPUTATION (INTERNAL HELPER)
   =========================================================================
   Computes syndrome = XOR of all matrix columns whose corresponding
   bit in reg_val is set.  Used by get_data_syndrome() and
   get_red_syndrome().
   ========================================================================= */
static uint8_t calc_pesec_syndrome(uint16_t reg_val,
                                   uint8_t *mat_ptr,
                                   uint8_t  num_cols)
{
    uint8_t syndrome = 0;
    uint8_t idx;

    for (idx = 0; idx < num_cols; idx++)
    {
        if ((reg_val >> idx) & 1u)
        {
            syndrome ^= mat_ptr[idx];
        }
    }

    return syndrome;
}

/* =========================================================================
   DATA-BIT SYNDROME  (Matrix A x data_val)
   ========================================================================= */
uint8_t get_data_syndrome(uint16_t data_val)
{
    return calc_pesec_syndrome(data_val, PESEC_MAT_A, pesec_num_a_cols);
}

/* =========================================================================
   REDUNDANCY-BIT SYNDROME  (Matrix D x red_val)
   ========================================================================= */
uint8_t get_red_syndrome(uint16_t red_val)
{
    return calc_pesec_syndrome(red_val, PESEC_MAT_D, pesec_num_d_cols);
}

/* =========================================================================
   OVERALL PARITY COMPUTATION  (W4 -- SEC+DED)
   =========================================================================
   Computes the population parity (popcount mod 2) of a 16-bit value
   using XOR folding.  Optimised for the 8051 instruction set:
     1. XOR the high byte into the low byte (one 8-bit XOR).
     2. Fold the resulting byte: nibbles, pairs, then single bit.
   Total: 5 XORs + 3 shifts + 1 AND -- no branches, no lookup table.

   Returns: 0 if the number of set bits is even, 1 if odd.
   ========================================================================= */
uint8_t calculate_overall_parity(uint16_t v)
{
    uint8_t x;

    x = (uint8_t)(v >> 8) ^ (uint8_t)(v);
    x ^= x >> 4;
    x ^= x >> 2;
    x ^= x >> 1;

    return (x & 1u);
}
