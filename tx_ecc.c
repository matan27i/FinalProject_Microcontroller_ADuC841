/* File: ecc.c
 * Group 3: PESEC Error Correction
 * ================================
 * Target: ADuC841 (8052 single-cycle core)
 *
 * Contains:
 *   - pesec_redundancy_reg: the 10-bit ECC redundancy vector
 *   - Dynamic matrix storage (D and A) and block solver metadata
 *   - Init_PESEC_Matrices(): builds D and A from block-size config
 *   - Syndrome helpers and delta solver
 *   - ECC(): main encode cycle — differential redundancy update + output
 *
 * PESEC ENCODER OVERVIEW:
 *   The encoder maintains a redundancy register R such that [x | R] is
 *   always a valid codeword.  When the data vector x changes, a
 *   "syndrome gap" is computed and resolved into a minimal set of bit
 *   flips (delta_vector) applied to R via XOR.
 *
 * Portability: Pure algorithmic logic.  The only external call is
 *              output_to_shift_registers() from Group 1.
 */

#include "tx_header.h"

/* =========================================================================
 * STATE VARIABLE DEFINITIONS  (owned by this module)
 * ========================================================================= */

/* 10-bit redundancy register, updated differentially.
 * [N2] Not volatile: only accessed from main-loop call chain. */
uint16_t pesec_redundancy_reg = 0;

/* Dynamic matrix storage */
uint8_t PESEC_MAT_D[20];   /* Redundancy-bit columns */
uint8_t PESEC_MAT_A[40];   /* Data-bit columns       */

/* Block solver metadata */
uint8_t pesec_num_blocks;
uint8_t pesec_bit_offsets[MAX_PESEC_BLOCKS];
uint8_t pesec_col_offsets[MAX_PESEC_BLOCKS];
uint8_t pesec_chunk_masks[MAX_PESEC_BLOCKS];

/* Column counts */
uint8_t pesec_num_d_cols;
uint8_t pesec_num_a_cols;

/* =========================================================================
 * MATRIX INITIALISATION
 * =========================================================================
 * Builds matrices D (redundancy columns) and A (data columns) from an
 * array of block sizes.  Also pre-computes per-block masks and offsets
 * for O(1) delta solving.
 *
 * For config {m1, m2, ...}:
 *   Block b contributes (2^mb - 1) columns to D.
 *   A = syndrome-space values NOT present in D.
 * ========================================================================= */
void Init_PESEC_Matrices(uint8_t* m_sizes, uint8_t num_blocks)
{
    uint8_t i, b, val;
    uint8_t is_in_d;
    uint8_t total_m = 0;
    uint8_t current_col_offset = 0;
    uint8_t max_syndrome;

    pesec_num_blocks = num_blocks;
    pesec_num_d_cols = 0;
    pesec_num_a_cols = 0;

    /* --- Build Matrix D and pre-calculate solver metadata --- */
    for (b = 0; b < num_blocks; b++)
    {
        pesec_bit_offsets[b] = total_m;
        pesec_col_offsets[b] = current_col_offset;
        pesec_chunk_masks[b] = ((1 << m_sizes[b]) - 1) << total_m;

        /* Each block contributes columns 1..(2^m - 1) shifted by total_m */
        for (i = 1; i < (1 << m_sizes[b]); i++)
        {
            PESEC_MAT_D[pesec_num_d_cols++] = (i << total_m);
        }

        total_m += m_sizes[b];
        current_col_offset += ((1 << m_sizes[b]) - 1);
    }

    max_syndrome = (1 << total_m) - 1;

    /* --- Build Matrix A: everything in syndrome space NOT in D --- */
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
 * SYNDROME COMPUTATION HELPERS
 * ========================================================================= */

/* Generic syndrome: XOR matrix columns selected by set bits in reg_val */
static uint8_t calc_pesec_synd(uint16_t reg_val, uint8_t* mat_ptr, uint8_t cols)
{
    uint8_t synd_res = 0;
    uint8_t idx;

    for (idx = 0; idx < cols; idx++)
    {
        if ((reg_val >> idx) & 1)
        {
            synd_res ^= mat_ptr[idx];
        }
    }
    return synd_res;
}

/* Syndrome of data bits via Matrix A */
static uint8_t get_data_pesec_synd(uint16_t data_val)
{
    return calc_pesec_synd(data_val, PESEC_MAT_A, pesec_num_a_cols);
}

/* Syndrome of redundancy bits via Matrix D */
static uint8_t get_red_pesec_synd(uint16_t red_val)
{
    return calc_pesec_synd(red_val, PESEC_MAT_D, pesec_num_d_cols);
}

/* =========================================================================
 * DELTA SOLVER
 * =========================================================================
 * Converts a syndrome gap into the minimal set of redundancy-bit flips.
 * Iterates through each H1-type sub-block, extracts its chunk of the
 * syndrome gap, and maps it to a single bit position in the redundancy
 * register.  O(num_blocks) — effectively O(1) for small block counts.
 * ========================================================================= */
static uint16_t solve_pesec_delta(uint8_t synd_gap)
{
    uint16_t delta_vec = 0;
    uint8_t  b, chunk, val;

    for (b = 0; b < pesec_num_blocks; b++)
    {
        chunk = synd_gap & pesec_chunk_masks[b];

        if (chunk != 0)
        {
            val = chunk >> pesec_bit_offsets[b];
            delta_vec |= (1 << (pesec_col_offsets[b] + val - 1));
        }
    }

    return delta_vec;
}

/* =========================================================================
 * MAIN ECC FUNCTION
 * =========================================================================
 * Called after each nibble encoding cycle in process_nibble().
 *
 * Steps:
 *   1. Compute syndrome of data bits (Matrix A × current_bus_state)
 *   2. Compute syndrome of redundancy bits (Matrix D × pesec_redundancy_reg)
 *   3. XOR to find the syndrome gap
 *   4. Solve for the optimal redundancy-bit flip vector
 *   5. Apply the differential update to pesec_redundancy_reg
 *   6. Output all 25 bits to shift registers
 * ========================================================================= */
void ECC(void)
{
    uint8_t  synd_data;
    uint8_t  synd_red;
    uint8_t  synd_diff;
    uint16_t delta_vector;

    /* Step 1: Syndrome of the new data */
    synd_data = get_data_pesec_synd(current_bus_state);

    /* Step 2: Syndrome of the current redundancy */
    synd_red = get_red_pesec_synd(pesec_redundancy_reg);

    /* Step 3: Syndrome gap */
    synd_diff = synd_data ^ synd_red;

    /* Step 4: Solve for minimal redundancy flip */
    delta_vector = solve_pesec_delta(synd_diff);

    /* Step 5: Apply differential update */
    pesec_redundancy_reg ^= delta_vector;

    /* Step 6: Output 25 bits to daisy-chained shift registers */
    output_to_shift_registers();
}
