/* File: ecc.c
 * Group 3: PESEC Error Correction + SEC+DED Overall Parity
 * Target: ADuC841 (8052 single-cycle core)
 *
 * Contains:
 *   - pesec_redundancy_reg: the 10-bit ECC redundancy vector
 *   - pesec_overall_parity: the 26th bit for SEC+DED (W4 upgrade)
 *   - Dynamic matrix storage (D and A) and block solver metadata
 *   - Init_PESEC_Matrices(): builds D and A from block-size config
 *   - Syndrome helpers and delta solver
 *   - ECC(): main encode cycle -- redundancy update + parity + output
 *
 * SEC+DED UPGRADE (W4):
 *   After computing the redundancy, a single overall even-parity bit P
 *   is computed over all 25 bits (15 data + 10 redundancy).  This
 *   expands the physical bus word to 26 bits and upgrades the error
 *   model from SEC to SEC+DED.
 *
 * Portability: Pure algorithmic logic.  The only external call is
 *              output_to_shift_registers() from Group 1.
 */

#include "tx_header.h"

/* 
 * STATE VARIABLE DEFINITIONS  (owned by this module)
 */

/* 10-bit redundancy register, updated differentially.
 * [N2] Not volatile: only accessed from main-loop call chain. */
uint16_t pesec_redundancy_reg = 0;

/* [W4] Overall even-parity bit for SEC+DED.
 * Computed fresh from all 25 bits (15 data + 10 redundancy) after each
 * redundancy update.  Read by output_to_shift_registers() in tx_main.c
 * to shift it out as the 26th bit on the bus.
 * Value: 0 = even number of 1s in the 25-bit word, 1 = odd. */
uint8_t pesec_overall_parity = 0;

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

/* 
 * MATRIX INITIALISATION
 */
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

/* 
 * SYNDROME COMPUTATION HELPERS
 */

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

static uint8_t get_data_pesec_synd(uint16_t data_val)
{
    return calc_pesec_synd(data_val, PESEC_MAT_A, pesec_num_a_cols);
}

static uint8_t get_red_pesec_synd(uint16_t red_val)
{
    return calc_pesec_synd(red_val, PESEC_MAT_D, pesec_num_d_cols);
}

/* =========================================================================
 * PARITY COMPUTATION HELPER  (W4 -- SEC+DED)
 * =========================================================================
 * Computes the population parity (popcount mod 2) of a 16-bit value
 * using XOR folding.  Optimised for the 8051 instruction set:
 *   1. XOR the high byte into the low byte (one 8-bit XOR).
 *   2. Fold the resulting byte: nibbles, pairs, then single bit.
 * Total: 5 XORs + 3 shifts + 1 AND -- no branches, no lookup table.
 *
 * Returns: 0 if the number of set bits is even, 1 if odd.
 * ========================================================================= */
static uint8_t parity16(uint16_t v)
{
    uint8_t x;

    x = (uint8_t)(v >> 8) ^ (uint8_t)(v);
    x ^= x >> 4;
    x ^= x >> 2;
    x ^= x >> 1;

    return (x & 1u);
}

/* 
 * DELTA SOLVER
 */
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

/* 
 * MAIN ECC FUNCTION
 * Called after each nibble encoding cycle in process_nibble().
 *
 * Steps:
 *   1. Compute syndrome of data bits (Matrix A x current_bus_state)
 *   2. Compute syndrome of redundancy bits (Matrix D x pesec_redundancy_reg)
 *   3. XOR to find the syndrome gap
 *   4. Solve for the optimal redundancy-bit flip vector
 *   5. Apply the differential update to pesec_redundancy_reg
 *   6. [W4] Compute overall even parity of all 25 bits
 *   7. Output all 26 bits to shift registers
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

    /* Step 6: [W4] Compute overall even parity of all 25 bits.
     *
     * P = parity(data[14:0]) XOR parity(redundancy[9:0])
     *
     * The parity of the 25-bit concatenation [data | red] equals the
     * XOR of the individual parities because parity is linear over
     * disjoint bit fields.  The receiver checks:
     *   received_P XOR parity(received_data) XOR parity(received_red)
     * and combines the result with the PESEC syndrome to distinguish
     * single errors (correctable) from double errors (detectable). */
    pesec_overall_parity = parity16(current_bus_state & BUS_STATE_MASK)
                         ^ parity16(pesec_redundancy_reg & PESEC_RED_MASK);

    /* Step 7: Output 26 bits to daisy-chained shift registers */
    output_to_shift_registers();
}
