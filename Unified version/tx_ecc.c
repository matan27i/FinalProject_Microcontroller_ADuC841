/* =============================================================================
   File: tx_ecc.c
   Module: Group 3 -- TX-Specific PESEC Encoding + SEC+DED Parity
   =============================================================================
   Target: ADuC841 (8052 single-cycle core)

   Contains:
     - pesec_redundancy_reg: the 10-bit ECC redundancy vector
     - pesec_overall_parity: the 26th bit for SEC+DED (W4 upgrade)
     - Delta solver (TX-only: computes optimal redundancy-bit flip vector)
     - ECC(): main encode cycle -- redundancy update + parity + output

   DRY REFACTOR:
     Matrix initialisation, syndrome computation, and parity calculation
     have been extracted into the shared pesec_core module.  This file
     now calls pesec_init_matrices(), get_data_syndrome(),
     get_red_syndrome(), and calculate_overall_parity() from pesec_core.c.

   SEC+DED UPGRADE (W4):
     After computing the redundancy, a single overall even-parity bit P
     is computed over all 25 bits (15 data + 10 redundancy).  This
     expands the physical bus word to 26 bits and upgrades the error
     model from SEC to SEC+DED.

   Portability:
     Pure algorithmic logic.  The only external call is
     output_to_shift_registers() from Group 1.
   =============================================================================
*/

#include "tx_header.h"

/* =========================================================================
   TX-SPECIFIC STATE VARIABLE DEFINITIONS
   ========================================================================= */

/* 10-bit redundancy register, updated differentially.
 * [N2] Not volatile: only accessed from main-loop call chain. */
uint16_t pesec_redundancy_reg = 0;

/* [W4] Overall even-parity bit for SEC+DED.
 * Computed fresh from all 25 bits (15 data + 10 redundancy) after each
 * redundancy update.  Read by output_to_shift_registers() in tx_main.c
 * to shift it out as the 26th bit on the bus.
 * Value: 0 = even number of 1s in the 25-bit word, 1 = odd. */
uint8_t pesec_overall_parity = 0;

/* =========================================================================
   DELTA SOLVER  (TX-specific)
   =========================================================================
   Given the syndrome gap between the desired and current redundancy
   syndromes, produces the minimal-weight redundancy-bit flip vector.
   This is encoding-side logic with no counterpart on the RX.
   ========================================================================= */
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
   MAIN ECC FUNCTION
   =========================================================================
   Called after each nibble encoding cycle in process_nibble().

   Steps:
     1. Compute syndrome of data bits          (shared: get_data_syndrome)
     2. Compute syndrome of redundancy bits     (shared: get_red_syndrome)
     3. XOR to find the syndrome gap
     4. Solve for the optimal redundancy-bit flip vector  (TX-specific)
     5. Apply the differential update to pesec_redundancy_reg
     6. [W4] Compute overall even parity        (shared: calculate_overall_parity)
     7. Output all 26 bits to shift registers
   ========================================================================= */
void ECC(void)
{
    uint8_t  synd_data;
    uint8_t  synd_red;
    uint8_t  synd_diff;
    uint16_t delta_vector;

    /* Step 1: Syndrome of the new data */
    synd_data = get_data_syndrome(current_bus_state);

    /* Step 2: Syndrome of the current redundancy */
    synd_red = get_red_syndrome(pesec_redundancy_reg);

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
    pesec_overall_parity =
        calculate_overall_parity(current_bus_state & BUS_STATE_MASK)
      ^ calculate_overall_parity(pesec_redundancy_reg & PESEC_RED_MASK);

    /* Step 7: Output 26 bits to daisy-chained shift registers */
    output_to_shift_registers();
}
