/* File: bus_encoder.c
 * =============================================================================
 * H1-Type Stateful Bus Encoder - Core Algorithm Implementation
 * Target: ADuC841 (8052 single-cycle core)
 * =============================================================================
 *
 * H1-TYPE MATRIX STRUCTURE (4×15):
 * --------------------------------
 * The H1-type check matrix H has dimensions 4×15.
 * Column index i (1-based, i = 1..15) contains the 4-bit binary representation of i.
 * 
 * Column-to-bit mapping (DO NOT CHANGE - this defines the H1 structure):
 *   H column index i (1..15) => bit position (i-1) in current_bus_state (0..14)
 *   
 *   Column 1  = 0001 binary => bit 0
 *   Column 2  = 0010 binary => bit 1
 *   Column 3  = 0011 binary => bit 2
 *   ...
 *   Column 15 = 1111 binary => bit 14
 *
 * The matrix H is NOT stored. Instead, column i is computed as simply i.
 *
 * SYNDROME COMPUTATION:
 * ---------------------
 * S = H * x^T is computed by XOR-ing together the column indices (1..15)
 * for every bit position that is set in x.
 *
 * If bit j (0-indexed) is set in x, it contributes column (j+1) to the syndrome.
 * syndrome = XOR of all (j+1) where x[j] = 1
 *
 * MINIMAL-WEIGHT w SELECTION:
 * ---------------------------
 * Algorithm: Exploit H1-type structure for deterministic minimal-weight solution.
 * 
 * For any s_target in {1..15}, there exists a weight-1 solution:
 *   w = single bit at position (s_target - 1)
 * because H * e_{s_target}^T = column s_target = s_target (the syndrome).
 *
 * For s_target = 0, no change needed: w = 0 (weight-0 solution).
 *
 * This is provably minimal: the H1-type matrix has all distinct columns,
 * so the minimum distance is 3. Any syndrome except 0 requires at least
 * one bit flip to produce, and exactly one bit flip achieves any syndrome 1..15.
 *
 * Tie-breaker rule: Since there's only one weight-1 solution for each nonzero
 * syndrome, no tie-breaking is needed. The algorithm is fully deterministic.
 *
 * =============================================================================
 */

#include <aduc841.h>
#include "header.h"

/* ---------------------------------------------------------------------------
 * compute_syndrome_from_bus
 * ---------------------------------------------------------------------------
 * Computes S = H * x^T on-the-fly using bitwise XOR.
 *
 * The H1-type matrix column i (1-indexed) equals i in binary.
 * So: syndrome = XOR of all column indices (j+1) for bits j that are set in x.
 *
 * Implementation: Loop through bits 0..14 of bus_state. For each set bit at
 * position j, XOR the syndrome with (j+1).
 *
 * No lookup tables. No stored matrix.
 * ---------------------------------------------------------------------------
 */
uint8_t compute_syndrome_from_bus(uint16_t bus_state)
{
    uint8_t syndrome = 0;
    uint8_t col_idx;
    uint16_t temp_state;
    
    /* Mask to only use bits 0..14 */
    temp_state = bus_state & BUS_STATE_MASK;
    
    /* 
     * Iterate through bit positions 0..14.
     * If bit j is set, XOR in column index (j+1).
     * Column index (j+1) is just the 4-bit binary representation of (j+1).
     */
    for (col_idx = 1; col_idx <= HAMMING_N; col_idx++)
    {
        /* Check if bit (col_idx - 1) is set */
        if (temp_state & 0x0001)
        {
            /* XOR the column index (which IS the column value in H1-type matrix) */
            syndrome ^= col_idx;
        }
        /* Shift to next bit */
        temp_state >>= 1;
    }
    
    return syndrome;
}

/* ---------------------------------------------------------------------------
 * find_minimal_w
 * ---------------------------------------------------------------------------
 * Finds the minimal Hamming-weight 15-bit vector w such that H * w^T == s_target.
 *
 * Algorithm: Exploit H1-type matrix structure (bounded search, actually O(1)).
 *
 * Key insight: In an H1-type matrix, column i equals the binary value i.
 * Therefore, to produce syndrome s_target (where 1 <= s_target <= 15),
 * we simply need to set bit (s_target - 1), which contributes column s_target.
 *
 * Weight-0: s_target = 0 => w = 0
 * Weight-1: s_target in {1..15} => w = (1 << (s_target - 1))
 *
 * This is provably minimal because:
 * - Zero syndrome requires zero changes (trivially minimal).
 * - Nonzero syndrome requires at least one bit (minimum distance ≥ 3).
 * - Exactly one bit at position (s-1) produces syndrome s.
 *
 * Determinism: Unique solution for each syndrome. No tie-breaking needed.
 * Smallest numeric w is achieved naturally since there's only one weight-1 solution.
 * ---------------------------------------------------------------------------
 */
uint16_t find_minimal_w(uint8_t s_target)
{
    /* Mask to 4 bits (just in case) */
    s_target &= 0x0F;
    
    /* Zero syndrome => no change needed */
    if (s_target == 0)
    {
        return 0;
    }
    
    /*
     * Nonzero syndrome s_target in {1..15}:
     * The minimal-weight solution is to flip exactly the bit at position (s_target - 1).
     * This sets w to have a single '1' at bit position (s_target - 1).
     *
     * H * w^T = column s_target = s_target (by H1-type definition).
     */
    return ((uint16_t)1 << (s_target - 1));
}

/* ---------------------------------------------------------------------------
 * process_nibble
 * ---------------------------------------------------------------------------
 * Core stateful encoder function. Processes one 4-bit syndrome S_new.
 *
 * Steps:
 * 1. Compute S_old = H * current_bus_state^T (on-the-fly)
 * 2. Compute S_target = S_new XOR S_old (modulo-2 arithmetic)
 * 3. Find minimal-weight w such that H * w^T = S_target
 * 4. Update: current_bus_state ^= w (differential toggle)
 * 5. Output new bus state to shift registers
 *
 * CRITICAL: Uses XOR for syndrome arithmetic. Never OR or other operations.
 * CRITICAL: Does NOT overwrite current_bus_state directly from S_new.
 *           All updates go through differential toggling via w.
 * ---------------------------------------------------------------------------
 */
void process_nibble(uint8_t s_new)
{
    uint8_t s_old;
    uint8_t s_target;
    uint16_t w;
    
    /* Mask s_new to 4 bits */
    s_new &= 0x0F;
    
    /* Step 1: Compute S_old from current bus state on-the-fly */
    s_old = compute_syndrome_from_bus(current_bus_state);
    
    /* Step 2: Compute target syndrome using XOR (mod-2) */
    s_target = s_new ^ s_old;
    
    /* Step 3: Find minimal-weight w */
    w = find_minimal_w(s_target);
    
    /* Step 4: Differential update (XOR, not overwrite!) */
    current_bus_state ^= w;
    
    /* Ensure we stay in valid range */
    current_bus_state &= BUS_STATE_MASK;
    
    /* Step 5: Output to shift registers */
    output_to_shift_registers();
}
