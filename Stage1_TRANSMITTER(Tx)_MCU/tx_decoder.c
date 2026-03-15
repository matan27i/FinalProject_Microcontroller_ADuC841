/* File: decoder.c
 * Group 2: H1-Type Bus Encoding & UART Character Handling
 * ========================================================
 * Target: ADuC841 (8052 single-cycle core)
 *
 * Contains:
 *   - current_bus_state: the 15-bit physical bus vector
 *   - Syndrome computation (on-the-fly, no lookup tables)
 *   - Minimal-weight codeword selection (O(1) for H1-type matrices)
 *   - process_nibble(): stateful differential encoder
 *   - tx_handler(): UART byte → nibble splitter
 *
 * H1-TYPE MATRIX STRUCTURE (4×15):
 *   Column i (1-based) = binary representation of i.
 *   Syndrome S = XOR of column indices for every set bit in x.
 *
 * MINIMAL-WEIGHT SELECTION:
 *   For s_target in {1..15}, w = 1 << (s_target - 1)  is weight-1
 *   and provably minimal (H1-type minimum distance = 3).
 *
 * Portability: No hardware dependencies.  Calls ECC() from Group 3
 *              (which in turn calls output_to_shift_registers() in
 *              Group 1) at the end of each nibble cycle.
 */

#include "tx_header.h"

/* =========================================================================
 * STATE VARIABLE DEFINITIONS  (owned by this module)
 * ========================================================================= */

/* The 15-bit physical bus state vector x.
 * Only bits 0..14 are meaningful (BUS_STATE_MASK = 0x7FFF).
 * H * current_bus_state^T  gives the current syndrome.
 * Initialised to 0 → syndrome 0, bus in zero state.              */
volatile uint16_t current_bus_state = 0;

/* =========================================================================
 * SYNDROME COMPUTATION
 * =========================================================================
 * Computes S = H * x^T using the H1-type identity: column i = i.
 * No stored matrix.  Loop through set bits and XOR their 1-based index.
 * ========================================================================= */
uint8_t compute_syndrome_from_bus(uint16_t bus_state)
{
    uint8_t  syndrome = 0;
    uint8_t  col_idx;
    uint16_t temp_state;

    temp_state = bus_state & BUS_STATE_MASK;

    for (col_idx = 1; col_idx <= HAMMING_N; col_idx++)
    {
        if (temp_state & 0x0001)
        {
            syndrome ^= col_idx;
        }
        temp_state >>= 1;
    }

    return syndrome;
}

/* =========================================================================
 * MINIMAL-WEIGHT CODEWORD SEARCH
 * =========================================================================
 * Exploits H1-type structure:
 *   s_target == 0  →  w = 0          (weight-0, no change)
 *   s_target != 0  →  w = 1 << (s-1) (weight-1, single bit flip)
 * Provably minimal and fully deterministic.
 * ========================================================================= */
uint16_t find_minimal_w(uint8_t s_target)
{
    s_target &= 0x0F;

    if (s_target == 0)
    {
        return 0;
    }

    return ((uint16_t)1 << (s_target - 1));
}

/* =========================================================================
 * PROCESS NIBBLE  (Core Stateful Encoder)
 * =========================================================================
 * Steps:
 *   1. S_old    = H * current_bus_state^T
 *   2. S_target = S_new XOR S_old
 *   3. w        = minimal-weight vector with H * w^T = S_target
 *   4. current_bus_state ^= w          (differential toggle)
 *   5. Call ECC() to compute redundancy and output all 25 bits
 * ========================================================================= */
void process_nibble(uint8_t s_new)
{
    uint8_t  s_old;
    uint8_t  s_target;
    uint16_t w;

    s_new &= 0x0F;

    /* Step 1: Current syndrome from bus state */
    s_old = compute_syndrome_from_bus(current_bus_state);

    /* Step 2: Target syndrome (mod-2 arithmetic) */
    s_target = s_new ^ s_old;

    /* Step 3: Find minimal-weight change vector */
    w = find_minimal_w(s_target);

    /* Step 4: Differential update (XOR, never overwrite) */
    current_bus_state ^= w;
    current_bus_state &= BUS_STATE_MASK;

    /* Step 5: Compute ECC and output to hardware */
    ECC();
}

/* =========================================================================
 * TX HANDLER  (UART Character → Nibble Splitter)
 * =========================================================================
 * INPUT:  8-bit character from UART.
 * OUTPUT: Two calls to process_nibble (high nibble first, then low).
 *
 * Special characters:
 *   '\r', '\n'  — Batch terminators.  Set buffer_flag, no nibble output.
 *   '='         — Full bus + ECC reset.  Clears shift registers via
 *                 hardware clear sequence, resets both state vectors.
 * ========================================================================= */
void tx_handler(uint8_t rx_char)
{
    uint8_t high_nibble;
    uint8_t low_nibble;

    /* --- Batch terminators --- */
    if (rx_char == '\r' || rx_char == '\n')
    {
        buffer_flag = 1;
        return;
    }

    /* --- Full reset command --- */
    if (rx_char == '=')
    {
        /* Reset decoder state */
        current_bus_state = 0;

        /* FIX: Reset ECC redundancy register to stay in sync */
        pesec_redundancy_reg = 0;

        /* Hardware clear of shift registers (Mode 1) */
        flag_t2_mod = 1;
        counter_t2  = 0;
        TH2 = 0xD4;
        TL2 = 0xCD;
        TR2 = 1;

        /* FIX: Block until clear sequence completes to prevent
         * race condition with subsequent UART characters */
        while (TR2 == 1);

        buffer_flag = 1;
        return;
    }

    /* --- Normal data character: split into two nibbles --- */
    high_nibble = (rx_char >> 4) & 0x0F;
    low_nibble  = rx_char & 0x0F;

    /* Process HIGH nibble first (per protocol spec) */
    process_nibble(high_nibble);

    /* Process LOW nibble second */
    process_nibble(low_nibble);

    /* Track nibble count */
    if (buffer_count < 255)
    {
        buffer_count += 2;
    }
}
