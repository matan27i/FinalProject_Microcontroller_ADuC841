/* File: encoder.c
 * Group 2: H1-Type Bus Encoding & UART Character Handling
 * ========================================================
 * Target: ADuC841 (8052 single-cycle core)
 *
 * Contains:
 *   - current_bus_state: the 15-bit physical bus vector
 *   - Syndrome computation (on-the-fly, no lookup tables)
 *   - Minimal-weight codeword selection (O(1) for H1-type matrices)
 *   - process_nibble(): stateful differential encoder
 *   - tx_handler(): UART byte -> nibble splitter
 *
 * FIX LOG:
 *   [W4] tx_handler '=' reset now also clears pesec_overall_parity
 *        to ensure the 26th bit is zero after a full bus reset.
 *
 * Portability: No hardware dependencies.  Calls ECC() from Group 3.
 */

#include "tx_header.h"

/* =========================================================================
 * STATE VARIABLE DEFINITIONS  (owned by this module)
 * ========================================================================= */

uint16_t current_bus_state = 0;

/* =========================================================================
 * SYNDROME COMPUTATION
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
 * ========================================================================= */
void process_nibble(uint8_t s_new)
{
    uint8_t  s_old;
    uint8_t  s_target;
    uint16_t w;

    s_new &= 0x0F;

    s_old = compute_syndrome_from_bus(current_bus_state);

    s_target = s_new ^ s_old;

    w = find_minimal_w(s_target);

    current_bus_state ^= w;
    current_bus_state &= BUS_STATE_MASK;

    ECC();
}

/* =========================================================================
 * TX HANDLER  (UART Character -> Nibble Splitter)
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
        /* Reset encoder state */
        current_bus_state = 0;

        /* Reset ECC redundancy register to stay in sync */
        pesec_redundancy_reg = 0;

        /* [W4] Reset overall parity bit so the 26th output is zero.
         * After reset, all 26 bits (data + red + parity) are zero,
         * which the RX detects as a bus-reset condition. */
        pesec_overall_parity = 0;

        /* Hardware clear of shift registers (Mode 1) */
        flag_t2_mod = 1;
        counter_t2  = 0;
        TH2 = T2_RELOAD_H;
        TL2 = T2_RELOAD_L;
        TR2 = 1;

        while (TR2 == 1);

        buffer_flag = 1;
        return;
    }

    /* --- Normal data character: split into two nibbles --- */
    high_nibble = (rx_char >> 4) & 0x0F;
    low_nibble  = rx_char & 0x0F;

    process_nibble(high_nibble);

    process_nibble(low_nibble);

    if (buffer_count <= 253)
    {
        buffer_count += 2;
    }
}
