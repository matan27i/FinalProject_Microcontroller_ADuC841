/* File: shift_output.c
 * =============================================================================
 * Shift Register Output Driver for H1-Type Bus Encoder
 * Target: ADuC841 (8052 single-cycle core, 11.0592 MHz typical)
 * =============================================================================
 *
 * BIT MAPPING AND SHIFT ORDER:
 * ----------------------------
 * H column index i (1..15) maps to bit position (i-1) in current_bus_state.
 * 
 *   Bit 0  <=> Column 1  <=> first physical shift register output
 *   Bit 1  <=> Column 2
 *   ...
 *   Bit 14 <=> Column 15 <=> last physical shift register output
 *
 * Shift order: MSB-first (bit 14 down to bit 0).
 * This means column 15 is shifted out first, and column 1 is shifted out last.
 * After latching, bit 0 (column 1) appears at Q0 of the first shift register
 * in the chain, and bit 14 (column 15) appears at Q6 of the second shift register
 * (assuming 8-bit shift registers, 15 bits = 2 registers).
 *
 * CLK TIMING ANALYSIS:
 * --------------------
 * Target CLK frequency: ~1 MHz (1 µs period, 0.5 µs high, 0.5 µs low)
 * 
 * ADuC841 with 11.0592 MHz crystal:
 *   - Single-cycle core: 1 instruction cycle = 1 / 11.0592 MHz ≈ 90.4 ns
 *   - For 0.5 µs delay: need ~5-6 instruction cycles
 *
 * NOP-based delay calibration:
 *   - Each _nop_() is 1 cycle ≈ 90 ns
 *   - 5 NOPs ≈ 450 ns, 6 NOPs ≈ 540 ns
 *   - Use 5 NOPs for each half-period to achieve ~1 MHz CLK
 *
 * TO ADAPT TIMING FOR DIFFERENT CPU CLOCKS:
 *   - For 22.1184 MHz: double the NOPs (10 per half-period)
 *   - For 5.5296 MHz: halve the NOPs (2-3 per half-period)
 *   - Formula: NOPs ≈ (desired_delay_ns / 90) - instruction_overhead
 *
 * =============================================================================
 */

#include <aduc841.h>
#include <intrins.h>  /* For _nop_() */
#include "header.h"

/* ---------------------------------------------------------------------------
 * NOP delay macros for CLK timing
 * Assuming 11.0592 MHz CPU, each NOP ≈ 90 ns
 * Target: 5 NOPs ≈ 450 ns delay for ~1 MHz CLK
 * ---------------------------------------------------------------------------
 */
#define CLK_DELAY_NOPS() do { \
    _nop_(); _nop_(); _nop_(); _nop_(); _nop_(); \
} while(0)

/* ---------------------------------------------------------------------------
 * output_to_shift_registers
 * ---------------------------------------------------------------------------
 * Bit-bangs current_bus_state (15 bits) to chained shift registers.
 *
 * Protocol sequence:
 * 1. Pull LATCH_PIN low (prepare for shifting)
 * 2. For bit = 14 down to 0 (MSB-first):
 *    a. Set DATA_PIN to ((current_bus_state >> bit) & 1)
 *    b. Pulse CLK_PIN high for ~0.5 µs
 *    c. Pulse CLK_PIN low for ~0.5 µs
 * 3. Pulse LATCH_PIN high to transfer shift register contents to outputs
 *
 * Interrupt safety: Disables interrupts during bit-bang to ensure timing
 * consistency and atomic bus state output. Re-enables after completion.
 *
 * Reentrant safety: Protected by interrupt disable. Not truly reentrant,
 * but safe in single-threaded context with ISRs disabled.
 * ---------------------------------------------------------------------------
 */
void output_to_shift_registers(void)
{
    uint16_t state_copy;
    signed char bit_pos;  /* Signed for loop decrement below zero */
    uint8_t saved_ea;
    
    /* === Begin Critical Section === */
    /* Save and disable interrupts for atomic, timing-consistent output */
    saved_ea = EA;
    EA = 0;
    
    /* Make local copy of bus state (won't change during output) */
    state_copy = current_bus_state & BUS_STATE_MASK;
    
    /* Step 1: Pull LATCH low to enable shifting */
    LATCH_PIN = 0;
    
    /* Step 2: Shift out 15 bits, MSB-first (bit 14 down to bit 0) */
    for (bit_pos = (HAMMING_N - 1); bit_pos >= 0; bit_pos--)
    {
        /* 2a. Set DATA_PIN to current bit value */
        DATA_PIN = (uint8_t)((state_copy >> bit_pos) & 0x0001);
        
        /* 2b. CLK high pulse */
        CLK_PIN = 1;
        CLK_DELAY_NOPS();  /* ~450 ns delay for ~1 MHz CLK */
        
        /* 2c. CLK low */
        CLK_PIN = 0;
        CLK_DELAY_NOPS();  /* ~450 ns delay */
    }
    
    /* Step 3: Pulse LATCH high to transfer to outputs */
    LATCH_PIN = 1;
    CLK_DELAY_NOPS();  /* Short delay for latch setup */
    
    /* Optional: Return LATCH to low (depends on shift register type)
     * Most 74HC595-type registers latch on rising edge, so we can leave high.
     * Uncomment the following line if your hardware needs it low after:
     */
    /* LATCH_PIN = 0; */
    
    /* === End Critical Section === */
    EA = saved_ea;  /* Restore interrupt state */
}

/* ---------------------------------------------------------------------------
 * Port_Init
 * ---------------------------------------------------------------------------
 * Initialize GPIO pins for shift register interface.
 * Sets DATA_PIN, CLK_PIN, LATCH_PIN as outputs with known initial states.
 * ---------------------------------------------------------------------------
 */
void Port_Init(void)
{
    /* Initialize shift register control pins to known low state */
    DATA_PIN  = 0;
    CLK_PIN   = 0;
    LATCH_PIN = 0;
    
    /* 
     * Note: On ADuC841, port pins are configured via CFG841/CFG842 registers
     * for special functions. For standard GPIO, no special configuration needed.
     * The push-pull output mode is default for most pins.
     * 
     * If your pins are on Port 2 (as defined in header.h), ensure:
     * - P2.0, P2.1, P2.2 are configured as standard GPIO outputs
     */
}