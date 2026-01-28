/* File: shift_output.c
 * Shift Register Output Driver for H1-Type Bus Encoder
 * Shift Register: SN74HC595
 *
 * BIT MAPPING AND SHIFT ORDER:
 * H column index i (1..15) maps to bit position (i-1) in current_bus_state.
 * 
 *   Bit 0  <=> Column 1  <=> QA of first 74HC595
 *   Bit 1  <=> Column 2  <=> QB of first 74HC595
 *   ...
 *   Bit 7  <=> Column 8  <=> QH of first 74HC595
 *   Bit 8  <=> Column 9  <=> QA of second 74HC595
 *   ...
 *   Bit 14 <=> Column 15 <=> QG of second 74HC595
 *
 * Shift order: MSB-first (bit 14 down to bit 0).
 * First bit shifted ends up at QH of second chip, last bit at QA of first chip.
 *
 * 74HC595 TIMING (from datasheet @ VCC=4.5V):
 *   - fmax (SRCLK): 25 MHz
 *   - tsu (SER before SRCLK↑): 25 ns min
 *   - th (SER after SRCLK↑): 0 ns
 *   - tsu (SRCLK↑ before RCLK↑): 19 ns min
 *   - tw (SRCLK/RCLK high or low): 20 ns min
 *
 * CLK TIMING ANALYSIS:
 *
 * Target CLK frequency: ~100 kHz (conservative, well within 74HC595 specs)
 * 
 * NOP-based delay calibration:
 *   - Each _nop_() is 1 cycle ≈ 90 ns
 *   - 55 NOPs ≈ 4.95 µs ≈ 5 µs
 *   - Use 55 NOPs for each half-period to achieve ~100 kHz CLK
 *
 */

#include <aduc841.h>
#include <intrins.h>  /* For _nop_() */
#include "header.h"

/* NOP delay macros for CLK timing
 * Assuming 11.0592 MHz CPU, each NOP ≈ 90 ns
 * Target: 55 NOPs ≈ 5 µs delay for ~100 kHz CLK
 * (100 kHz = 10 µs period, 5 µs per half-period)
 */
#define CLK_DELAY_NOPS() do { \
    _nop_(); _nop_(); _nop_(); _nop_(); _nop_(); _nop_(); _nop_(); _nop_(); _nop_(); _nop_(); \
    _nop_(); _nop_(); _nop_(); _nop_(); _nop_(); _nop_(); _nop_(); _nop_(); _nop_(); _nop_(); \
    _nop_(); _nop_(); _nop_(); _nop_(); _nop_(); _nop_(); _nop_(); _nop_(); _nop_(); _nop_(); \
    _nop_(); _nop_(); _nop_(); _nop_(); _nop_(); _nop_(); _nop_(); _nop_(); _nop_(); _nop_(); \
    _nop_(); _nop_(); _nop_(); _nop_(); _nop_(); _nop_(); _nop_(); _nop_(); _nop_(); _nop_(); \
    _nop_(); _nop_(); _nop_(); _nop_(); _nop_(); \
} while(0)

/* output_to_shift_registers
 * Bit-bangs current_bus_state (15 bits) to chained 74HC595 shift registers.
 *
 * 74HC595 Pin Mapping:
 *   SER_PIN   -> SER (pin 14)   - Serial data input
 *   SRCLK_PIN -> SRCLK (pin 11) - Shift register clock (rising edge triggered)
 *   RCLK_PIN  -> RCLK (pin 12)  - Storage register clock (rising edge triggered)
 *
 * Protocol sequence (per 74HC595 datasheet):
 * 1. For each bit (MSB-first, bit 14 down to bit 0):
 *    a. Set SER_PIN to the bit value
 *    b. Pulse SRCLK_PIN high then low - data shifts on rising edge
 * 2. After all bits shifted, pulse RCLK_PIN high then low
 *    - This transfers shift register contents to output latches on rising edge
 *
 * Note: RCLK and SRCLK are independent. RCLK doesn't need to be held low
 * during shifting - only the rising edge matters for latching.
 *
 * Interrupt safety: Disables interrupts during bit-bang to ensure timing
 * consistency and atomic bus state output. Re-enables after completion.
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
    
    /* Step 1: Shift out 15 bits, MSB-first (bit 14 down to bit 0) */
    /* Data is clocked into 74HC595 shift register on SRCLK rising edge */
    for (bit_pos = (HAMMING_N - 1); bit_pos >= 0; bit_pos--)
    {
        /* 1a. Set SER_PIN to current bit value */
        SER_PIN = (uint8_t)((state_copy >> bit_pos) & 0x0001);
        
        /* Small setup time for data before clock edge (tsu = 25ns min @ 4.5V) */
        _nop_(); _nop_();
        
        /* 1b. SRCLK rising edge - data shifts in */
        SRCLK_PIN = 1;
        CLK_DELAY_NOPS();  /* ~5 µs high time for 100 kHz CLK */
        
        /* 1c. SRCLK falling edge */
        SRCLK_PIN = 0;
        CLK_DELAY_NOPS();  /* ~5 µs low time */
    }
    
    /* Step 2: Pulse RCLK to transfer shift register to output latches */
    /* 74HC595 latches data on RCLK rising edge */
    
    /* Small delay after last SRCLK before RCLK (tsu: SRCLK↑ before RCLK↑ = 19ns min @ 4.5V) */
    _nop_(); _nop_(); _nop_();
    
    /* RCLK rising edge - transfers shift register to storage register */
    RCLK_PIN = 1;
    CLK_DELAY_NOPS();  /* Hold high for setup time */
    
    /* RCLK falling edge - prepare for next transfer */
    RCLK_PIN = 0;
    
    /* === End Critical Section === */
    EA = saved_ea;  /* Restore interrupt state */
}

/* Port_Init
 * Initialize GPIO pins for 74HC595 shift register interface.
 * Pin initialization:
 *   SER_PIN (DATA)   - LOW (no data)
 *   SRCLK_PIN (CLK)  - LOW (ready for rising edge)
 *   RCLK_PIN (LATCH) - LOW (ready for rising edge)
 *
 */
void Port_Init(void)
{
    /* Initialize shift register control pins */
    SER_PIN   = 0;   /* Serial data input - idle low */
    SRCLK_PIN = 0;   /* Shift register clock - idle low */
    RCLK_PIN  = 0;   /* Storage register clock - idle low */
}