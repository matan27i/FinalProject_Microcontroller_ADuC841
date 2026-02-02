/* File: shift_output.c
 */

#include <aduc841.h>
#include <intrins.h>  /* For _nop_() */
#include "header.h"

 /*
  delay_half_period - Delay for one half-period of shift register clock
  Provides a 5 µs delay for 100 kHz shift register clock timing.
 
  Calculation:
    - Target: 5 µs = 5000 ns
    - CPU cycle time: 1 / 11.0592 MHz = 90.422 ns
    - Required cycles: 5000 / 90.422 ≈ 55.3 cycles
 
  Implementation:
    - uint8_t initialization: 1 cycle (MOV)
    - Loop body: DJNZ (2 cycles) + NOP (1 cycle) = 3 cycles per iteration
    - 18 iterations: 18 * 3 = 54 cycles
    - Total: 1 + 54 = 55 cycles ≈ 4.973 µs
 
  Actual timing: 55 * 90.422 ns = 4973 ns = 4.973 µs
  Clock frequency: 1 / (2 * 4.973 µs) = 100.5 kHz
 
  Note: This is a blocking delay. The function disables interrupts during
  the shift operation, so timing precision is maintained.
  ---------------------------------------------------------------------------
  */
static void delay_half_period(void)
{
    uint8_t i = 18;  /* Calibrated for 5 µs at 11.0592 MHz */

    /* Each iteration: 2 cycles (DJNZ) + 1 cycle (NOP) = 3 cycles */
    while (i--) {
        _nop_();
    }
}

void output_to_shift_registers(void)
{
    uint16_t state_copy;
    signed char bit_pos;  /* Signed to allow decrement below zero */
    uint8_t saved_ea;
    uint8_t bit_value;

    /* === Begin Critical Section === */
    /* Save and disable interrupts for atomic, timing-consistent output */
    saved_ea = EA;
    EA = 0;

    /* Make local copy of bus state (guarantees consistency during output) */
    state_copy = current_bus_state & BUS_STATE_MASK;

    /* Step 1: Ensure LATCH is low before starting shift sequence */
    LATCH_PIN = 0;
    delay_half_period();  /* Allow pin to settle and outputs to stabilize */

    /* Step 2: Shift out 15 bits, MSB-first (bit 14 down to bit 0) */
    for (bit_pos = (HAMMING_N - 1); bit_pos >= 0; bit_pos--)
    {
        /* Extract current bit value
           Note: Explicit extraction to uint8_t for clarity and to avoid
           any potential issues with implicit type conversions */
        bit_value = (uint8_t)((state_copy >> bit_pos) & 0x0001);

        /* Set DATA_PIN (SER input) to current bit value */
        DATA_PIN = bit_value;

        /* Data Setup Time: Ensure data is stable before clock rising edge
           74HC595 requires minimum 25 ns setup time @ 5V
           We provide 2 NOPs = 2 * 90.422 ns = 180.8 ns
           This is 7.2× the minimum requirement, ensuring robust operation */
        _nop_();
        _nop_();

        /* Clock Rising Edge: Shift data into register on SRCLK rising edge
           The 74HC595 samples DATA_PIN on the rising edge of CLK_PIN */
        CLK_PIN = 1;
        delay_half_period();  /* 5 µs high pulse */

        /* Clock Falling Edge: Complete the clock cycle
           This also provides data hold time for the current bit
           and preparation time for setting the next bit */
        CLK_PIN = 0;
        delay_half_period();  /* 5 µs low pulse */

        /* At this point:
           - Current bit has been shifted into the shift register
           - Clock is low, ready for next bit
           - We have 5 µs to set up the next data bit (far exceeds tsu) */
    }

    /* Step 3: Setup time before latching
       Ensure adequate settling after last clock falling edge
       The 74HC595 requires setup time between last SRCLK falling edge
       and RCLK rising edge. We provide an additional 5 µs for margin. */
    delay_half_period();

    /* Step 4: Pulse LATCH (RCLK) to transfer shift register → storage register
       This makes the shifted data appear on the output pins (Q0-Q7)
       Minimum latch pulse width: 25 ns @ 5V
       We provide: 5 µs (200× minimum) */
    LATCH_PIN = 1;
    delay_half_period();  /* 5 µs latch pulse */

    /* Step 5: Return LATCH to low for clean idle state
       This is the standard idle condition for 74HC595:
         - SRCLK (CLK_PIN) = LOW
         - RCLK (LATCH_PIN) = LOW
         - SER (DATA_PIN) = Previous last bit (don't care while idle)
       Having all control signals low reduces power and prevents
       spurious operation during subsequent power-up or noise events */
    LATCH_PIN = 0;

    /* === End Critical Section === */
    EA = saved_ea;  /* Restore interrupt enable state */

}


void Port_Init(void)
{
    /* Initialize all shift register control pins to LOW (idle state) */
    DATA_PIN = 0;  /* P2.0: Serial data input to shift registers */
    CLK_PIN = 0;  /* P2.1: Shift register clock (SRCLK) */
    LATCH_PIN = 0;  /* P2.2: Storage register clock (RCLK) */
      
}