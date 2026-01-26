/* File: transmitter_out.c */
#include <aduc841.h>
#include <intrins.h> // Required for _nop_() function
#include "header.h"

// Note: 'i' is now defined locally inside the function to avoid linker errors.

void transmit_X_to_shift_reg(const uint8_t *X_stream_output, uint8_t total_bits)
{
    int i; 
    int d; // Variable for the delay loop

    SR_LATCH = 0; 
    SR_CLOCK = 0;

    // Iterate through all bits (MSB first logic)
    for (i = total_bits - 1; i >= 0; i--)
    {
        SR_CLOCK = 0;

        // 1. Set Data
        if (X_stream_output[i] != 0)
        {
            SR_DATA = 1;
        }
        else
        {
            SR_DATA = 0;
        }

        // --- DELAY 1: Setup Time ---
        // Waits for Data to stabilize before raising Clock.
        // Increase '50' to make it slower.
        for(d=0; d<2000; d++) { _nop_(); } 

        // 2. Clock High (Rising Edge)
        SR_CLOCK = 1; 

        // --- DELAY 2: Hold Time / Clock Width ---
        // Keeps the Clock HIGH for a while so you can see the pulse.
        for(d=0; d<200; d++) { _nop_(); } 

        // 3. Clock Low
        // (Loop repeats)
    }

    SR_CLOCK = 0;
    
    // Pulse the Latch to update output
    SR_LATCH = 1; 
    for(d=0; d<200; d++) { _nop_(); } // Small delay for Latch
    SR_LATCH = 0; 
}