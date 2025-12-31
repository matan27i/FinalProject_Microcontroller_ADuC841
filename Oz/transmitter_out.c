/* File: transmitter_out.c
 * Outputs 12 bits of X to Port 2 (P2.0-P2.7) and Port 3 (P3.4-P3.7)
 * Pin mapping based on PCB schematic:
 * X[0-7]  -> P2.0-P2.7
 * X[8-11] -> P3.4-P3.7
 */
#include <aduc841.h>
#include "header.h"

void transmit_X_to_bus(const uint8_t *X_stream_output)
{
    uint8_t p2_value = 0;
    uint8_t p3_upper = 0;
    uint8_t i;
    
    // Pack X[0-7] into P2
    for (i = 0; i < 8; i++)
    {
        if (X_stream_output[i] != 0)
        {
            p2_value |= (1 << i);
        }
    }
    
    // Pack X[8-10] into P3.5-P3.7
    // Reset p3_upper before the loop (assuming it is defined locally)

    for (i = 8; i < 11; i++)
    {
        if (X_stream_output[i] != 0)
        {
            p3_upper |= (1 << (i - 8 + 5));  
        }
    }
    
    EA = 0; //  Atomic write to ports = Uninterruptible code clock
    P2 = p2_value;
    P3 = (P3 & 0x1F) | p3_upper;     // Change mask to 0x1F (0001 1111) to preserve P3.0 through P3.4 We only want to overwrite P3.5, P3.6, P3.7
    EA = 1;
}
