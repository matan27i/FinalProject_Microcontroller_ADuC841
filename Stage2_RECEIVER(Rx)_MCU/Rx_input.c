/* File: rx_input.c
 * Reads ONLY the defined pins based on transmitter_out.c mapping.
 * * Hardware Mapping:
 * X[0-7]  <-- P2.0 - P2.7
 * X[8]    <-- P3.5
 * X[9]    <-- P3.6
 * X[10]   <-- P3.7
 * NOTE: P3.4 is NOT used and is ignored to prevent floating input errors.
 *

#include <aduc841.h>
#include <string.h>
#include "header.h"

void read_X_from_bus(uint8_t *X)
{
    uint8_t p2_val, p3_val;
    uint8_t i;
    
    // 1. Clear output array to ensure no garbage data remains in unused bits
    memset(X, 0, HAMMING_N);
    
    // 2. Atomic read from ports (Critical Section)
    EA = 0;
    p2_val = P2;
    p3_val = P3;
    EA = 1;
    
    // 3. Unpack P2 into X[0-7]
    for (i = 0; i < 8; i++)
    {
        X[i] = (p2_val >> i) & 0x01;
    }
    
    // 4. Unpack P3.5-P3.7 into X[8-10]
    // Explicitly shifting specific bits to avoid reading P3.4
    
    X[8]  = (p3_val >> 5) & 0x01; // Read P3.5
    X[9]  = (p3_val >> 6) & 0x01; // Read P3.6
    X[10] = (p3_val >> 7) & 0x01; // Read P3.7
    
    // Bits X[11] through X[14] remain 0 (from memset)
}/
/* File: rx_input.c
 * Active Low Logic:
 * Voltage High (1) -> Interpreted as Logic 0 (Idle)
 * Voltage Low (0/GND) -> Interpreted as Logic 1 (Active)
 */

#include <aduc841.h>
#include <string.h>
#include "header.h"

void read_X_from_bus(uint8_t *X)
{
    uint8_t p2_val, p3_val;
    uint8_t i;
    
    // 1. Clear output array
    memset(X, 0, HAMMING_N);
    
    // 2. Atomic read from ports
    EA = 0;
    p2_val = P2;
    p3_val = P3;
    EA = 1;
    
    // 3. Unpack P2 into X[0-7] with INVERSION
    // We use the '!' operator to flip the bit value.
    // If P2.i is 1 (Floating), X[i] becomes 0.
    // If P2.i is 0 (Grounded), X[i] becomes 1.
    for (i = 0; i < 8; i++)
    {
        // Check if the specific bit is LOW (0)
        if ( (p2_val & (1 << i)) == 0 ) 
        {
            X[i] = 1; // Active!
        }
        else
        {
            X[i] = 0; // Idle
        }
    }
    
    // 4. Unpack P3.5-P3.7 with INVERSION
    
    // Check P3.5 -> X[8]
    if ((p3_val & (1 << 5)) == 0) X[8] = 1; else X[8] = 0;

    // Check P3.6 -> X[9]
    if ((p3_val & (1 << 6)) == 0) X[9] = 1; else X[9] = 0;

    // Check P3.7 -> X[10]
    if ((p3_val & (1 << 7)) == 0) X[10] = 1; else X[10] = 0;
}