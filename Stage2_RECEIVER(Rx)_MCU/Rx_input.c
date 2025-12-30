/* File: rx_input.c
 * Reads 12-bit X from Port 2 (P2.0-P2.7) and Port 3 (P3.4-P3.7)
 */
#include <aduc841.h>
#include <string.h>
#include "header.h"

void read_X_from_bus(uint8_t *X)
{
    uint8_t p2_val, p3_val;
    uint8_t i;
    
    // Clear output
    memset(X, 0, HAMMING_N);
    
    // Read ports atomically
    EA = 0;
    p2_val = P2;
    p3_val = P3;
    EA = 1;
    
    // Unpack P2 into X[0-7]
    for (i = 0; i < 8; i++)
    {
        X[i] = (p2_val >> i) & 0x01;
    }
    
    // Unpack P3.4-P3.7 into X[8-11]
    for (i = 0; i < 4; i++)
    {
        X[8 + i] = (p3_val >> (4 + i)) & 0x01;
    }
}
