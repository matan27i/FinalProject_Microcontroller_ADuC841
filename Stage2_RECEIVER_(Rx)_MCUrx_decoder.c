/* File: rx_decoder.c
 * Uses PROVIDED decoding logic
 */
#include <aduc841.h>
#include <string.h>
#include "header.h"

void get_S_from_X(const uint8_t *X_vector, uint8_t hamming_R, uint8_t *S_output)
{
    /* Renamed variable to avoid conflict with #define HAMMING_N in header.h */
    uint8_t hamming_n_limit; 
    uint8_t i;
    uint8_t position_val = 0;
    
    /* Calculate N based on R input (2^R - 1) */
    hamming_n_limit = (1 << hamming_R) - 1;
    
    /* Find position of '1' bit */
    for (i = 0; i < hamming_n_limit; i++)
    {
        if (X_vector[i] != 0)
        {
            position_val = i + 1;
            break;
        }
    }
    
    /* Reset output */
    memset(S_output, 0, hamming_R);
    
    /* Convert position to binary array (MSB first) */
    if (position_val > 0)
    {
        for (i = 0; i < hamming_R; i++)
        {
            S_output[i] = (uint8_t)((position_val >> (hamming_R - 1 - i)) & 0x01);
        }
    }
}

uint8_t bits_to_decimal(const uint8_t *bits, uint8_t length)
{
    uint8_t result = 0;
    uint8_t i;
    
    for (i = 0; i < length; i++)
    {
        result = (result << 1) | (bits[i] & 0x01);
    }
    
    return result;
}
