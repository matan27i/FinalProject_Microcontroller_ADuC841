/* File: bus_encoder.c */
#include <aduc841.h>
#include <string.h> 
#include "header.h"

/* Single Packet Decoder */
uint8_t get_X_from_S(const uint8_t *S_vector, uint8_t hamming_R, uint8_t *X_output)
{
    // Calculate block size N based on R: N = 2^R - 1
    uint8_t HAMMING_N_CALC = (1 << hamming_R) - 1;
    uint8_t i;
    uint8_t col_index = 0; 

    // Reset output vector X to 0 (assume no errors initially)
    memset(X_output, 0, HAMMING_N_CALC);

    // Convert the Syndrome vector (S) from binary bits to an integer value.
    // This value represents the position column index .
    for (i = 0; i < hamming_R; i++)
    {
        col_index = (col_index << 1) | S_vector[i];
    }

    // Safety check: Ensure the calculated index is within the valid block range
    if (col_index > HAMMING_N_CALC) return 0;

    // If col_index is 0, the syndrome is zero -> No error occurred.
    // If col_index > 0, it points to the bit location of the error (1-based index).
    if (col_index > 0)
    {
        // Mark the error location in the X vector (convert 1-based to 0-based index)
        X_output[col_index - 1] = 1;
    }

    // Return the calculated error position
    return col_index;
}

/* Multi Packet Decoder */
uint8_t get_multi_X_from_S(const uint8_t *S_stream, const uint8_t *R_list, uint8_t packet_count, uint8_t *X_stream)
{
    uint8_t p;        
    uint8_t current_R; 
    uint8_t current_N; 
    
    // Initialize running pointers to traverse the input/output arrays
    const uint8_t *curr_S_ptr = S_stream;
    uint8_t *curr_X_ptr = X_stream;

    for (p = 0; p < packet_count; p++)
    {
        // 1. Get configuration for current packet
        current_R = R_list[p];            // Get R (number of parity bits/syndrome length)
        current_N = (1 << current_R) - 1; // Calculate block length N = (2^R - 1)

        // 2. Process single packet
        // Decodes the Syndrome (S) into Error Vector (X) for the current block
        get_X_from_S(curr_S_ptr, current_R, curr_X_ptr);
        
        // 3. Advance pointers for the next iteration
        curr_S_ptr += current_R; // Input pointer moves by R (Syndrome size)
        curr_X_ptr += current_N; // Output pointer moves by N (Block size)
    }
    return 0;
}