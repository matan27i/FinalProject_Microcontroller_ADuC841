/* File: bus_encoder.c */
#include <aduc841.h>
#include <string.h> 
#include "header.h"

/* Single Packet Decoder */
uint8_t get_X_from_S(const uint8_t *S_vector, uint8_t hamming_R, uint8_t *X_output)
{
    uint8_t HAMMING_N_CALC = (1 << hamming_R) - 1;
    uint8_t i;
    uint8_t col_index = 0; 

    memset(X_output, 0, HAMMING_N_CALC);

    for (i = 0; i < hamming_R; i++)
    {
        col_index = (col_index << 1) | S_vector[i];
    }

    if (col_index > HAMMING_N_CALC) return 0;

    if (col_index > 0)
    {
        X_output[col_index - 1] = 1;
    }
    return col_index;
}

/* Multi Packet Decoder */
int get_multi_X_from_S(const uint8_t *S_stream, const uint8_t *R_list, uint8_t packet_count, uint8_t *X_stream)
{
    uint8_t p;        
    uint8_t current_R; 
    uint8_t current_N; 
    
    const uint8_t *curr_S_ptr = S_stream;
    uint8_t *curr_X_ptr = X_stream;

    for (p = 0; p < packet_count; p++)
    {
        current_R = R_list[p];
        current_N = (1 << current_R) - 1;

        get_X_from_S(curr_S_ptr, current_R, curr_X_ptr);
        
        curr_S_ptr += current_R;
        curr_X_ptr += current_N;
    }
    return 0;
}
