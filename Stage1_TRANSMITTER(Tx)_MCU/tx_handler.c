/* File: tx_handler.c */
#include <aduc841.h>
#include "header.h"


uint8_t xdata S_input_buffer[MAX_BLOCKS];


void tx_handler(uint8_t tx_char)
{
    uint8_t parsed_val = 0xFF; 

    if (tx_char == '\r' || tx_char == '\n')
    {
        buffer_flag = 1; 
        return; 
    }

    // --- 2. ASCII to Hex Conversion ---
    if (tx_char >= '0' && tx_char <= '9')       
        parsed_val = tx_char - '0';
    else if (tx_char >= 'A' && tx_char <= 'F') 
        parsed_val = tx_char - 'A' + 10;
    else if (tx_char >= 'a' && tx_char <= 'f') // Support lowercase hex
        parsed_val = tx_char - 'a' + 10;

    // --- 3. Buffer Logic ---
    if (parsed_val != 0xFF)
    {
        // Store value in buffer if there is space
        if (buffer_count < num_active_blocks)
        {
            S_input_buffer[buffer_count] = parsed_val;
            buffer_count++;
        }
        
        // Auto-trigger if buffer is full (Standard behavior)
        if (buffer_count >= num_active_blocks)
        {
            buffer_flag = 1; 
        }
    }
}