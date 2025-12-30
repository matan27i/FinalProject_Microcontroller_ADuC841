/* File: rx_handler.c */
/*
 * Function: rx_handler
 * --------------------
 * Processes a single character received from UART.
 * 1. Converts ASCII Hex character ('0'-'9', 'A'-'F') to integer value.
 * 2. Expands the 4-bit nibble into individual bits in the S_stream_expanded array.
 * 3. Updates the buffer count and sets a flag for the main loop.
 */
#include <aduc841.h>
#include "header.h"

void tx_handler(uint8_t tx_char)
{
    uint8_t parsed_val = 0xFF; // Initialize as invalid (0xFF)
    uint8_t i;
    uint8_t index;
		// --- 0. Check If We Done To Send Data ---
		if (tx_char == '\r' || tx_char == '\n')
    {
			buffer_flag = 1;       
      return; 
    }

    // --- 1. ASCII to Hex Conversion ---
    
    // Check if character is a digit '0' through '9'
    if (tx_char >= '0' && tx_char <= '9')       
    {
        parsed_val = tx_char - '0';
    }
    // Check if character is a letter 'A' through 'F'
    else if (tx_char >= 'A' && tx_char <= 'F') 
    {
        parsed_val = tx_char - 'A' + 10;
    }

    // --- 2. Process Valid Data ---
    
    // Proceed only if the character was a valid Hex digit
    if (parsed_val != 0xFF)
    {
        Snew = parsed_val; // Store the raw syndrome value

        // Check if there is space in the buffer
        if (buffer_count < MAX_PACKETS)
        {
            // --- 3. Bitwise Expansion ---
            // Loop to extract each bit from the nibble (MSB to LSB)
            // and store it as a separate byte in the expanded array.
            for (i = 0; i < HAMMING_R; i++)
            {
                // Calculate the linear index in the global xdata array
                index = (buffer_count * HAMMING_R) + i;
                
                // Logic:
                // 1. Shift 'parsed_val' right to move the target bit to position 0.
                // 2. Mask with 0x01 to isolate that single bit.
                S_stream_expanded[index] = 
                    (parsed_val >> (HAMMING_R - 1 - i)) & 0x01;
            }
            
            buffer_count++; // Increment the number of received packets
        }
    }
}


//Test For GitHub