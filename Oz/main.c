/* File: main.c */
#include <aduc841.h>
#include <string.h>
#include "header.h"

volatile uint8_t Snew = 0;
volatile uint8_t buffer_count = 0;
volatile bit buffer_flag = 0;
volatile bit tx_flag = 0;
volatile uint8_t tx_temp_byte = 0;

// Must be xdata to fit in memory
volatile uint8_t xdata S_stream_expanded[MAX_PACKETS * 4]; 

// Output Buffer xdata
uint8_t xdata X_stream_output[MAX_PACKETS * HAMMING_N];

//uint8_t R_config_list[MAX_PACKETS];
uint8_t R_config_list[] = {4,3};

void main(void)
{

    // --- Hardware Init ---
    GlobalINT();
    Timer3_Init();
    UART_Init();
    
 
    while(1)
    {
        // --- Handle TX ---
        if (tx_flag == 1)
        {
            tx_flag = 0; 
            tx_handler(tx_temp_byte);
        }

        // --- Process Buffer ---
        if (buffer_flag)
        {
            ES = 0; // Disable serial interrupt (Critical Section)
            buffer_flag = 0; 
            
            // Process/Decode data stream
            get_multi_X_from_S((uint8_t*)S_stream_expanded, R_config_list, buffer_count, X_stream_output);

            transmit_X_to_bus(X_stream_output); // Send output to bus
            buffer_count = 0;// Reset buffer
            
            buffer_count = 0; // Reset counter
            ES = 1; // Re-enable serial interrupt
        }
    }
}
