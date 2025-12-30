/* File: main.c */
#include <aduc841.h>
#include <string.h>
#include "header.h"

volatile uint8_t Snew = 0;
volatile uint8_t buffer_count = 0;
volatile bit buffer_flag = 0;
volatile bit rx_flag = 0;
volatile uint8_t rx_temp_byte = 0;

// Must be xdata to fit in memory
volatile uint8_t xdata S_stream_expanded[MAX_PACKETS * 4]; 

// Output Buffer xdata
uint8_t xdata X_stream_output[MAX_PACKETS * HAMMING_N];

//uint8_t R_config_list[MAX_PACKETS];
uint8_t R_config_list[] = {4,3};

void main(void)
{
    //uint8_t i;

    // --- Hardware Init ---
    GlobalINT();
    Timer3_Init();
    UART_Init();
    
    // --- Config Init ---
    /*for(i = 0; i < MAX_PACKETS; i++) 
    {
        R_config_list[i] = HAMMING_R;
    }*/
    while(1)
    {
        // --- Handle RX ---
        if (rx_flag == 1)
        {
            rx_flag = 0; 
            rx_handler(rx_temp_byte);
        }

        // --- Process Buffer ---
        if (buffer_flag)
        {
            ES = 0; // Disable serial interrupt (Critical Section)
            buffer_flag = 0; 
            
            // Process/Decode data stream
            get_multi_X_from_S((uint8_t*)S_stream_expanded, R_config_list, buffer_count, X_stream_output);
            
            buffer_count = 0; // Reset counter
            ES = 1; // Re-enable serial interrupt
        }
    }
}