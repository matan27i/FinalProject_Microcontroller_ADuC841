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
//volatile uint8_t xdata S_stream_expanded[MAX_PACKETS * 4]; 

// Output Buffer xdata
//uint8_t xdata X_stream_output[MAX_PACKETS * HAMMING_N];

//uint8_t R_config_list[MAX_PACKETS];
uint8_t code_config_R[] = {4, 3};
uint8_t num_active_blocks = 6;

void main(void)
{
    // --- Hardware Init ---
    GlobalINT();
    Timer3_Init();
    UART_Init();
	  Port_Init();
    Init_Bus_State();
while(1)
    {
				
				   
				if (tx_flag == 1)
        {
            tx_flag = 0; 
            tx_handler(tx_temp_byte);
        }
				
        // --- Process Buffer ---
        if (buffer_flag)
        {
            ES = 0; 
            process_diagonal_system(S_input_buffer);      
            // Reset buffer logic for the next packet
            buffer_count = 0; 
            buffer_flag = 0;
            ES = 1;
        }
    }
}