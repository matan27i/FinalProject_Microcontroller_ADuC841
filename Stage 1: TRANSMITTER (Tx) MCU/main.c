/* File: main.c 

Final project Stage 1 Input + Transmitter - "Decoder" implemition

DESCRIPTION: ASCII input via UART then converting characters into bit streams
send to x from s (multi) functiom stored in extended memory waiting to send to bus (pin 1?)
*/

#include <aduc841.h>
#include <string.h>
#include "header.h"

volatile uint8_t Snew = 0;
volatile uint8_t buffer_count = 0;
volatile bit buffer_flag = 0;
int i;

volatile uint8_t xdata S_stream_expanded[Max_in_for_outpins * HAMMING_R]; // Must be xdata to fit in memory , volatile make the compiler cheack if there changes
                                                 
uint8_t xdata X_stream_output[Max_in_for_outpins * HAMMING_N]; // Output Buffer xdata - 15 * buffer size byte

uint8_t R_config_list[Max_in_for_outpins];

hexCharToInt(uint8_t tempsnew) 
			{
                                           
         if (tempsnew >= '0' && tempsnew <= '9') return (tempsnew - '0');             // Check if the character is a digit 0-9
                                          
         else if (tempsnew >= 'A' && tempsnew <= 'F') return (tempsnew - 'A' + 10);    // Check if the character is an uppercase letter A-F
				
         return -1;
  	  } 
bit ProcessRxData(uint8_t Snew)
			{
			 
			 for (i = 0; i < HAMMING_R; i++)
                {
                    S_stream_expanded[(buffer_count * HAMMING_R) + i] = (Snew >> (HAMMING_R - 1 - i)) & 0x01;       // split the number in binary to 4 Bytes in buffer
                }
                
                buffer_count++;
                
                if (buffer_count == 1) // only one char make interrupt
                {
                   return buffer_flag = 1;
                }
								return 0;
  			}
void main(void)
{
    GlobalINT();
    Timer3_Init();
    UART_Init();
	  Port_Init();
	  
    while(1)
    {
        if (buffer_flag)
        {
					
            ES = 0; 
            buffer_flag = 0; 
					
            get_multi_X_from_S((uint8_t*)S_stream_expanded, R_config_list, buffer_count, X_stream_output);   
					
					  transmit_X_to_bus(X_stream_output); // Send output to bus
				  	buffer_count = 0;// Reset buffer
            ES = 1; // Re-enable Interrupts
        }
    }
}
