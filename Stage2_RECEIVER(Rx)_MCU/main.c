/* File: main.c */
#include <aduc841.h>
#include "header.h"

volatile bit sample_flag = 0;

void main(void)
{
    uint8_t X[HAMMING_N];
    uint8_t S[HAMMING_R];
    uint8_t decimal_value;
    
    GlobalINT();
    Timer3_Init();
    UART_Init();
    Port_Init();
    Timer0_Init(); 
    
    // DAC Configuration
    DACCON = 0x1D;
    DAC0H = 0x0F;
    DAC0L = 0xFF;
	
    while(1)
    {
        if (sample_flag)
        {
            EA = 0;
            sample_flag = 0;
            
            read_X_from_bus(X);
            get_S_from_X(X, HAMMING_R, S);
            decimal_value = bits_to_decimal(S, HAMMING_R);
            
            // Filter: Only transmit if value is not 0
            if (decimal_value != 0)
            {
                // CHANGE: Call the HEX transmit function
                transmit_hex_uart(decimal_value);
            }
            
            EA = 1;
        }
    }
}