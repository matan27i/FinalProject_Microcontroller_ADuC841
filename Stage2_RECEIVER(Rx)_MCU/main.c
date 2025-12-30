/* File: main.c
 * Receiver - Reads 11-bit X from ports, decodes to S, outputs decimal ASCII via UART
 */
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
    Timer0_Init();  // Periodic sampling timer
    
    while(1)
    {
        if (sample_flag)
        {
            EA = 0;
            sample_flag = 0;
            
            // Read X from input ports
            read_X_from_bus(X);
            
            // Decode X to S using provided logic
            get_S_from_X(X, HAMMING_R, S);
            
            // Convert S bit array to decimal value
            decimal_value = bits_to_decimal(S, HAMMING_R);
            
            // Transmit as decimal ASCII
            transmit_decimal_uart(decimal_value);
            
            EA = 1;
        }
    }
}
