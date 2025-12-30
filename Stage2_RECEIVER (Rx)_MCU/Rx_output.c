/* File: rx_output.c
 * Transmits decimal value as ASCII string via UART
 */
#include <aduc841.h>
#include "header.h"

void transmit_decimal_uart(uint8_t value)
{
    uint8_t digits[3];  // Max 2 digits + null
    uint8_t i = 0;
    uint8_t j;
    
    // Convert to decimal digits (reversed)
    if (value == 0)
    {
        digits[i++] = '0';
    }
    else
    {
        while (value > 0 && i < 2)
        {
            digits[i++] = '0' + (value % 10);
            value /= 10;
        }
    }
    
    // Transmit in correct order (reverse)
    for (j = i; j > 0; j--)
    {
        while (!TI);
        TI = 0;
        SBUF = digits[j - 1];
    }
    
    // Transmit newline
    while (!TI);
    TI = 0;
    SBUF = '\r';
    
    while (!TI);
    TI = 0;
    SBUF = '\n';
}
