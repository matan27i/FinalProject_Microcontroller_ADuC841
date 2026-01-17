/* File: Rx_output.c
 * Transmits value as HEX character (0-9, A-F) via UART
 */
#include <aduc841.h>
#include "header.h"

void transmit_hex_uart(uint8_t value)
{
    // Limit processing to 4-bit nibble (0-15)
    // If you expect larger numbers, this logic needs adjustment.
    value = value & 0x0F; 

    // Wait for previous transmission to complete
    while (!TI);
    TI = 0;

    // Convert to ASCII
    if (value <= 9)
    {
        // '0' (48) + value -> '0' to '9'
        SBUF = '0' + value;
    }
    else
    {
        // 'A' (65) + (value - 10) -> 'A' to 'F'
        SBUF = 'A' + (value - 10);
    }
    
    // --- Send Newline (\r\n) for PuTTY ---
    
    while (!TI);
    TI = 0;
    SBUF = '\r';
    
    while (!TI);
    TI = 0;
    SBUF = '\n';
}