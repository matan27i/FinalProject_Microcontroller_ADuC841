/* File: peripherals.c */
#include <aduc841.h>
#include "header.h"
void Timer3_Init(void) // Configure Timer 3 for 9600 Baud rate
{
    T3CON &= 0xFE; 
    T3CON |= 0x86; 
    T3FD = 0x08;   
}

void UART_Init(void) // Uart 
{
    SM0 = 0;
    SM1 = 1;
    REN = 1;
    RI = 0; 
    TI = 0;
    ES = 1;
}

void GlobalINT(void) // Global Interrupt enable
{    
    EA = 1;  
}

void UART_ISR(void) interrupt 4 // ISR
{
	if (RI)
    {
				RI = 0;
        tx_temp_byte = SBUF;
        tx_flag = 1;
    }
	if (TI) TI = 0;
}