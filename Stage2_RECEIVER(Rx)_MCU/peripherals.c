/* File: peripherals.c */
#include <aduc841.h>
#include "header.h"

void Timer3_Init(void)
{
    T3CON &= 0xFE;
    T3CON |= 0x86;
    T3FD = 0x08;
}

void UART_Init(void)
{
    SM0 = 0;
    SM1 = 1;
    REN = 0;  // Transmit only
    TI = 1;   // Set TI to allow first transmission
}

void GlobalINT(void)
{
    EA = 1;
}

void Port_Init(void)
{
    // Port 2: Input (all bits = 1 for high-impedance input)
    P2 = 0xFF;
    
    // Port 3: P3.4-P3.7 as inputs
    P3 |= 0xF0;
}

void Timer0_Init(void)
{
    // Mode 1: 16-bit timer, sample every ~10ms
    TMOD &= 0xF0;
    TMOD |= 0x01;
    
    TH0 = 0xD8;  // Reload for ~10ms @ 11.0592MHz
    TL0 = 0xF0;
    
    ET0 = 1;  // Enable Timer0 interrupt
    TR0 = 1;  // Start timer
}

void Timer0_ISR(void) interrupt 1
{
    TH0 = 0xD8;
    TL0 = 0xF0;
    sample_flag = 1;
}
