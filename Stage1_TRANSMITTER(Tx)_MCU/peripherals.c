/* File: peripherals.c
  Peripheral Configuration for ADuC841
  UART and Timer Setup for H1-Type Bus Encoder
 */

#include <aduc841.h>
#include "header.h"

volatile uint8_t counter_t2 = 0;
volatile uint8_t flag_t2_mod = 0;
\
void Port_Init(void)
{
		SR_CLK = 0;
    SR_LATCH = 1;
    SR_CLR = 1;
}

void Init_Timer2(void) 
	{
    T2CON = 0x00;
    RCAP2H = 0xD4;
    RCAP2L = 0xCD;
    ET2 = 1;
    EA = 1;
    TR2 = 0;
	}
	
/* 
  Timer3_Init
-
  Configure Timer 3 as UART baud rate generator for 9600 baud.
  
  ADuC841 Timer 3 configuration for 9600 baud with 11.0592 MHz crystal:
  T3CON settings and T3FD fractional divider per datasheet.
 */
void Timer3_Init(void)
{
    T3CON &= 0xFE;  /* Disable Timer 3 */
    T3CON |= 0x86;  /* Set mode bits for UART baud generation */
    T3FD = 0x08;    /* Fractional divider for 9600 baud accuracy */
}

/*
  UART_Init

  Initialize UART for 8-bit data, no parity, 1 stop bit (8N1).
  Baud rate is set by Timer 3.
  Enable receive and serial interrupt.
 */
void UART_Init(void)
{
    SM0 = 0;    /* Mode 1: 8-bit UART */
    SM1 = 1;    /* Mode 1: variable baud rate */
    REN = 1;    /* Enable receiver */
    RI = 0;     /* Clear receive interrupt flag */
    TI = 0;     /* Clear transmit interrupt flag */
    ES = 1;     /* Enable serial interrupt */
}

/*
  GlobalINT

  Enable global interrupt master switch.
  
 */
void GlobalINT(void)
{
    EA = 1;     /* Enable All interrupts */
}

/* 
  UART_ISR
  UART Interrupt Service Routine (Interrupt 4).
  
  On receive (RI):
    - Clear RI flag
    - Copy SBUF to tx_temp_byte
    - Set tx_flag to signal main loop
 
  On transmit (TI):
    - Clear TI flag (not used for transmission in this application)
 */
void UART_ISR(void) interrupt 4
{
    if (RI)
    {
        RI = 0;                 /* Clear receive interrupt flag */
        tx_temp_byte = SBUF;    /* Copy received byte */
        tx_flag = 1;            /* Signal main loop */
    }
    if (TI)
    {
        TI = 0;                 /* Clear transmit interrupt flag */
    }
}

	
	
void Timer2_ISR(void) interrupt 5 
{
    TF2 = 0; 
    if (flag_t2_mod == 0)
    {
        if(counter_t2 < 2) 
        {
            SR_CLK = !SR_CLK; 
            counter_t2++;
        }
        else
        {
            TR2 = 0;      
            SR_CLK = 0;    
        }
    }
    else if (flag_t2_mod == 1)
    {
        if(counter_t2 < 4) 
        {
					SR_CLR = 0;
					SR_LATCH = 0;
					SR_CLK = !SR_CLK;
          counter_t2++;
        }
        else
        {
            TR2 = 0;
            SR_LATCH = 1;
            SR_CLR = 1;
            flag_t2_mod = 0; 
        }
    }
    else if (flag_t2_mod == 2)
    {
        if(counter_t2 < 2)
        {
            SR_LATCH = !SR_LATCH; 
            counter_t2++;
        }
        else
        {
            TR2 = 0;
            flag_t2_mod = 0;   
        }
    }
}