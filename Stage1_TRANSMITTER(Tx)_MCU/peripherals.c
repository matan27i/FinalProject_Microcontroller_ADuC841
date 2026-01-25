/* File: peripherals.c
 * =============================================================================
 * Peripheral Configuration for ADuC841
 * UART and Timer Setup for H1-Type Bus Encoder
 * =============================================================================
 */

#include <aduc841.h>
#include "header.h"

/* ---------------------------------------------------------------------------
 * Timer3_Init
 * ---------------------------------------------------------------------------
 * Configure Timer 3 as UART baud rate generator for 9600 baud.
 * 
 * ADuC841 Timer 3 configuration for 9600 baud with 11.0592 MHz crystal:
 * T3CON settings and T3FD fractional divider per datasheet.
 * ---------------------------------------------------------------------------
 */
void Timer3_Init(void)
{
    T3CON &= 0xFE;  /* Disable Timer 3 */
    T3CON |= 0x86;  /* Set mode bits for UART baud generation */
    T3FD = 0x08;    /* Fractional divider for 9600 baud accuracy */
}

/* ---------------------------------------------------------------------------
 * UART_Init
 * ---------------------------------------------------------------------------
 * Initialize UART for 8-bit data, no parity, 1 stop bit (8N1).
 * Baud rate is set by Timer 3.
 * Enable receive and serial interrupt.
 * ---------------------------------------------------------------------------
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

/* ---------------------------------------------------------------------------
 * GlobalINT
 * ---------------------------------------------------------------------------
 * Enable global interrupt master switch.
 * ---------------------------------------------------------------------------
 */
void GlobalINT(void)
{
    EA = 1;     /* Enable All interrupts */
}

/* ---------------------------------------------------------------------------
 * UART_ISR
 * ---------------------------------------------------------------------------
 * UART Interrupt Service Routine (Interrupt 4).
 * 
 * On receive (RI):
 *   - Clear RI flag
 *   - Copy SBUF to tx_temp_byte
 *   - Set tx_flag to signal main loop
 *
 * On transmit (TI):
 *   - Clear TI flag (not used for transmission in this application)
 * ---------------------------------------------------------------------------
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
