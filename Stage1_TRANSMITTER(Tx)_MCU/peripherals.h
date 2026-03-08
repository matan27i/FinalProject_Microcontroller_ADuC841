/* File: peripherals.h
   Hardware definitions, initializations, and peripheral states.
*/
#ifndef PERIPHERALS_H
#define PERIPHERALS_H

#include "common_types.h"

/* Shift Register Pin Definitions (sbit for Keil 8051) */
sbit SR_CLK   = P2^0;
sbit SR_DATA  = P2^1;
sbit SR_LATCH = P2^3;
sbit SR_CLR   = P2^4;

/* Timer 2 and hardware flags */
extern volatile uint8_t counter_t2;
extern volatile uint8_t flag_t2_mod;



/* Hardware Initialization Prototypes */
void Port_Init(void);
void Init_Timer2(void);
void Timer3_Init(void);
void UART_Init(void);
void GlobalINT(void);


#endif /* PERIPHERALS_H */