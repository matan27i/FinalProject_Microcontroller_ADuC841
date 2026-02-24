/* File: peripherals.c
  Peripheral Configuration for ADuC841
  UART and Timer Setup for H1-Type Bus Encoder
 */

#include <aduc841.h>
#include "header.h"

volatile uint8_t counter_t2 = 0;
volatile uint8_t flag_t2_mod = 0;
/* Dynamic PESEC Variables */
#define MAX_PESEC_BLOCKS 5

volatile uint16_t pesec_redundancy_reg = 0;

/* Arrays to store dynamic matrices (Allocated ONLY HERE) */
uint8_t PESEC_MAT_D[20]; 
uint8_t PESEC_MAT_A[40]; 

/* Block management arrays for O(1) solving */
uint8_t pesec_num_blocks;
uint8_t pesec_bit_offsets[MAX_PESEC_BLOCKS];
uint8_t pesec_col_offsets[MAX_PESEC_BLOCKS];
uint8_t pesec_chunk_masks[MAX_PESEC_BLOCKS];

uint8_t pesec_num_d_cols;
uint8_t pesec_num_a_cols;


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

/* Init_PESEC_Matrices
  Generates Matrix D and Matrix A dynamically based on an array of block sizes.
*/

void Init_PESEC_Matrices(uint8_t* m_sizes, uint8_t num_blocks) 
{
    uint8_t i, b, val;
    uint8_t is_in_d;
    uint8_t total_m = 0;
    uint8_t current_col_offset = 0;
    uint8_t max_syndrome;
    
    pesec_num_blocks = num_blocks;
    pesec_num_d_cols = 0;
    pesec_num_a_cols = 0;
    
    /* --- Build Matrix D and Pre-calculate Solver Data --- */
    for (b = 0; b < num_blocks; b++) 
    {
        /* Store offsets and masks for fast delta solving */
        pesec_bit_offsets[b] = total_m;
        pesec_col_offsets[b] = current_col_offset;
        pesec_chunk_masks[b] = ((1 << m_sizes[b]) - 1) << total_m;
        
        /* Build Matrix D block columns */
        for (i = 1; i < (1 << m_sizes[b]); i++) 
        {
            PESEC_MAT_D[pesec_num_d_cols++] = (i << total_m);
        }
        
        /* Advance global offsets for the next block */
        total_m += m_sizes[b];
        current_col_offset += ((1 << m_sizes[b]) - 1);
    }
    
    max_syndrome = (1 << total_m) - 1;
    
    /* --- Build Matrix A (Everything not in D) --- */
    for (val = 1; val <= max_syndrome; val++) 
    {
        is_in_d = 0;
        
        for (i = 0; i < pesec_num_d_cols; i++) 
        {
            if (PESEC_MAT_D[i] == val) 
            {
                is_in_d = 1;
                break;
            }
        }
        
        if (is_in_d == 0) 
        {
            PESEC_MAT_A[pesec_num_a_cols++] = val;
        }
    }
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