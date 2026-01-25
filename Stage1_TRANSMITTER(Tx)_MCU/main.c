/* File: main.c
 * =============================================================================
 * H1-Type Stateful Bus Encoder - Main Entry Point
 * Target: ADuC841 (8052 single-cycle core)
 * =============================================================================
 *
 * This file contains:
 * - Global variable definitions (including the critical current_bus_state)
 * - Hardware initialization
 * - Main loop handling UART reception and batch processing
 *
 * INITIALIZATION:
 * ---------------
 * current_bus_state is initialized to 0 (all-zero bus).
 * To change initialization, modify the initializer below or add an
 * explicit assignment before the main loop.
 *
 * The initial syndrome (S_initial = H * 0^T = 0) means the bus starts
 * representing syndrome 0x0.
 *
 * =============================================================================
 */

#include <aduc841.h>
#include "header.h"

/* =============================================================================
 * GLOBAL VARIABLE DEFINITIONS
 * =============================================================================
 */

/* 
 * current_bus_state: The 15-bit physical bus state vector x.
 * Only bits 0..14 are used (BUS_STATE_MASK = 0x7FFF).
 * 
 * Relationship: H * current_bus_state^T = S_current (the current syndrome)
 * 
 * INITIALIZATION: 0 (all zeros)
 * This means initial syndrome is 0, and the bus starts in the zero state.
 * 
 * To change: Assign a different value here or before the main loop.
 * Example: current_bus_state = 0x0001; // Start with syndrome 1
 */
volatile uint16_t current_bus_state = 0;

/* Legacy status flags (preserved for compatibility) */
volatile bit buffer_flag = 0;   /* Set when batch terminator received */
volatile bit tx_flag = 0;       /* Set by UART ISR when byte received */

/* Buffer tracking */
volatile uint8_t buffer_count = 0;  /* Nibble count processed */

/* UART receive buffer */
volatile uint8_t tx_temp_byte = 0;  /* Raw byte from UART ISR */

/* =============================================================================
 * MAIN FUNCTION
 * =============================================================================
 */
void main(void)
{
    /* --- Hardware Initialization --- */
    GlobalINT();        /* Enable global interrupts */
    Timer3_Init();      /* Configure Timer 3 for 9600 baud */
    UART_Init();        /* Configure UART: 8N1, 9600 baud */
    Port_Init();        /* Initialize shift register GPIO pins */
    
    /* --- Initial bus state output --- */
    /* Output the initial zero state to shift registers */
    output_to_shift_registers();
    
    /* ==========================================================================
     * MAIN LOOP
     * ==========================================================================
     * The main loop handles two events:
     * 1. tx_flag set by UART ISR: Process received character
     * 2. buffer_flag set by terminator: Perform any batch-end actions
     */
    while (1)
    {
        /* --- Handle UART Reception --- */
        if (tx_flag == 1)
        {
            tx_flag = 0;
            
            /* 
             * tx_handler() splits the character into high/low nibbles,
             * processes each through the H1 encoder, and outputs to
             * shift registers after each nibble.
             */
            tx_handler(tx_temp_byte);
        }
        
        /* --- Handle Batch Terminator --- */
        if (buffer_flag)
        {
            /* Enter critical section */
            ES = 0;  /* Disable UART interrupt */
            
            buffer_flag = 0;
            
            /*
             * Batch terminator received ('\r' or '\n').
             * 
             * In the stateful H1 encoder, each nibble is processed immediately.
             * The batch terminator is primarily a signal for the host that
             * a logical message boundary has been reached.
             * 
             * Optional: Add any batch-end processing here if needed.
             * For now, we just reset the counter for statistics.
             */
            buffer_count = 0;
            
            /* Exit critical section */
            ES = 1;  /* Re-enable UART interrupt */
        }
    }
}
