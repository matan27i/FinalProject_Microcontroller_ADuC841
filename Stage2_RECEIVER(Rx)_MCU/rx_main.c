/* File: rx_main.c

   H1-Type Stateful Bus Decoder - Main Entry Point (RECEIVER SIDE)
   Target: ADuC841 (8052 single-cycle core)

 
   This file contains:
   - Global variable definitions (decoder state, counters, flags)
   - Hardware initialization
   - Main loop handling bus state reception and nibble reassembly
 
   INITIALIZATION:
   rx_current_state is initialized to RX_STATE_WAIT_HIGH (waiting for high nibble).
   rx_stored_high_nibble is initialized to 0.
 
   OPERATION:
   1. Wait for DATA_READY signal from Stage 1
   2. Read corrected 15-bit bus state
   3. Compute syndrome (extract data nibble)
   4. Reassemble nibbles into bytes
   5. Transmit complete bytes via UART
 
   SYNCHRONIZATION:
   If the decoder loses synchronization (wrong character output),
   call rx_reset_state_machine() to resync to high nibble boundary.

 */

#include <aduc841.h>
#include "rx_header.h"

/* 
   GLOBAL VARIABLE DEFINITIONS
 */

/* Decoder state machine current state
   Initialized to WAIT_HIGH: expecting high nibble first
 */
volatile rx_state_t rx_current_state = RX_STATE_WAIT_HIGH;

/* Stored high nibble (used when assembling complete byte)
   When in WAIT_LOW state, this holds the high nibble from previous cycle
 */
volatile uint8_t rx_stored_high_nibble = 0;

/* Status flags */
volatile bit rx_byte_ready = 0;        /* Set when complete byte assembled */
volatile bit rx_data_available = 0;    /* Set when new bus state from Stage 1 */

/* Statistics counters (optional, for debugging/monitoring) */
volatile uint16_t rx_bytes_decoded = 0;     /* Count of bytes successfully decoded */
volatile uint16_t rx_states_processed = 0;  /* Count of bus states processed */

/* 
   MAIN FUNCTION
 
   Initializes hardware and enters main loop.
   Main loop polls for new bus states from Stage 1 and processes them.
 
   Alternative implementations:
   - Interrupt-driven: Use external interrupt when DATA_READY asserts
   - Timer-based: Sample bus state at fixed intervals
   - DMA/FIFO: Process buffered states from Stage 1
 */
void main(void)
{
    uint16_t bus_state;
    
    /* --- Hardware Initialization --- */
    RX_GlobalINT();        /* Enable global interrupts */
    RX_Timer3_Init();      /* Configure Timer 3 for 9600 baud */
    RX_UART_Init();        /* Configure UART: 8N1, 9600 baud, TX enabled */
    RX_Port_Init();        /* Initialize interface pins to Stage 1 */
    
    /* --- Reset decoder state machine --- */
    rx_reset_state_machine();
    
    /* --- Optional: Send startup message to PC --- */
    /* Uncomment if you want to signal RX is ready:
    rx_send_uart_byte('R');
    rx_send_uart_byte('X');
    rx_send_uart_byte(':');
    rx_send_uart_byte('O');
    rx_send_uart_byte('K');
    rx_send_uart_byte('\r');
    rx_send_uart_byte('\n');
    */
    
    /*
       MAIN LOOP
       
       This implementation uses polling to check for new data from Stage 1.
       
       Workflow:
       1. Check if DATA_READY signal is asserted (or check status register)
       2. Read 15-bit bus state from Stage 1 interface
       3. Process bus state (compute syndrome, reassemble nibbles)
       4. Complete bytes are automatically sent via UART in rx_process_bus_state()
       
       Alternative: Use external interrupt (INT0 or INT1) triggered by
       DATA_READY signal for immediate response without polling.
     */
    while (1)
    {
        /* --- Poll for new data from Stage 1 --- */
        /* 
           USER CUSTOMIZATION REQUIRED:
           Modify this condition based on your Stage 1 interface.
           
           Examples:
           - if (DATA_READY == 1)  // Hardware pin
           - if (rx_data_available) // Set by ISR
           - if (*STAGE1_STATUS_REG & 0x01)  // Status register
         */
        if (DATA_READY)  /* Example: Pin P3.2 asserted by Stage 1 */
        {
            /* Read 15-bit bus state from Stage 1 */
            bus_state = rx_read_bus_state();
            
            /* Process the bus state (decode and reassemble) */
            rx_process_bus_state(bus_state);
            
            /* Update statistics */
            rx_states_processed++;
            
            /* Clear data ready flag if needed */
            /* If using software flag: rx_data_available = 0; */
            
            /* Optional: Add delay if Stage 1 needs time between reads */
            /* Adjust based on Stage 1 timing requirements */
        }
        
        /*
           Optional: Add other tasks here
           - Error checking
           - Timeout detection
           - Sync reset on loss of valid data
           - LED status indication
         */
    }
}

/*
   ALTERNATIVE IMPLEMENTATION: Interrupt-Driven Reception
   
   If Stage 1 asserts an interrupt when new data is ready,
   use this approach instead of polling:
 */
#if 0  /* Set to 1 to enable interrupt-driven mode */

/* External interrupt 0 ISR - triggered by Stage 1 DATA_READY signal */
void EXT0_ISR(void) interrupt 0
{
    uint16_t bus_state;
    
    /* Read bus state from Stage 1 */
    bus_state = rx_read_bus_state();
    
    /* Process immediately */
    rx_process_bus_state(bus_state);
    
    /* Update statistics */
    rx_states_processed++;
    
    /* ISR automatically clears interrupt flag */
}

void main_interrupt_driven(void)
{
    /* Initialize hardware */
    RX_GlobalINT();
    RX_Timer3_Init();
    RX_UART_Init();
    RX_Port_Init();
    rx_reset_state_machine();
    
    /* Configure external interrupt 0 */
    IT0 = 1;    /* Edge-triggered on INT0 (falling edge) */
    EX0 = 1;    /* Enable INT0 */
    EA = 1;     /* Enable global interrupts */
    
    /* Main loop can perform other tasks */
    while (1)
    {
        /* Decoding happens automatically in ISR */
        /* Add other background tasks here */
    }
}

#endif  /* End interrupt-driven alternative */
