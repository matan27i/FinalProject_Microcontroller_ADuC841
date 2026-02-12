/**
 * main.c
 * 
 * Example integration of Stage 2 Decoder for ADuC841
 * 
 * This demonstrates how to use the rx_decoder module in your main application.
 * Assumes Stage 1 (error correction) provides corrected_bus_state.
 */

#include <reg52.h>
#include "rx_decoder.h"

// ---------------------------------------------------------------------------
// UART Initialization (example for 9600 baud at 11.0592 MHz crystal)
// ---------------------------------------------------------------------------

void UART_Init(void) {
    // Configure UART Mode 1 (8-bit UART, variable baud rate)
    SCON = 0x50;    // Mode 1: 8-bit UART, REN=1 (enable receiver)
    
    // Configure Timer 1 for baud rate generation
    TMOD |= 0x20;   // Timer 1 in Mode 2 (8-bit auto-reload)
    TH1 = 0xFD;     // Reload value for 9600 baud @ 11.0592 MHz
    TL1 = 0xFD;     // Initial value
    TR1 = 1;        // Start Timer 1
    
    TI = 0;         // Clear transmit interrupt flag
    RI = 0;         // Clear receive interrupt flag
}

// ---------------------------------------------------------------------------
// External Interface (Stage 1 output)
// ---------------------------------------------------------------------------

// This function should be called by Stage 1 after error correction
// You can implement this as an interrupt handler or polling function
extern uint16_t get_corrected_bus_state(void);  // Provided by Stage 1

// ---------------------------------------------------------------------------
// Main Application Loop
// ---------------------------------------------------------------------------

void main(void) {
    uint16_t corrected_bus_state;
    
    // Initialize UART
    UART_Init();
    
    // Initialize decoder state machine
    rx_decoder_init();
    
    // Main loop
    while (1) {
        // Wait for Stage 1 to provide a corrected bus state
        // (Implementation depends on your Stage 1 interface)
        // Options:
        //   1. Polling: Check a flag or register
        //   2. Interrupt-driven: Process in ISR
        //   3. DMA/FIFO: Read from buffer
        
        // Example: Polling approach
        if (/* condition indicating new data available */) {
            // Get the corrected 15-bit bus state from Stage 1
            corrected_bus_state = get_corrected_bus_state();
            
            // Process the bus state (decode and reassemble)
            rx_process_state(corrected_bus_state);
            
            // The decoder will automatically send complete bytes via UART
            // after receiving both high and low nibbles
        }
        
        // Other tasks can be performed here
    }
}

// ---------------------------------------------------------------------------
// Alternative: Interrupt-Driven Example
// ---------------------------------------------------------------------------

#if 0  // Enable this section if using interrupt-driven approach

// External interrupt (e.g., INT0) triggered when new bus state is ready
void ext_int0_isr(void) interrupt 0 {
    uint16_t corrected_bus_state;
    
    // Read corrected bus state from Stage 1 hardware/registers
    corrected_bus_state = get_corrected_bus_state();
    
    // Process immediately
    rx_process_state(corrected_bus_state);
    
    // Clear interrupt flag (if necessary)
}

void main_interrupt_driven(void) {
    // Initialize UART
    UART_Init();
    
    // Initialize decoder
    rx_decoder_init();
    
    // Configure external interrupt
    IT0 = 1;    // Edge-triggered on INT0
    EX0 = 1;    // Enable INT0
    EA = 1;     // Enable global interrupts
    
    // Main loop can perform other tasks
    while (1) {
        // Decoder runs automatically in ISR
    }
}

#endif

// ---------------------------------------------------------------------------
// Alternative: Direct Call Example (for testing)
// ---------------------------------------------------------------------------

#if 0  // Enable this section for simple testing

void test_decoder(void) {
    uint16_t test_states[] = {
        0x421F,  // Example: encoded high nibble
        0x1234   // Example: encoded low nibble
    };
    uint8_t i;
    
    // Initialize
    UART_Init();
    rx_decoder_init();
    
    // Process test states
    for (i = 0; i < 2; i++) {
        rx_process_state(test_states[i]);
        // After second state, a complete byte is transmitted
    }
}

#endif
