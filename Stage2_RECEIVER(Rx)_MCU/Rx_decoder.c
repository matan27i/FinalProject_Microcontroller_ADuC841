/**
 * rx_decoder.c
 * 
 * Implementation of Stage 2 Decoder & Reassembler
 * Target: ADuC841 (8052 core) with Keil C51 compiler
 */

#include "rx_decoder.h"
#include <reg52.h>   // Standard 8052 register definitions (compatible with ADuC841)

// ---------------------------------------------------------------------------
// State Machine for Nibble Reassembly
// ---------------------------------------------------------------------------

typedef enum {
    WAITING_HIGH_NIBBLE = 0,    // Expecting high nibble (first cycle)
    WAITING_LOW_NIBBLE  = 1     // Expecting low nibble (second cycle)
} rx_state_t;

// Static variables for state machine
static rx_state_t current_state = WAITING_HIGH_NIBBLE;
static uint8_t stored_high_nibble = 0;

// ---------------------------------------------------------------------------
// Function Implementations
// ---------------------------------------------------------------------------

/**
 * Initialize the decoder state machine
 */
void rx_decoder_init(void) {
    current_state = WAITING_HIGH_NIBBLE;
    stored_high_nibble = 0;
}

/**
 * Compute syndrome of 15-bit bus state
 * 
 * Algorithm:
 * For H1 matrix, column i (1-indexed) has binary value i.
 * Syndrome S = XOR of all column indices where bit is set.
 * 
 * Example:
 * If bits 3, 5, 7 are set (0-indexed), we XOR columns 4, 6, 8:
 * S = 4 ^ 6 ^ 8 = 0100 ^ 0110 ^ 1000 = 1010 = 0xA
 * 
 * @param bus_state: 15-bit corrected bus state
 * @return: 4-bit syndrome (equals data nibble)
 */
uint8_t compute_syndrome(uint16_t bus_state) {
    uint8_t syndrome = 0;
    uint8_t bit_index;
    
    // XOR indices of all set bits (using 1-based column numbering)
    for (bit_index = 0; bit_index < 15; bit_index++) {
        if (bus_state & (1u << bit_index)) {
            // Column index = bit_index + 1 (convert 0-indexed to 1-indexed)
            syndrome ^= (bit_index + 1);
        }
    }
    
    // Mask to 4 bits (valid range: 0x0 - 0xF)
    return syndrome & 0x0F;
}

/**
 * Process incoming bus state and reassemble bytes
 * 
 * This function implements a simple 2-state FSM:
 * 
 * State 1 (WAITING_HIGH_NIBBLE):
 *   - Extract syndrome from bus state
 *   - Store as high nibble
 *   - Transition to WAITING_LOW_NIBBLE
 * 
 * State 2 (WAITING_LOW_NIBBLE):
 *   - Extract syndrome from bus state  
 *   - Combine with stored high nibble
 *   - Send complete byte via UART
 *   - Transition back to WAITING_HIGH_NIBBLE
 * 
 * @param current_bus_state: 15-bit corrected bus state from Stage 1
 */
void rx_process_state(uint16_t current_bus_state) {
    uint8_t data_nibble;
    uint8_t complete_byte;
    
    // Step 1: Extract the data nibble from current bus state
    data_nibble = compute_syndrome(current_bus_state);
    
    // Step 2: Process according to current state
    switch (current_state) {
        
        case WAITING_HIGH_NIBBLE:
            // First cycle: Store high nibble and wait for low nibble
            stored_high_nibble = data_nibble;
            current_state = WAITING_LOW_NIBBLE;
            break;
            
        case WAITING_LOW_NIBBLE:
            // Second cycle: Combine nibbles and transmit
            complete_byte = (stored_high_nibble << 4) | (data_nibble & 0x0F);
            UART_Tx(complete_byte);
            
            // Reset state machine for next character
            current_state = WAITING_HIGH_NIBBLE;
            stored_high_nibble = 0;
            break;
            
        default:
            // Safety: Reset to known state if corrupted
            current_state = WAITING_HIGH_NIBBLE;
            stored_high_nibble = 0;
            break;
    }
}

/**
 * Transmit a byte via UART
 * 
 * Uses standard 8051 UART (compatible with ADuC841):
 * - SBUF: Serial buffer register
 * - TI: Transmit interrupt flag (bit 1 of SCON)
 * 
 * Prerequisites:
 * - UART must be initialized (baud rate, mode, enable)
 * - TI should be cleared after initialization
 * 
 * @param c: Character/byte to transmit
 */
void UART_Tx(char c) {
    // Wait for previous transmission to complete
    // TI is set by hardware when transmission is complete
    while (!TI);
    
    // Clear transmit interrupt flag
    TI = 0;
    
    // Load byte into UART transmit buffer
    // Hardware will automatically start transmission
    SBUF = c;
}

/**
 * Optional: Get current decoder state (for debugging)
 * 
 * @return: 0 if waiting for high nibble, 1 if waiting for low nibble
 */
uint8_t rx_get_state(void) {
    return (uint8_t)current_state;
}
