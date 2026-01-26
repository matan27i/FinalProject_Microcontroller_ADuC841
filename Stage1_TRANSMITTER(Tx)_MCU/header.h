/* File: header.h */
#ifndef HEADER_H
#define HEADER_H

// --- 1. Type Definitions ---
typedef unsigned char uint8_t;

// --- 2. System Constants ---
#define MAX_BLOCKS   4                   // Maximum number of sub-matrices (blocks) supported
#define TOTAL_X_BITS 64                  // Maximum total size of the X vector (Information bits)

// --- 3. Global Variables (Externs) ---
// Flags & Status
extern volatile bit buffer_flag;         // Flag: Indicates a full batch of syndromes is ready
extern volatile bit tx_flag;             // Flag: New byte received from UART ISR
extern volatile uint8_t buffer_count;    // Current index in the input buffer
extern volatile uint8_t tx_temp_byte;    // Raw byte received from ISR

// Code Configuration (Defined in main.c)
extern uint8_t code_config_R[];          // Array defining 'R' (parity bits) for each block
extern uint8_t num_active_blocks;        // Number of active blocks in the diagonal matrix

// State Variables (Memory of the system)
extern uint8_t xdata current_S_state[MAX_BLOCKS];  // Stores the last known Syndrome (W) for each block
extern uint8_t xdata X_global_state[TOTAL_X_BITS]; // Stores the current Error Vector (X)

// Hardware Pins (ADuC841)
sbit SR_DATA  = P3^4;  // Shift Register Data
sbit SR_CLOCK = P2^0;
sbit SR_LATCH = P3^6;  // Shift Register Latch

// Input Buffer
extern uint8_t xdata S_input_buffer[MAX_BLOCKS];   // Buffer to store incoming syndromes from UART

// --- 4. Function Prototypes ---
void Timer3_Init(void);
void UART_Init(void);
void GlobalINT(void);
void Port_Init(void);
void long_delay(void);
void tx_handler(uint8_t rx_char);

// Function to shift the X vector out to hardware
void transmit_X_to_shift_reg(const uint8_t *X_stream_output, uint8_t total_bits);

// Core function for the Block Diagonal "Delta" logic
void process_diagonal_system(uint8_t *new_S_vector);
void Init_Bus_State(void);

#endif