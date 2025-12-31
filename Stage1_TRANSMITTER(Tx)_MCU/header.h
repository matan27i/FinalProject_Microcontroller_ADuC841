/* File: header.h */

#ifndef HEADER_H
#define HEADER_H

// --- 1. Type Definitions ---
typedef unsigned char uint8_t;

// --- 2. Hamming & Buffer Constants ---
#define HAMMING_R    4                  // Number of parity bits (R)
#define HAMMING_N    ((1 << HAMMING_R) - 1) // Block length (N=15)
#define MAX_PACKETS  10                 // Max input characters to buffer

// --- 3. Global Variables (Externs) ---
// Flags & Status
extern volatile bit buffer_flag;        // Flag: Ready to process batch
extern volatile bit tx_flag;            // Flag: New byte received from ISR
extern volatile uint8_t buffer_count;   // Current packet count
extern volatile uint8_t Snew;           // Last parsed hex value

// Data Storage
extern volatile uint8_t tx_temp_byte;   // Raw byte from ISR
// Expanded bit stream in external memory (xdata)
extern volatile uint8_t xdata S_stream_expanded[MAX_PACKETS * HAMMING_R];

// --- 4. Function Prototypes ---
// Hardware Initialization
void Timer3_Init(void);
void UART_Init(void);
void GlobalINT(void);

// Data Processing & Logic
void tx_handler(uint8_t rx_char);
uint8_t get_X_from_S(const uint8_t *S_vector, uint8_t hamming_R, uint8_t *X_output);
uint8_t get_multi_X_from_S(const uint8_t *S_stream, const uint8_t *R_list, uint8_t packet_count, uint8_t *X_stream);
void Port_Init(void);
void transmit_X_to_bus(const uint8_t *X_stream_output);

#endif
