/* File: header.h*/
 

#ifndef HEADER_H
#define HEADER_H


typedef unsigned char uint8_t;
typedef unsigned int  uint16_t;

/*H1 Bus Encoder Constants */
#define HAMMING_R       4                       /* Number of syndrome bits (m) */
#define HAMMING_N       ((1 << HAMMING_R) - 1)  /* Bus width N = 2^R - 1 = 15 */
#define BUS_STATE_MASK  0x7FFF                  /* Mask for bits 0..14 */

/* Shift Register Pin Definitions (User-Editable)
 * Pin mapping for SN74HC595 shift registers.
 * Directly matches 74HC595 signal names for clarity.
 * Map them to actual port pins based on your PCB schematic.
 */
sbit SER_PIN   = P2^0;   /* Serial data input (74HC595 pin 14) */
sbit SRCLK_PIN = P2^1;   /* Shift register clock (74HC595 pin 11) - rising edge */
sbit RCLK_PIN  = P2^2;   /* Storage register clock / Latch (74HC595 pin 12) - rising edge */


#define DATA_PIN  SER_PIN
#define CLK_PIN   SRCLK_PIN
#define LATCH_PIN RCLK_PIN

/*Global Variables (Externs) */

/* Stateful bus state: 15-bit vector, only bits 0..14 are used.
 * current_bus_state is the "x" vector where H * x^T = S_current.
 * INITIALIZED TO ZERO in main.c. */
extern volatile uint16_t current_bus_state;

/* Legacy flags (preserved from original codebase) */
extern volatile bit buffer_flag;        /* Flag: Ready to process batch */
extern volatile bit tx_flag;            /* Flag: New byte received from ISR */
extern volatile uint8_t buffer_count;   /* Current nibble count in buffer */
extern volatile uint8_t tx_temp_byte;   /* Raw byte from UART ISR */

/*Function Prototypes  */

/* Hardware Initialization */
void Timer3_Init(void);
void UART_Init(void);
void GlobalINT(void);
void Port_Init(void);

/* H1-Type Bus Encoder Core Functions */

/*process_nibble - Process a single 4-bit syndrome (S_new) and update bus state
 * s_new: The new 4-bit syndrome value (0x0 to 0xF)
 * 
 * This function:
 * 1. Computes S_old from current_bus_state via H * current_bus_state^T
 * 2. Computes S_target = S_new ^ S_old
 * 3. Finds minimal-weight w such that H * w^T == S_target
 * 4. Updates current_bus_state ^= w
 * 5. Outputs the new state to shift registers
 */
void process_nibble(uint8_t s_new);

/**
 * compute_syndrome_from_bus - Compute H * x^T on-the-fly
 * bus_state: The 15-bit bus state x (bits 0..14)
 * return: The 4-bit syndrome S = H * x^T
 * 
 * Column i (1..15) of the H1 matrix is just the binary representation of i.
 * This function computes the syndrome using bitwise XOR of column indices
 * for all bits that are set in bus_state.
 */
uint8_t compute_syndrome_from_bus(uint16_t bus_state);

/*find_minimal_w - Find minimal Hamming-weight vector w such that H*w^T = s_target
 * s_target: The target 4-bit syndrome (0x0 to 0xF)
 * return: The 15-bit w vector with minimal Hamming weight
 * 
 * Algorithm: Bounded search exploiting H1-type structure.
 * For any nonzero s_target, weight-1 solution exists (w = single bit at position s_target-1).
 * If s_target is zero, w = 0 (weight-0).
 * Tie-breaker: smallest numeric value of w.
 */
uint16_t find_minimal_w(uint8_t s_target);

/*output_to_shift_registers - Bit-bang current_bus_state to chained 74HC595 shift registers
 * 
 * Shift order: MSB-first (bit 14 down to bit 0).
 * Protocol: For each bit, set SER then pulse SRCLK; finally pulse RCLK to latch.
 * CLK timing targets ~100 kHz with NOP-based delays.
 */
void output_to_shift_registers(void);

/*tx_handler - Handle received UART character
 * rx_char: The received character
 * 
 * For non-terminator characters:
 * Splits character into high nibble and low nibble
 * Processes high nibble first, then low nibble
 * For '\r' or '\n': sets buffer_flag (preserved for compatibility)
 */
void tx_handler(uint8_t rx_char);

#endif
