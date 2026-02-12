/* File: rx_header.h

   H1-Type Stateful Bus Decoder for ADuC841 (RECEIVER SIDE)

   
   README BLOCK (max 10 lines):
   - Input: Each 15-bit bus state is received from Stage 1 (error corrector).
     Stage 1 provides SEC-DED corrected bus states.
   - Decoding: Syndrome S = H * bus_state^T extracts the 4-bit data nibble.
   - Reassembly: Two sequential bus states → HIGH nibble, then LOW nibble → 8-bit byte.
   - Output: Reconstructed bytes are transmitted via UART to PC.
   - Timing: UART configured for 9600 baud (same as TX side).
   - The H1-type matrix columns are indexed 1..15, mapping to bits 0..14 of bus state.
   - Interface: Stage 1 output is accessed via external port or interface (user-configurable).

 */

#ifndef RX_HEADER_H
#define RX_HEADER_H

/* 1. Type Definitions */
typedef unsigned char uint8_t;
typedef unsigned int  uint16_t;

/* 2. H1 Bus Decoder Constants */
#define HAMMING_R       4                       /* Number of syndrome bits (m) */
#define HAMMING_N       ((1 << HAMMING_R) - 1)  /* Bus width N = 2^R - 1 = 15 */
#define BUS_STATE_MASK  0x7FFF                  /* Mask for bits 0..14 */

/* 3. Stage 1 Interface Pin Definitions (User-Editable)
   These pins connect to the Stage 1 error corrector output.
   Modify based on your hardware interface design.
   
   Example configurations:
   - Direct port mapping: P0 = bits 0-7, P1.0-P1.6 = bits 8-14
   - Shift register input: Serial data from Stage 1
   - Parallel interface: Direct 15-bit bus from error corrector
 */

/* Example: Parallel interface using P0 and P1 */
/* Uncomment and modify based on your actual hardware: */
/* 
#define BUS_LOW_BYTE    P0          // Bits 0-7
#define BUS_HIGH_PORT   P1          // Bits 8-14 on P1.0-P1.6
#define BUS_READY_FLAG  P3^2        // Signal from Stage 1: new data ready
*/

/* Example: Serial interface flag */
sbit DATA_READY = P3^2;  /* Signal from Stage 1: corrected bus state available */

/* 4. Decoder State Machine States */
typedef enum {
    RX_STATE_WAIT_HIGH = 0,  /* Waiting for high nibble */
    RX_STATE_WAIT_LOW  = 1   /* Waiting for low nibble */
} rx_state_t;

/* 5. Global Variables (Externs) */

/* Decoder state machine current state */
extern volatile rx_state_t rx_current_state;

/* Stored high nibble (waiting for low nibble to complete byte) */
extern volatile uint8_t rx_stored_high_nibble;

/* Status flags */
extern volatile bit rx_byte_ready;       /* Flag: Complete byte assembled and sent */
extern volatile bit rx_data_available;   /* Flag: New bus state available from Stage 1 */

/* Statistics/debugging counters */
extern volatile uint16_t rx_bytes_decoded;   /* Total bytes successfully decoded */
extern volatile uint16_t rx_states_processed; /* Total bus states processed */

/* 6. Function Prototypes */

/* Hardware Initialization */
void RX_Timer3_Init(void);
void RX_UART_Init(void);
void RX_GlobalINT(void);
void RX_Port_Init(void);

/* H1-Type Bus Decoder Core Functions */

/**
  rx_compute_syndrome - Compute syndrome S = H * bus_state^T
  bus_state: The 15-bit corrected bus state from Stage 1 (bits 0..14)
  return: The 4-bit syndrome S (equals the data nibble)
  
  This function is IDENTICAL to compute_syndrome_from_bus() in TX side.
  For H1 matrix, column i (1..15) = binary representation of i.
  Syndrome = XOR of column indices for all set bits.
  
  Example:
    bus_state = 0x0234 (bits 2,4,5,9 set)
    columns: 3, 5, 6, 10
    syndrome = 3 ^ 5 ^ 6 ^ 10 = 0xC
 */
uint8_t rx_compute_syndrome(uint16_t bus_state);

/**
  rx_process_bus_state - Process one 15-bit bus state from Stage 1
  bus_state: The corrected 15-bit bus state
  
  This function:
  1. Computes syndrome (extracts data nibble)
  2. Updates state machine (high/low nibble tracking)
  3. On second nibble: assembles complete byte and sends via UART
  
  State machine:
  - State WAIT_HIGH: Store syndrome as high nibble, transition to WAIT_LOW
  - State WAIT_LOW: Combine with stored high nibble, send byte, return to WAIT_HIGH
 */
void rx_process_bus_state(uint16_t bus_state);

/**
  rx_send_uart_byte - Transmit one byte via UART
  data_byte: The byte to transmit
  
  Blocking transmission: waits for TI flag before loading SBUF.
  UART must be initialized before calling this function.
 */
void rx_send_uart_byte(uint8_t data_byte);

/**
  rx_read_bus_state - Read 15-bit bus state from Stage 1 interface
  return: The 15-bit corrected bus state
  
  IMPLEMENTATION REQUIRED: This function must be customized based on
  your Stage 1 hardware interface. Examples:
  
  Example 1: Parallel port interface
    uint16_t state = P0;  // Low byte
    state |= ((uint16_t)(P1 & 0x7F) << 8);  // High 7 bits
    return state;
  
  Example 2: Serial shift-in from Stage 1
    // Shift in 15 bits from serial interface
  
  Example 3: Shared memory or register
    return *((volatile uint16_t*)STAGE1_OUTPUT_REG);
  
  This function should be called when DATA_READY signal is asserted.
 */
uint16_t rx_read_bus_state(void);

/**
  rx_reset_state_machine - Reset decoder to initial state
  
  Call this function to resynchronize the decoder if sync is lost.
  Resets to WAIT_HIGH state and clears stored nibble.
 */
void rx_reset_state_machine(void);

#endif /* RX_HEADER_H */
