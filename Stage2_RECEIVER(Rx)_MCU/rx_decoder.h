/**
 * rx_decoder.h
 * 
 * Stage 2 Decoder & Reassembler for H1-Type Stateful Bus Encoder System
 * Target: ADuC841 (8052 core)
 * 
 * Description:
 * Decodes 15-bit bus states back into 8-bit ASCII characters by:
 * 1. Computing syndrome (which equals the data nibble)
 * 2. Reassembling high and low nibbles into complete bytes
 * 3. Transmitting via UART
 */

#ifndef RX_DECODER_H
#define RX_DECODER_H

#include <stdint.h>

/**
 * Compute syndrome of 15-bit bus state
 * 
 * For H1-type encoding, the syndrome equals the data nibble.
 * Syndrome = XOR of column indices (1-based) of all set bits
 * 
 * @param bus_state: 15-bit corrected bus state (bits 0-14 valid)
 * @return: 4-bit data nibble (0x0 - 0xF)
 */
uint8_t compute_syndrome(uint16_t bus_state);

/**
 * Process incoming bus state and reassemble bytes
 * 
 * State Machine:
 * - Cycle 1: Extract high nibble, store internally
 * - Cycle 2: Extract low nibble, combine with high nibble, send via UART
 * 
 * @param current_bus_state: 15-bit corrected bus state from Stage 1
 */
void rx_process_state(uint16_t current_bus_state);

/**
 * Transmit a byte via UART
 * 
 * Sends the reconstructed ASCII character back to PC
 * Note: UART must be initialized before calling this function
 * 
 * @param c: Character/byte to transmit
 */
void UART_Tx(char c);

/**
 * Initialize the decoder state machine
 * 
 * Call this once at system startup before processing any bus states
 */
void rx_decoder_init(void);

#endif /* RX_DECODER_H */
