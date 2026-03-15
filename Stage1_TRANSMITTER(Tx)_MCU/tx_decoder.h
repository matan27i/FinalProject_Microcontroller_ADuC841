/* File: decoder.h
 * Group 2: H1-Type Bus Encoding & UART Character Handling
 * ========================================================
 * Declares the stateful bus encoder state and prototypes for:
 *   - Syndrome computation from bus state
 *   - Minimal-weight codeword search
 *   - Nibble-level stateful encoding
 *   - UART character reception and splitting
 *
 * Portability: This module is pure algorithmic logic with no
 *              hardware dependencies.  It calls into Group 1
 *              for shift-register output and Group 3 for ECC.
 */

#ifndef TX_DECODER_H
#define TX_DECODER_H

/* ---- Stateful Bus State Vector ---- */
/* 15-bit physical bus state x.  H * x^T = current syndrome.
 * Initialised to 0 (zero bus, zero syndrome).
 * [N2] Not volatile: only accessed from main-loop call chain. */
extern uint16_t current_bus_state;

/* ---- Core Encoder Prototypes ---- */
uint8_t  compute_syndrome_from_bus(uint16_t bus_state);
uint16_t find_minimal_w(uint8_t s_target);
void     process_nibble(uint8_t s_new);

/* ---- UART Character Handler ---- */
void tx_handler(uint8_t rx_char);

#endif /* TX_DECODER_H */
