/* =============================================================================
   File: rx_decoder.h
   Module: Group 2 -- Decoding Logic
   Public interface, FSM type, and extern declarations.

   This module handles everything between receiving a corrected 15-bit bus
   state and emitting a reconstructed byte:
     1. Syndrome computation (H1 matrix multiply -> data nibble extraction)
     2. Nibble-pair reassembly finite-state machine (HIGH + LOW -> byte)
     3. FSM reset / resynchronisation
     4. TX reset detection (all-zero 25-bit word)

   Dependency on hardware:
     rx_process_bus_state() calls rx_send_uart_byte() (declared in rx_hw.h)
     to emit the assembled byte.  All other functions in this module are
     pure arithmetic with no hardware side-effects.
   =============================================================================
*/

#ifndef RX_DECODER_H
#define RX_DECODER_H

#include "rx_types.h"

/* ---------------------------------------------------------------------------
   Nibble-reassembly FSM states

   The TX side sends one nibble per bus state in alternating order:
     WAIT_HIGH -> receive bits [7:4] of the next byte
     WAIT_LOW  -> receive bits [3:0], combine, emit byte, return to WAIT_HIGH
   ---------------------------------------------------------------------------
*/
typedef enum
{
    RX_STATE_WAIT_HIGH = 0,   /* Expecting the high nibble (bits 7-4) */
    RX_STATE_WAIT_LOW  = 1    /* Expecting the low  nibble (bits 3-0) */
} rx_state_t;

/* ---------------------------------------------------------------------------
   Module state variables -- defined in rx_decoder.c.

   rx_current_state
     Current position in the nibble-reassembly FSM.

   rx_stored_high_nibble
     Holds the high nibble captured in WAIT_HIGH until the matching
     low nibble arrives in the next WAIT_LOW cycle.

   rx_byte_ready
     Pulsed to 1 each time a complete byte is sent to the PC via UART.
     The consumer must clear it.

   rx_bytes_decoded
     Cumulative count of bytes successfully assembled and transmitted.

   rx_states_processed
     Cumulative count of corrected bus states handed to the decoder.
     Incremented by rx_main.c each time rx_process_bus_state() is called.
   ---------------------------------------------------------------------------
*/
/* [N1] volatile removed: no ISRs access these; main-loop only. */
extern rx_state_t rx_current_state;
extern uint8_t    rx_stored_high_nibble;
extern uint8_t    rx_byte_ready;
extern uint16_t   rx_bytes_decoded;
extern uint16_t   rx_states_processed;

/* ---------------------------------------------------------------------------
   rx_compute_syndrome
   ---------------------------------------------------------------------------
   Computes  S = H * bus_state^T  using the H1-type matrix identity:
     Column i (1-indexed) = binary value i.
     S = XOR{ (j + 1) : bit j of bus_state is 1,  j = 0..14 }

   The returned 4-bit syndrome equals the data nibble that the TX encoder
   intended for this bus state.

   Parameters:
     bus_state   15-bit corrected bus state (bits 0..14).
   Returns:
     4-bit syndrome / data nibble (upper nibble is zero).
   ---------------------------------------------------------------------------
*/
uint8_t rx_compute_syndrome(uint16_t bus_state);

/* ---------------------------------------------------------------------------
   rx_process_bus_state
   ---------------------------------------------------------------------------
   Core decode entry point.  Accepts one corrected 15-bit bus state,
   extracts its data nibble, and drives the reassembly FSM:

     WAIT_HIGH:  store nibble as bits [7:4], transition to WAIT_LOW.
     WAIT_LOW:   combine with stored high nibble, transmit byte via UART,
                 increment rx_bytes_decoded, return to WAIT_HIGH.

   Parameters:
     bus_state   15-bit corrected bus state from the ECC pipeline.
   ---------------------------------------------------------------------------
*/
void rx_process_bus_state(uint16_t bus_state);

/* ---------------------------------------------------------------------------
   rx_reset_state_machine
   ---------------------------------------------------------------------------
   Resets the nibble-reassembly FSM to its initial WAIT_HIGH state.

   When to call:
     - At system startup (before the first bus state arrives).
     - After detecting a TX reset (all-zero bus).
     - After detecting a synchronisation loss (garbled output).
   ---------------------------------------------------------------------------
*/
void rx_reset_state_machine(void);

/* ---------------------------------------------------------------------------
   rx_is_bus_reset
   ---------------------------------------------------------------------------
   Detects a TX hard-reset condition by checking whether the received
   25-bit word (15 data + 10 redundancy) is all zeros.

   TX RESET PROTOCOL:
     When the TX receives the '=' character, it executes:
       1. current_bus_state = 0
       2. pesec_redundancy_reg = 0
       3. Hardware clear of all shift registers (forces outputs to 0)
     This produces a 25-bit all-zero word on the bus, which is NOT a
     valid data encoding (the encoder never produces all-zeros as a
     normal differential update from a non-zero state with non-zero
     redundancy).

   Detection logic:
     If BOTH data_bits AND red_bits are zero, return 1 (reset detected).
     Otherwise return 0 (normal data).

   Why this is reliable:
     During normal operation, the PESEC redundancy register tracks the
     data state differentially.  For the redundancy to be zero, the
     syndrome of all data bits through Matrix A must also be zero,
     which only happens when the data is in a very specific subset of
     states.  Combined with the data being exactly zero, this condition
     is uniquely associated with the TX reset command.

   Parameters:
     data_bits   15-bit data portion of the received word.
     red_bits    10-bit redundancy portion of the received word.
   Returns:
     1 if reset detected, 0 otherwise.
   ---------------------------------------------------------------------------
*/
uint8_t rx_is_bus_reset(uint16_t data_bits, uint16_t red_bits);

#endif /* RX_DECODER_H */
