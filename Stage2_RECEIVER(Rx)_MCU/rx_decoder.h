/* =============================================================================
   File: rx_decoder.h
   Module: Group 2 — Decoding Logic
   Public interface, FSM type, and extern declarations.

   This module handles everything between receiving a corrected 15-bit bus
   state and emitting a reconstructed byte:
     1. Syndrome computation (H1 matrix multiply -> data nibble extraction)
     2. Nibble-pair reassembly finite-state machine (HIGH + LOW -> byte)
     3. FSM reset / resynchronisation

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

   The enum is declared here (not in rx_types.h) because it is an
   implementation detail of the decoder module alone.
   ---------------------------------------------------------------------------
*/
typedef enum
{
    RX_STATE_WAIT_HIGH = 0,   /* Expecting the high nibble (bits 7-4) */
    RX_STATE_WAIT_LOW  = 1    /* Expecting the low  nibble (bits 3-0) */
} rx_state_t;

/* ---------------------------------------------------------------------------
   Module state variables — defined in rx_decoder.c.

   rx_current_state
     Current position in the nibble-reassembly FSM.

   rx_stored_high_nibble
     Holds the high nibble captured in WAIT_HIGH until the matching
     low nibble arrives in the next WAIT_LOW cycle.

   rx_byte_ready
     Pulsed to 1 each time a complete byte is sent to the PC via UART.
     The main loop or an ISR may poll this flag for flow-control purposes.
     The flag is NOT automatically cleared; the consumer must clear it.

   rx_bytes_decoded
     Cumulative count of bytes successfully assembled and transmitted.

   rx_states_processed
     Cumulative count of corrected bus states handed to the decoder.
     Incremented by rx_main.c each time rx_process_bus_state() is called.
   ---------------------------------------------------------------------------
*/
extern volatile rx_state_t rx_current_state;
extern volatile uint8_t    rx_stored_high_nibble;
extern volatile uint8_t    rx_byte_ready;
extern volatile uint16_t   rx_bytes_decoded;
extern volatile uint16_t   rx_states_processed;

/* ---------------------------------------------------------------------------
   rx_compute_syndrome
   ---------------------------------------------------------------------------
   Computes  S = H * bus_state^T  entirely in hardware registers, with no
   lookup tables and no stored copy of the 4x15 matrix.

   H1-type matrix identity used:
     Column i (1-indexed, i = 1 .. 15) of H equals the 4-bit binary
     representation of i.  Therefore the syndrome is simply the XOR of all
     column indices whose corresponding bus bit is set:

         S = XOR{ (j + 1) : bit j of bus_state is 1,  j = 0..14 }

   Example (from the decoder spec):
     bus_state = 0x0015  (bits 0, 2, 4 set)
     column indices contributed: 1, 3, 5
     S = 1 XOR 3 XOR 5 = 0b0111 = 0x7

   The returned 4-bit syndrome equals the data nibble that the TX encoder
   intended for this bus state.

   Parameters:
     bus_state   15-bit corrected bus state (bits 0..14).
   Returns:
     4-bit syndrome / data nibble (upper nibble of return value is zero).
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

   Timing requirement:
     Must be called at the same rate the TX encoder emits bus states.

   Resynchronisation:
     If the byte stream becomes corrupt (e.g. missed state), call
     rx_reset_state_machine() to force a fresh WAIT_HIGH alignment.

   Parameters:
     bus_state   15-bit corrected bus state from rx_correct_bus_state().
   ---------------------------------------------------------------------------
*/
void rx_process_bus_state(uint16_t bus_state);

/* ---------------------------------------------------------------------------
   rx_reset_state_machine
   ---------------------------------------------------------------------------
   Resets the nibble-reassembly FSM to its initial WAIT_HIGH state.

   When to call:
     - At system startup (before the first bus state arrives).
     - After detecting a synchronisation loss (garbled output).
     - After a timeout when an expected bus state does not arrive.
   ---------------------------------------------------------------------------
*/
void rx_reset_state_machine(void);

#endif /* RX_DECODER_H */
