/* 
   File: rx_decoder.h
   Module: Group 2 
   Public interface, FSM type, and extern declarations.
*/

#ifndef RX_DECODER_H
#define RX_DECODER_H

#include "rx_types.h"

/* 
   Nibble-reassembly FSM states
*/
typedef enum
{
    RX_STATE_WAIT_HIGH = 0,
    RX_STATE_WAIT_LOW  = 1
} rx_state_t;

/* 
   Module state variables -- defined in rx_decoder.c.
   [N1] volatile removed: no ISRs access these; main-loop only.
   [MEM] XDATA: saves 7 bytes of DATA.
   
*/
extern rx_state_t xdata rx_current_state;
extern uint8_t    xdata rx_stored_high_nibble;
extern uint8_t    xdata rx_byte_ready;
extern uint16_t   xdata rx_bytes_decoded;
extern uint16_t   xdata rx_states_processed;

/* Byte-alignment recovery: toggled by rx_main on PESEC_UNCORRECTABLE
   when the FSM was at WAIT_HIGH (odd-parity drop), consumed by
   rx_process_bus_state to drop the next clean frame and restore even
   nibble parity with the TX side.  See rx_decoder.c for full rationale. */
extern volatile uint8_t xdata rx_skip_next_frame;

/* 
   Prototypes
   
*/
uint8_t rx_compute_syndrome(uint16_t bus_state);
void rx_process_bus_state(uint16_t bus_state);
void rx_reset_state_machine(void);
uint8_t rx_is_bus_reset(uint16_t data_bits, uint16_t red_bits);

#endif 
