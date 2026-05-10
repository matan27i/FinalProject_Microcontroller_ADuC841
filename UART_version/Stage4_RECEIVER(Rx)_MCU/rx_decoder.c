/* 
   File: rx_decoder.c
   Module: Group 2 Decoding Logic

   Dependencies:
     rx_hw.h - rx_send_uart_byte()

   Portability:
     rx_compute_syndrome(), rx_reset_state_machine(), and rx_is_bus_reset()
     contain no hardware calls and can run on any host.
*/

#include "rx_system.h"

/* 
   Module-private state variables
   [N1] volatile removed: no ISRs access these variables.
   [MEM] Placed in XDATA to free DATA space (7 bytes saved).
   
*/

rx_state_t xdata rx_current_state    = RX_STATE_WAIT_HIGH;
uint8_t    xdata rx_stored_high_nibble = 0;
uint8_t    xdata rx_byte_ready       = 0;
uint16_t   xdata rx_bytes_decoded    = 0;
uint16_t   xdata rx_states_processed = 0;

/* Byte-alignment recovery flag.
   ---
   Each uncorrectable PESEC frame drops exactly one nibble.  For the
   nibble FSM to remain aligned to TX byte boundaries, the total number
   of dropped nibbles per recovery window must be EVEN.  When the FSM
   was WAIT_HIGH at the time of an uncorrectable, we dropped a HIGH
   nibble (odd count) -- the next clean frame is its LOW partner and
   must be skipped to restore even parity.  When the FSM was WAIT_LOW,
   resetting the FSM also discards the orphaned stored HIGH, which
   already accounts for two effective drops (the stored HIGH + the
   uncorrectable LOW), so no extra skip is needed.

   Toggle (^=1) rather than set (=1) so consecutive uncorrectables
   self-cancel correctly: e.g. two uncorrectables in WAIT_HIGH drop two
   nibbles total -- skip count goes 0 -> 1 -> 0, no extra skip needed.

   [MEM] XDATA. */
volatile uint8_t xdata rx_skip_next_frame = 0;


/* 
   rx_compute_syndrome
*/
uint8_t rx_compute_syndrome(uint16_t bus_state)
{
    uint8_t  syndrome = 0;
    uint8_t  col_idx;
    uint16_t temp;

    temp = bus_state & BUS_STATE_MASK;

    for (col_idx = 1; col_idx <= HAMMING_N; col_idx++)
    {
        if (temp & 0x0001u)
        {
            syndrome ^= col_idx;
        }
        temp >>= 1;
    }

    return (syndrome & 0x0Fu);
}


/* 
   rx_process_bus_state
   
*/
void rx_process_bus_state(uint16_t bus_state)
{
    uint8_t data_nibble;
    uint8_t complete_byte;

    /* Alignment recovery: if a previous PESEC_UNCORRECTABLE event left
       us off by one nibble, drop this frame to restore even parity with
       the TX byte boundary.  See rx_skip_next_frame in this file. */
    if (rx_skip_next_frame)
    {
        rx_skip_next_frame = 0;
        return;
    }

    data_nibble = rx_compute_syndrome(bus_state);

    switch (rx_current_state)
    {
        case RX_STATE_WAIT_HIGH:
            rx_stored_high_nibble = data_nibble & 0x0Fu;
            rx_current_state      = RX_STATE_WAIT_LOW;
            break;

        case RX_STATE_WAIT_LOW:
            complete_byte = (uint8_t)((rx_stored_high_nibble << 4)
                             | (data_nibble & 0x0Fu));

            rx_send_uart_byte(complete_byte);

            rx_bytes_decoded++;
            rx_byte_ready         = 1;
            rx_current_state      = RX_STATE_WAIT_HIGH;
            rx_stored_high_nibble = 0;
            break;

        default:
            rx_reset_state_machine();
            break;
    }
}


/* 
   rx_reset_state_machine
   
*/
void rx_reset_state_machine(void)
{
    rx_current_state      = RX_STATE_WAIT_HIGH;
    rx_stored_high_nibble = 0;
    rx_byte_ready         = 0;
    /* rx_skip_next_frame is intentionally NOT cleared here -- alignment
       recovery is decoupled from FSM reset.  A bus-reset frame ('=' from
       the host) clears it explicitly via rx_perform_full_reset. */
}


/* 
   rx_is_bus_reset
   
*/
uint8_t rx_is_bus_reset(uint16_t data_bits, uint16_t red_bits)
{
    if ((data_bits == 0) && (red_bits == 0))
    {
        return 1;
    }
    return 0;
}
