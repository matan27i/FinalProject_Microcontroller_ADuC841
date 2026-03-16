/* =============================================================================
   File: rx_decoder.c
   Module: Group 2 -- Decoding Logic
   Implementation of syndrome computation, nibble FSM, state reset,
   and TX reset detection.

   Dependencies:
     rx_hw.h  ->  rx_send_uart_byte()  (UART output, hardware module)

   Portability:
     rx_compute_syndrome(), rx_reset_state_machine(), and rx_is_bus_reset()
     contain no hardware calls and can run on any host.
     rx_process_bus_state() calls rx_send_uart_byte() once per assembled
     byte; substitute any byte-output routine when porting.
   =============================================================================
*/

#include "rx_system.h"

/* ---------------------------------------------------------------------------
   Module-private state variables
   Defined here; declared extern in rx_decoder.h.
   ---------------------------------------------------------------------------
*/

/* [N1] volatile removed: no ISRs access these variables.
   All access is from the main-loop call chain only.
   [MEM] Placed in XDATA to free DATA space (7 bytes saved). */

/* Current position in the nibble-reassembly FSM. */
rx_state_t xdata rx_current_state    = RX_STATE_WAIT_HIGH;

/* High nibble stored between the WAIT_HIGH and WAIT_LOW calls. */
uint8_t    xdata rx_stored_high_nibble = 0;

/* Pulsed to 1 each time a complete byte is dispatched via UART. */
uint8_t    xdata rx_byte_ready       = 0;

/* Cumulative bytes successfully decoded and transmitted. */
uint16_t   xdata rx_bytes_decoded    = 0;

/* Cumulative bus states processed (incremented by rx_main.c). */
uint16_t   xdata rx_states_processed = 0;


/* ---------------------------------------------------------------------------
   rx_compute_syndrome
   ---------------------------------------------------------------------------
   Computes S = H * bus_state^T on-the-fly using bitwise XOR.

   Algorithm:
     Iterate over bit positions j = 0 .. (HAMMING_N - 1).
     For each bit j that is set in bus_state, XOR the syndrome
     accumulator with the column index (j + 1).

   Equivalence to matrix multiplication:
     The H1-type matrix stores the value (j+1) in column (j+1).
     By linearity of XOR over GF(2):
         H * bus_state^T  =  XOR{ (j+1) : bit j set }
   ---------------------------------------------------------------------------
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


/* ---------------------------------------------------------------------------
   rx_process_bus_state
   ---------------------------------------------------------------------------
   Nibble-reassembly FSM.  See rx_decoder.h for the full state description.
   ---------------------------------------------------------------------------
*/
void rx_process_bus_state(uint16_t bus_state)
{
    uint8_t data_nibble;
    uint8_t complete_byte;

    /* Extract the 4-bit data nibble encoded in this bus state. */
    data_nibble = rx_compute_syndrome(bus_state);

    switch (rx_current_state)
    {
        case RX_STATE_WAIT_HIGH:
            /*
               First nibble of a new byte pair.
               Store as the high nibble (bits [7:4]) and wait for the
               low nibble in the next bus state.
            */
            rx_stored_high_nibble = data_nibble & 0x0Fu;
            rx_current_state      = RX_STATE_WAIT_LOW;
            break;

        case RX_STATE_WAIT_LOW:
            /*
               Second nibble of the pair.
               Combine with the stored high nibble to form a full byte.

               Byte layout:
                 bits [7:4]  =  rx_stored_high_nibble  (arrived first)
                 bits [3:0]  =  data_nibble             (arrives now)

               Example:
                 high = 0x4, low = 0x1  ->  byte = 0x41 = 'A'
            */
            complete_byte = (uint8_t)((rx_stored_high_nibble << 4)
                             | (data_nibble & 0x0Fu));

            rx_send_uart_byte(complete_byte);

            rx_bytes_decoded++;
            rx_byte_ready         = 1;
            rx_current_state      = RX_STATE_WAIT_HIGH;
            rx_stored_high_nibble = 0;
            break;

        default:
            /* Should never reach here; reset to a safe, known state. */
            rx_reset_state_machine();
            break;
    }
}


/* ---------------------------------------------------------------------------
   rx_reset_state_machine
   ---------------------------------------------------------------------------
*/
void rx_reset_state_machine(void)
{
    rx_current_state      = RX_STATE_WAIT_HIGH;
    rx_stored_high_nibble = 0;
    rx_byte_ready         = 0;
}


/* ---------------------------------------------------------------------------
   rx_is_bus_reset
   ---------------------------------------------------------------------------
   Detects the TX hard-reset condition (all 25 bits zero).

   The TX '=' command clears both current_bus_state and pesec_redundancy_reg
   to zero, then hardware-clears the shift registers.  This produces an
   all-zero 25-bit word on the physical bus.

   Detection: both data and redundancy must be exactly zero.

   NOTE: At system startup the bus is also zero.  The main loop uses
   additional context (e.g., whether any states have been processed) to
   distinguish cold-start from mid-session reset if needed.  This
   function is a pure predicate; it does not perform any state changes.
   ---------------------------------------------------------------------------
*/
uint8_t rx_is_bus_reset(uint16_t data_bits, uint16_t red_bits)
{
    if ((data_bits == 0) && (red_bits == 0))
    {
        return 1;
    }
    return 0;
}
