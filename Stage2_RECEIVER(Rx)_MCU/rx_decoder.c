/* =============================================================================
   File: rx_decoder.c
   Module: Group 2 — Decoding Logic
   Implementation of syndrome computation, nibble FSM, and state reset.

   Dependencies:
     rx_hw.h  ->  rx_send_uart_byte()  (UART output, hardware module)

   Portability:
     rx_compute_syndrome() and rx_reset_state_machine() contain no
     hardware calls and can run on any host.
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

/* Current position in the nibble-reassembly FSM. */
volatile rx_state_t rx_current_state    = RX_STATE_WAIT_HIGH;

/* High nibble stored between the WAIT_HIGH and WAIT_LOW calls. */
volatile uint8_t    rx_stored_high_nibble = 0;

/* Pulsed to 1 each time a complete byte is dispatched via UART. */
volatile uint8_t    rx_byte_ready       = 0;

/* Cumulative bytes successfully decoded and transmitted. */
volatile uint16_t   rx_bytes_decoded    = 0;

/* Cumulative bus states processed (incremented by rx_main.c). */
volatile uint16_t   rx_states_processed = 0;


/* ---------------------------------------------------------------------------
   rx_compute_syndrome
   ---------------------------------------------------------------------------
   Computes S = H * bus_state^T on-the-fly using bitwise XOR.

   Algorithm:
     Iterate over bit positions j = 0 .. (HAMMING_N - 1).
     For each bit j that is set in bus_state, XOR the syndrome
     accumulator with the column index (j + 1).
     No matrix storage, no lookup table, O(N) time on 8/16-bit hardware.

   Equivalence to matrix multiplication:
     The H1-type matrix stores the value (j+1) in column (j+1).
     Therefore  H * e_j^T  =  (j+1),  where e_j is the unit vector with
     only bit j set.  By linearity of XOR over GF(2):
         H * bus_state^T  =  XOR{ (j+1) : bit j set }
   ---------------------------------------------------------------------------
*/
uint8_t rx_compute_syndrome(uint16_t bus_state)
{
    uint8_t  syndrome = 0;
    uint8_t  col_idx;
    uint16_t temp;

    temp = bus_state & BUS_STATE_MASK;   /* Work only on bits 0..14 */

    for (col_idx = 1; col_idx <= HAMMING_N; col_idx++)
    {
        if (temp & 0x0001u)
        {
            /* Bit (col_idx - 1) is set; XOR in its column index.
               The column index IS the column value in the H1 matrix,
               so no table lookup is needed. */
            syndrome ^= col_idx;
        }
        temp >>= 1;
    }

    return (syndrome & 0x0Fu);   /* Return the 4-bit result */
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

            rx_send_uart_byte(complete_byte);   /* Transmit to PC */

            rx_bytes_decoded++;
            rx_byte_ready         = 1;          /* Signal completion */
            rx_current_state      = RX_STATE_WAIT_HIGH;
            rx_stored_high_nibble = 0;           /* Clear for safety */
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
