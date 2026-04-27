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
