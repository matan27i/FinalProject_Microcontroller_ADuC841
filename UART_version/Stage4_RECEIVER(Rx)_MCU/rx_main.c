/*
   File: rx_main.c
   Module: Group 1 Main entry point and pipeline loop (UART Frame version)

   INGRESS PATH REFACTOR:
     The ingress path was changed from SPI slave to framed UART.  The
     main pipeline is otherwise identical: the UART ISR now populates
     rx_frame_data / rx_frame_red / rx_frame_parity exactly as the SPI
     ISR did, and pulses rx_frame_ready when a CRC-valid packet has
     arrived.  rx_is_bus_reset() still detects the TX '=' reset by the
     all-zero 26-bit bus condition.

   Pipeline stages:
     (Stage A: handled by UART ISR -- see rx_hw.c)
     Stage B: rx_is_bus_reset()        (decoder -- reset detect)
     Stage C: rx_pesec_correct()       (ecc -- 26-bit SEC+DED)
     Stage D: rx_correct_bus_state()   (ecc -- H1 differential)
     Stage E: rx_process_bus_state()   (decoder -- nibble FSM)
*/

#include "rx_system.h"

/*
   PESEC block configuration -- MUST MATCH THE TX ENCODER.
*/
static uint8_t pesec_config[] = {3, 2};


/*
   rx_perform_full_reset
*/
static void rx_perform_full_reset(void)
{
    rx_previous_bus_state = 0;

    rx_errors_corrected  = 0;
    rx_errors_detected   = 0;
    rx_pesec_corrections = 0;

    rx_reset_state_machine();
}


/*
   main
*/
void main(void)
{
    uint16_t raw_data;
    uint16_t raw_red;
    uint8_t  raw_parity;
    uint16_t corrected_data;

    /* Set core clock to full crystal speed. */
    PLLCON = 0x00;

    /* Enable internal 2 KB XRAM. */
    CFG841 |= 0x01;

    /* Hardware initialisation. */
    RX_Timer3_Init();
    RX_UART_Init();

    /* PESEC matrix initialisation. */
    rx_init_pesec_matrices(pesec_config, 2);

    /* Decoder and error-correction module reset. */
    rx_reset_state_machine();
    rx_previous_bus_state = 0;
    rx_errors_corrected   = 0;
    rx_errors_detected    = 0;
    rx_pesec_corrections  = 0;

    /* Enable global interrupts LAST. */
    EA = 1;

    /* Boot-message handshake.  Seeing "RX-READY\r\n" on PuTTY confirms
       the UART TX path (SCON mode + Timer 3 baud + TxD pin) is working,
       independently of any bridge input or PESEC/H1 logic. */
    rx_send_uart_byte('R');
    rx_send_uart_byte('X');
    rx_send_uart_byte('-');
    rx_send_uart_byte('R');
    rx_send_uart_byte('E');
    rx_send_uart_byte('A');
    rx_send_uart_byte('D');
    rx_send_uart_byte('Y');
    rx_send_uart_byte('\r');
    rx_send_uart_byte('\n');

    /* Main pipeline loop. */
    while (1)
    {
        if (rx_frame_ready)
        {
            uint8_t pesec_result;

            /* Atomically snapshot the frame staging registers. */
            EA = 0;
            raw_data       = rx_frame_data;
            raw_red        = rx_frame_red;
            raw_parity     = rx_frame_parity;
            rx_frame_ready = 0;
            EA = 1;

#if RX_DEBUG
            /* 'F' = a CRC-valid 4-byte payload was handed to the main
               loop.  Seeing 'F' but no decoded byte means the frame is
               arriving cleanly but PESEC or H1 is rejecting it. */
            rx_send_uart_byte('F');
#endif

            /* Stage B: TX reset detection. */
            if (rx_is_bus_reset(raw_data, raw_red))
            {
#if RX_DEBUG
                rx_send_uart_byte('R');
#endif
                rx_perform_full_reset();
                continue;
            }

            /* Stage C: SEC+DED PESEC error correction. */
            pesec_result = rx_pesec_correct(&raw_data, &raw_red, raw_parity);
            if (pesec_result == PESEC_UNCORRECTABLE)
            {
#if RX_DEBUG
                rx_send_uart_byte('U');
#endif
                continue;
            }
#if RX_DEBUG
            if (pesec_result == PESEC_CORRECTED_DATA)   rx_send_uart_byte('d');
            if (pesec_result == PESEC_CORRECTED_RED)    rx_send_uart_byte('r');
            if (pesec_result == PESEC_CORRECTED_PARITY) rx_send_uart_byte('p');
#endif

            /* Stage D: H1 differential validation. */
            corrected_data = rx_correct_bus_state(raw_data);

            /* Stage E: Nibble extraction and byte reassembly. */
            rx_process_bus_state(corrected_data);

            rx_states_processed++;
        }
    }
}
