/* =============================================================================
   File: rx_main.c
   Module: Group 1 -- Main entry point and pipeline loop (SPI Slave version)

   MODIFICATION SUMMARY:
     [W4] The main loop now snapshots rx_spi_parity alongside data and
     redundancy, and passes it to rx_pesec_correct() for SEC+DED decoding.
     The reset detection (rx_is_bus_reset) is unchanged: all 26 bits
     (data + red + parity) are zero after a TX '=' reset.

   Pipeline stages:
     (Stage A: handled by SPI ISR -- see rx_hw.c)
     Stage B: rx_is_bus_reset()        (decoder -- reset detect)
     Stage C: rx_pesec_correct()       (ecc -- 26-bit SEC+DED)
     Stage D: rx_correct_bus_state()   (ecc -- H1 differential)
     Stage E: rx_process_bus_state()   (decoder -- nibble FSM)
   =============================================================================
*/

#include "rx_system.h"

/* ---------------------------------------------------------------------------
   PESEC block configuration -- MUST MATCH THE TX ENCODER.
   ---------------------------------------------------------------------------
*/
static uint8_t pesec_config[] = {3, 2};


/* ---------------------------------------------------------------------------
   rx_perform_full_reset
   ---------------------------------------------------------------------------
*/
static void rx_perform_full_reset(void)
{
    rx_previous_bus_state = 0;

    rx_errors_corrected  = 0;
    rx_errors_detected   = 0;
    rx_pesec_corrections = 0;

    rx_reset_state_machine();
}


/* ---------------------------------------------------------------------------
   main
   ---------------------------------------------------------------------------
*/
void main(void)
{
    uint16_t raw_data;
    uint16_t raw_red;
    uint8_t  raw_parity;        /* [W4] Overall parity from 26th bit */
    uint16_t corrected_data;

    /* Set core clock to full crystal speed. */
    PLLCON = 0x00;

    /* Enable internal 2 KB XRAM. */
    CFG841 |= 0x01;

    /* Hardware initialisation. */
    RX_Timer3_Init();
    RX_UART_Init();
    RX_SPI_Slave_Init();

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

    /* Main pipeline loop. */
    while (1)
    {
        rx_uart_pump();

        if (rx_spi_frame_ready)
        {
            /* Atomically snapshot the SPI staging registers.
               [W4] Now also captures the overall parity bit. */
            EA = 0;
            raw_data   = rx_spi_data;
            raw_red    = rx_spi_red;
            raw_parity = rx_spi_parity;    /* [W4] */
            rx_spi_frame_ready = 0;
            EA = 1;

            /* Stage B: TX reset detection.
               After TX '=' reset, all 26 bits are zero.  The parity
               of an all-zero 25-bit word is 0, so the overall parity
               bit is also 0.  The existing rx_is_bus_reset() check
               (data==0 && red==0) remains correct. */
            if (rx_is_bus_reset(raw_data, raw_red))
            {
                rx_perform_full_reset();
                continue;
            }

            /* Stage C: [W4] SEC+DED PESEC error correction.
               Now passes the overall parity bit so the decoder can
               distinguish single errors (correctable) from double
               errors (detectable but uncorrectable). */
            rx_pesec_correct(&raw_data, &raw_red, raw_parity);

            /* Stage D: H1 differential validation. */
            corrected_data = rx_correct_bus_state(raw_data);

            /* Stage E: Nibble extraction and byte reassembly. */
            rx_process_bus_state(corrected_data);

            rx_states_processed++;
        }
    }
}
