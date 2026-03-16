/* =============================================================================
   File: rx_main.c
   Module: Group 1 -- Main entry point and pipeline loop (SPI Slave version)

   MODIFICATION SUMMARY:
     The original design polled a DATA_READY GPIO (P3.2) for rising edges
     and called rx_read_full_bus() to capture 25 parallel bits.  This
     version replaces that input mechanism with SPI slave reception:

       Old path:  DATA_READY edge -> rx_read_full_bus() -> pipeline
       New path:  SPI ISR fills rx_spi_data/rx_spi_red -> pipeline

     The five-stage decode pipeline (A through E) is preserved exactly.
     Stages B-E are completely unchanged.  Stage A is replaced by the
     SPI ISR in rx_hw.c; the main loop simply checks rx_spi_frame_ready.

   Responsibilities:
     - Hardware startup sequence (clock, XRAM, UART, SPI slave).
     - PESEC matrix initialisation (must match TX config).
     - Checking the SPI frame-ready flag each iteration.
     - Sequencing the full decode pipeline per received frame:
         (Stage A: handled by SPI ISR -- see rx_hw.c)
         Stage B: rx_is_bus_reset()        (decoder -- reset detect)
         Stage C: rx_pesec_correct()       (ecc -- 25-bit PESEC)
         Stage D: rx_correct_bus_state()   (ecc -- H1 differential)
         Stage E: rx_process_bus_state()   (decoder -- nibble FSM)

   This file owns no domain logic.  It is intentionally thin so that
   the three functional modules can be tested independently.
   =============================================================================
*/

#include "rx_system.h"

/* ---------------------------------------------------------------------------
   PESEC block configuration -- MUST MATCH THE TX ENCODER.

   The TX uses:  static uint8_t pesec_config[] = {3, 2};
   This produces:
     Block 0 (m=3): 7 redundancy columns, syndrome bits [2:0]
     Block 1 (m=2): 3 redundancy columns, syndrome bits [4:3]
     Total: 10 redundancy bits, 5-bit syndrome
   ---------------------------------------------------------------------------
*/
static uint8_t pesec_config[] = {3, 2};


/* ---------------------------------------------------------------------------
   rx_perform_full_reset
   ---------------------------------------------------------------------------
   Resets all receiver state to match the TX post-reset condition.
   Called when an all-zero 25-bit word is detected, indicating the TX
   has executed its '=' reset command.

   After TX reset:
     current_bus_state    = 0
     pesec_redundancy_reg = 0
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
    uint16_t corrected_data;

    /* ------------------------------------------------------------------
       [C1] Set core clock to full crystal speed FIRST.
       PLLCON default = 0x53 -> CD=3 -> fCORE = 11.0592/8 = 1.38 MHz.
       Timer 3 baud config and SPI slave timing require full speed.
       On ADuC841, reserved bits must be written as 0 (datasheet p. 49).
       ------------------------------------------------------------------
    */
    PLLCON = 0x00;  /* CD=0: fCORE = 11.0592 MHz */

    /* Enable internal 2 KB XRAM mapped to bottom of external data space.
       Required because tx_buf in rx_hw.c is declared xdata.
       Use ORL (|=) to preserve the auto-configured EPM bits (p. 45). */
    CFG841 |= 0x01;  /* XRAMEN = 1 */

    /* ------------------------------------------------------------------
       Hardware initialisation -- peripherals BEFORE interrupts.
       Order: Timer 3 (baud clock) -> UART -> SPI Slave + INT0.
       ------------------------------------------------------------------
    */
    RX_Timer3_Init();
    RX_UART_Init();
    RX_SPI_Slave_Init();

    /* ------------------------------------------------------------------
       PESEC matrix initialisation.
       Uses the same block configuration as the TX encoder: {3, 2}.
       ------------------------------------------------------------------
    */
    rx_init_pesec_matrices(pesec_config, 2);

    /* ------------------------------------------------------------------
       Decoder and error-correction module reset.
       All baselines start at 0 to match the TX encoder's initial state.
       ------------------------------------------------------------------
    */
    rx_reset_state_machine();
    rx_previous_bus_state = 0;
    rx_errors_corrected   = 0;
    rx_errors_detected    = 0;
    rx_pesec_corrections  = 0;

    /* ------------------------------------------------------------------
       Enable global interrupts LAST, after all peripherals are ready.
       This enables: INT0 (frame sync) and SPI interrupt (vector 6).
       ------------------------------------------------------------------
    */
    EA = 1;

    /* ------------------------------------------------------------------
       Main pipeline loop
       ------------------------------------------------------------------
       The loop structure is deliberately simple:
         1. Pump the UART TX buffer (non-blocking).
         2. Check if the SPI ISR has delivered a new 25-bit frame.
         3. If yes, run the decode pipeline (stages B through E).

       The SPI ISR handles all timing-critical reception.  The main loop
       only needs to process frames faster than they arrive, which is
       easily satisfied: the TX produces one frame every ~280 us, and
       the decode pipeline (syndrome + delta solve + nibble FSM) takes
       well under 100 us on the 8051 at 11 MHz.
       ------------------------------------------------------------------
    */
    while (1)
    {
        /* Pump the non-blocking UART TX buffer each iteration. */
        rx_uart_pump();

        /* Check for a new SPI frame. */
        if (rx_spi_frame_ready)
        {
            /* --- Consume the SPI frame --- */
            /* Disable interrupts briefly to take an atomic snapshot
               of the two 16-bit staging registers, then clear the flag.
               This prevents a new ISR write from corrupting the read. */
            EA = 0;
            raw_data = rx_spi_data;
            raw_red  = rx_spi_red;
            rx_spi_frame_ready = 0;
            EA = 1;

            /* Stage B: TX reset detection.
               All-zero 25-bit word means the TX executed '='. */
            if (rx_is_bus_reset(raw_data, raw_red))
            {
                rx_perform_full_reset();
                continue;
            }

            /* Stage C: PESEC error correction (25-bit codeword).
               raw_data and raw_red are corrected IN PLACE. */
            rx_pesec_correct(&raw_data, &raw_red);

            /* Stage D: H1 differential validation (15-bit data). */
            corrected_data = rx_correct_bus_state(raw_data);

            /* Stage E: Nibble extraction and byte reassembly. */
            rx_process_bus_state(corrected_data);

            rx_states_processed++;
        }
    }
}
