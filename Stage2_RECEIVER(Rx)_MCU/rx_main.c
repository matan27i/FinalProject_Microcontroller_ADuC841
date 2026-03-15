/* =============================================================================
   File: rx_main.c
   Module: Group 1 -- Main entry point and pipeline loop

   Responsibilities:
     - Hardware startup sequence.
     - PESEC matrix initialisation (must match TX config).
     - Rising-edge detection on DATA_READY.
     - Sequencing the full decode pipeline per bus state:
         Stage A: rx_read_full_bus()         (hw module -- read 25 bits)
         Stage B: rx_is_bus_reset()          (decoder module -- reset detect)
         Stage C: rx_pesec_correct()         (ecc module -- 25-bit PESEC)
         Stage D: rx_correct_bus_state()     (ecc module -- H1 differential)
         Stage E: rx_process_bus_state()     (decoder module -- nibble FSM)

   This file owns no domain logic.  It is intentionally thin so that
   the three functional modules can be tested independently.

   RESET SYNCHRONISATION:
     When the TX receives '=', it zeros all 25 output bits via a hardware
     clear of the shift registers.  The RX detects this all-zero condition
     and resets its internal state to match, preventing decode corruption
     from the discontinuous bus transition.
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
   rx_prev_data_ready
   ---------------------------------------------------------------------------
   Local edge-detection latch for the DATA_READY polling loop.
   Using uint8_t for portability (avoids C51-specific 'bit' type).
   ---------------------------------------------------------------------------
*/
static uint8_t rx_prev_data_ready = 0;


/* ---------------------------------------------------------------------------
   rx_perform_full_reset
   ---------------------------------------------------------------------------
   Resets all receiver state to match the TX post-reset condition:
     - H1 differential baseline to zero.
     - PESEC correction counters cleared (optional, for clean stats).
     - Nibble FSM back to WAIT_HIGH.

   Called when an all-zero 25-bit word is detected on the bus, indicating
   the TX has executed its '=' reset command.

   IMPORTANT: This must bring the RX into exact synchronisation with the
   TX, which after reset has:
     current_bus_state    = 0
     pesec_redundancy_reg = 0
   ---------------------------------------------------------------------------
*/
static void rx_perform_full_reset(void)
{
    /* Reset H1 differential baseline to match TX post-reset state. */
    rx_previous_bus_state = 0;

    /* Reset ECC statistics (optional; aids debugging). */
    rx_errors_corrected  = 0;
    rx_errors_detected   = 0;
    rx_pesec_corrections = 0;

    /* Reset the nibble-reassembly FSM.
       Any partially-received nibble is discarded, which is correct:
       after TX reset the nibble sequence restarts from scratch. */
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
    uint8_t  curr_data_ready;

    /* ------------------------------------------------------------------
       [C1] Set core clock to full crystal speed FIRST.
       PLLCON default = 0x53 -> CD=3 -> fCORE = 11.0592/8 = 1.38 MHz.
       Timer 3 baud config requires fCORE = 11.0592 MHz (CD=0).
       At CD=3 the UART runs at 1200 baud instead of 9600.
       On ADuC841, reserved bits must be written as 0 (datasheet p. 49).
       ------------------------------------------------------------------
    */
    PLLCON = 0x00;  /* CD=0: fCORE = 11.0592 MHz */

    /* Enable internal 2 KB XRAM mapped to bottom of external data space.
       Required because tx_buf in rx_hw.c is declared xdata.
       Use ORL (|=) to preserve the auto-configured EPM bits (p. 45). */
    CFG841 |= 0x01;  /* XRAMEN = 1 */

    /* ------------------------------------------------------------------
       [M1] Hardware initialisation -- peripherals BEFORE interrupts.
       Order: Timer 3 (baud clock) -> UART -> Ports.
       ------------------------------------------------------------------
    */
    RX_Timer3_Init();
    RX_UART_Init();
    RX_Port_Init();

    /* ------------------------------------------------------------------
       PESEC matrix initialisation.
       Uses the same block configuration as the TX encoder: {3, 2}.
       ------------------------------------------------------------------
    */
    rx_init_pesec_matrices(pesec_config, 2);

    /* ------------------------------------------------------------------
       Decoder and error-correction module reset.
       All baselines start at 0 to match the TX encoder's initial state
       before any nibble has been transmitted.
       ------------------------------------------------------------------
    */
    rx_reset_state_machine();
    rx_previous_bus_state = 0;
    rx_errors_corrected   = 0;
    rx_errors_detected    = 0;
    rx_pesec_corrections  = 0;

    /* ------------------------------------------------------------------
       Main pipeline loop
       ------------------------------------------------------------------
    */
    while (1)
    {
        /* [M2] Pump the non-blocking UART TX buffer each iteration.
           This ensures bytes are sent without blocking the loop. */
        rx_uart_pump();

        curr_data_ready = (uint8_t)DATA_READY;

        if (curr_data_ready && !rx_prev_data_ready)
        {
            /* --- Rising edge detected: new 25-bit word available. --- */

            /* Stage A: Read the full 25-bit word. */
            rx_read_full_bus(&raw_data, &raw_red);

            /* Stage B: TX reset detection.
               When the TX receives '=', it clears all shift register
               outputs to zero.  This all-zero word is not a valid
               data encoding, so it uniquely signals a reset. */
            if (rx_is_bus_reset(raw_data, raw_red))
            {
                rx_perform_full_reset();

                /* Do NOT process this sample through the decode pipeline.
                   The next valid DATA_READY edge will carry real data
                   starting from the TX's fresh zero-state. */
                rx_prev_data_ready = curr_data_ready;
                continue;
            }

            /* Stage C: PESEC error correction (25-bit codeword).
               raw_data and raw_red are corrected IN PLACE.
               On uncorrectable error, we proceed best-effort;
               the H1 layer may catch the residual corruption. */
            rx_pesec_correct(&raw_data, &raw_red);

            /* Stage D: H1 differential validation (15-bit data).
               Verifies the transition weight and updates the baseline. */
            corrected_data = rx_correct_bus_state(raw_data);

            /* Stage E: Nibble extraction and byte reassembly. */
            rx_process_bus_state(corrected_data);

            rx_states_processed++;
        }

        rx_prev_data_ready = curr_data_ready;
    }
}
