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
       Hardware initialisation -- order is significant:
         1. Global interrupts on.
         2. Timer 3 before UART (UART depends on Timer 3 for baud rate).
         3. UART init (also pre-arms TI flag).
         4. Port init last (configures all 25 bus-input pins).
       ------------------------------------------------------------------
    */
    RX_GlobalINT();
    RX_Timer3_Init();
    RX_UART_Init();
    RX_Port_Init();

    /* ------------------------------------------------------------------
       PESEC matrix initialisation.
       Uses the same block configuration as the TX encoder: {3, 2}.
       This builds matrices A and D so the receiver can compute syndromes
       and correct single-bit errors across the full 25-bit codeword.
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

       PIPELINE STAGES (per rising edge on DATA_READY):

       A) READ: Sample the full 25-bit word from the parallel bus.
          - raw_data: 15-bit H1 data  (P0 + P1[6:0])
          - raw_red:  10-bit PESEC redundancy  (P2 + P3[4:3])

       B) RESET DETECT: Check if all 25 bits are zero.
          If so, the TX has executed a '=' reset.  Perform a full
          receiver reset and skip the remaining stages for this sample.
          This prevents the H1 differential tracker from seeing a
          discontinuous multi-bit transition and producing garbage.

       C) PESEC CORRECT: Compute the PESEC syndrome across the 25-bit
          word and correct any single-bit error.  After this stage,
          raw_data and raw_red contain the corrected values.
          Result codes:
            PESEC_NO_ERROR       -- clean codeword, no correction needed.
            PESEC_CORRECTED_DATA -- data bit flipped and fixed.
            PESEC_CORRECTED_RED  -- redundancy bit flipped and fixed.
            PESEC_UNCORRECTABLE  -- multi-bit error, data may be corrupt.
          On UNCORRECTABLE, we still feed the data forward (best-effort).

       D) H1 CORRECT: Pass the PESEC-corrected 15-bit data through the
          H1 differential tracker.  This verifies that consecutive bus
          states differ by at most 1 bit (the TX encoder's invariant)
          and updates the differential baseline.

       E) DECODE: Feed the validated 15-bit state into the nibble FSM,
          which extracts the 4-bit data nibble and reassembles byte pairs.
       ------------------------------------------------------------------
    */
    while (1)
    {
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
