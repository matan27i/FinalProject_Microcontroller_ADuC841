/* =============================================================================
   File: rx_main.c
   Module: Group 1 — Main entry point and pipeline loop

   Responsibilities:
     - Hardware startup sequence.
     - Rising-edge detection on DATA_READY.
     - Sequencing the three pipeline stages per bus state:
         Stage A: rx_read_bus_state()     (hw module)
         Stage B: rx_correct_bus_state()  (ecc module)
         Stage C: rx_process_bus_state()  (decoder module)

   This file owns no domain logic.  It is intentionally thin so that
   the three functional modules can be tested independently.
   =============================================================================
*/

#include "rx_system.h"

/* ---------------------------------------------------------------------------
   rx_prev_data_ready
   ---------------------------------------------------------------------------
   Local edge-detection latch for the DATA_READY polling loop.

   Problem it solves:
     Stage 1 asserts DATA_READY and holds it high for several MCU cycles
     while the MCU reads the bus.  A naive  if (DATA_READY)  check would
     re-enter the pipeline on every iteration that the signal remains high,
     decoding the same bus state 2-N times and corrupting the byte stream.

   Solution:
     Capture the current level each iteration and compare to the level
     from the previous iteration.  The pipeline fires only on the
     LOW-to-HIGH transition (exactly once per new bus state).

   Type:
     Using uint8_t instead of the C51-specific 'bit' type so this
     variable declaration is portable to any C compiler.
   ---------------------------------------------------------------------------
*/
static uint8_t rx_prev_data_ready = 0;


/* ---------------------------------------------------------------------------
   main
   ---------------------------------------------------------------------------
*/
void main(void)
{
    uint16_t raw_bus_state;
    uint16_t corrected_bus_state;
    uint8_t  curr_data_ready;

    /* ------------------------------------------------------------------
       Hardware initialisation — order is significant:
         1. Interrupts on (so peripherals can use them if needed).
         2. Timer 3 configured before UART, which depends on Timer 3
            as its baud-rate source.
         3. UART init last (also pre-arms the TI flag).
         4. Port directions set after UART so P1 is not accidentally
            driven as an output before the digital-mode fix takes effect.
       ------------------------------------------------------------------
    */
    RX_GlobalINT();
    RX_Timer3_Init();
    RX_UART_Init();
    RX_Port_Init();

    /* ------------------------------------------------------------------
       Decoder and error-correction module reset.
       rx_previous_bus_state must start at 0 to match the TX encoder's
       own initial bus state before it has sent any nibble.
       ------------------------------------------------------------------
    */
    rx_reset_state_machine();
    rx_previous_bus_state = 0;
    rx_errors_corrected   = 0;
    rx_errors_detected    = 0;

    /* ------------------------------------------------------------------
       Main pipeline loop

       Each iteration that detects a rising edge on DATA_READY:
         A) Reads the raw 15-bit bus state from the parallel port.
         B) Passes it through the ECC module, which:
              - Computes the observed transition from the last good state.
              - Derives the expected (minimal-weight) transition.
              - Returns the corrected state and updates the baseline.
         C) Passes the corrected state to the decoder module, which:
              - Computes the syndrome (= data nibble).
              - Feeds the nibble into the HIGH/LOW FSM.
              - Transmits a complete byte to the PC when both nibbles
                of a pair have been received.
       ------------------------------------------------------------------
    */
    while (1)
    {
        curr_data_ready = (uint8_t)DATA_READY;

        if (curr_data_ready && !rx_prev_data_ready)
        {
            /* Rising edge detected — one new bus state is available. */

            raw_bus_state       = rx_read_bus_state();
            corrected_bus_state = rx_correct_bus_state(raw_bus_state);
            rx_process_bus_state(corrected_bus_state);

            rx_states_processed++;   /* Decoder-module statistics counter */
        }

        rx_prev_data_ready = curr_data_ready;   /* Advance edge latch */
    }
}
