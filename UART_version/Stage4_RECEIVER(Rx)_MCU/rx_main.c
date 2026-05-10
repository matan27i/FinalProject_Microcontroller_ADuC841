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
   ---
   Triggered when the TX sends an all-zero "bus reset" frame (the '='
   command on the host).  Restores the receiver to a clean baseline:
     - H1 differential anchor zeroed (matches TX's current_bus_state = 0)
     - Statistics counters cleared
     - Nibble FSM rewound to WAIT_HIGH so the next frame starts a byte
     - rx_resync_pending cleared (any in-flight desync recovery is now
       superseded by the explicit reset)
     - Error LED extinguished (the link is by definition healthy after
       a clean reset handshake)
*/
static void rx_perform_full_reset(void)
{
    rx_previous_bus_state = 0;

    rx_errors_corrected  = 0;
    rx_errors_detected   = 0;
    rx_pesec_corrections = 0;

    rx_resync_pending    = 0;
    rx_skip_next_frame   = 0;   /* explicit '=' reset overrides any pending realignment */

    rx_reset_state_machine();

    rx_led_off();
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
            uint8_t frame_had_error;

            /* Atomically snapshot the frame staging registers.  EA=0 is
               the smallest critical section that protects the multi-byte
               read against a concurrent UART_ISR write of the next frame. */
            EA = 0;
            raw_data       = rx_frame_data;
            raw_red        = rx_frame_red;
            raw_parity     = rx_frame_parity;
            rx_frame_ready = 0;
            EA = 1;

            frame_had_error = 0;

#if RX_DEBUG
            /* 'F' = a CRC-valid 4-byte payload was handed to the main
               loop.  Seeing 'F' but no decoded byte means the frame is
               arriving cleanly but PESEC or H1 is rejecting it. */
            rx_send_uart_byte('F');
#endif

            /* ============================================================
               Stage B: TX reset detection
               ============================================================ */
            if (rx_is_bus_reset(raw_data, raw_red))
            {
#if RX_DEBUG
                rx_send_uart_byte('R');
#endif
                rx_perform_full_reset();
                /* rx_perform_full_reset() turns the LED off explicitly --
                   a clean reset handshake means the link is healthy. */
                continue;
            }

            /* ============================================================
               Stage C: SEC+DED PESEC error correction
               ============================================================ */
            pesec_result = rx_pesec_correct(&raw_data, &raw_red, raw_parity);

            if (pesec_result == PESEC_UNCORRECTABLE)
            {
                /* Double error -- the data + red field are unreliable.
                   Drop the frame, and run the full recovery protocol:

                   (1) BUS-STATE BASELINE  (rx_resync_pending = 1)
                       The TX has already advanced its current_bus_state
                       past this frame.  Marking resync makes the H1
                       differential layer adopt the next clean frame's
                       absolutely-decoded state as a fresh anchor instead
                       of computing a stale 2-step delta -- which would
                       not match a minimal codeword and would propagate
                       the desync indefinitely.  See rx_correct_bus_state
                       in rx_ecc.c.

                   (2) NIBBLE PARITY  (rx_skip_next_frame ^= 1 if WAIT_HIGH)
                       Each uncorrectable drops exactly one nibble; the
                       FSM must remain even-aligned with TX byte boundaries.
                       If the FSM was WAIT_HIGH at the time of the drop,
                       the dropped nibble was a HIGH and its LOW partner
                       (the next clean frame) must be skipped to restore
                       even parity.  If the FSM was WAIT_LOW, the FSM
                       reset below already accounts for the dropped LOW
                       (it discards the orphaned stored HIGH), so no
                       extra skip is needed.  Toggling (^=) lets two
                       successive drops in WAIT_HIGH self-cancel:
                       skip 0 -> 1 -> 0.

                   (3) FSM RESET
                       Always rewind to WAIT_HIGH so any in-progress byte
                       (with a stored HIGH) is discarded -- it could not
                       have been completed cleanly anyway.

                   (4) USER-VISIBLE INDICATION
                       '!U' UART marker (always-on, may drop on ring
                       overflow) plus the LED (sticky, survives bursts). */
                rx_signal_error('U');
                rx_led_on();

                rx_resync_pending = 1;

                if (rx_current_state == RX_STATE_WAIT_HIGH)
                {
                    rx_skip_next_frame ^= 1u;
                }

                rx_reset_state_machine();
                continue;
            }

            /* PESEC fixed a single error somewhere -- frame is still
               usable, but record the event for the LED + UART trace. */
            if (pesec_result == PESEC_CORRECTED_DATA)
            {
                rx_signal_error('d');
                frame_had_error = 1;
            }
            else if (pesec_result == PESEC_CORRECTED_RED)
            {
                rx_signal_error('r');
                frame_had_error = 1;
            }
            else if (pesec_result == PESEC_CORRECTED_PARITY)
            {
                rx_signal_error('p');
                frame_had_error = 1;
            }
            /* else PESEC_NO_ERROR -- nothing to log */

            /* ============================================================
               Stage D: H1 differential validation
               ============================================================
               Returns the bus state that is consistent with the current
               nibble's syndrome.  Sets rx_h1_correction_flag if the
               received delta did not match a minimal codeword and the
               state was reconstructed from the previous baseline.

               When rx_resync_pending was set on a previous frame, the
               function automatically takes the resync path (adopt
               corrected_data as the new baseline) -- see rx_ecc.c. */
            corrected_data = rx_correct_bus_state(raw_data);

            if (rx_h1_correction_flag)
            {
                rx_signal_error('H');
                frame_had_error = 1;
            }

            /* ============================================================
               Stage E: Nibble extraction and byte reassembly
               ============================================================ */
            rx_process_bus_state(corrected_data);

            rx_states_processed++;

            /* ============================================================
               LED follows latest frame quality
               ============================================================
               ON  if any error event in this frame (corrected or H1)
               OFF if the frame was processed entirely cleanly

               This is the second leg of the user-visible indicator: the
               UART '!x' markers are real-time but can be dropped if the
               TX ring saturates; the LED is sticky and survives bursts.
               It clears here only when a whole frame passes through
               without any correction -- exactly the contract specified
               in the requirements. */
            if (frame_had_error)
            {
                rx_led_on();
            }
            else
            {
                rx_led_off();
            }
        }
    }
}
