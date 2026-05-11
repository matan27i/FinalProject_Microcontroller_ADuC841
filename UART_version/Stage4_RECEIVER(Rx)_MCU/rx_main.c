/*
   File: rx_main.c
   Module: Group 1 Main entry point and pipeline loop (UART Frame version)

   INGRESS PATH:
     UART_ISR (in rx_hw.c) hunts for the 0xAA SOF byte, collects the
     4-byte payload, verifies CRC-8, and stages a 26-bit codeword in
     rx_frame_data / rx_frame_red / rx_frame_parity.  rx_frame_ready
     pulses to 1 to hand the frame off to the main loop.

   Pipeline stages (post H1-removal):
     (Stage A: UART ISR -- see rx_hw.c)
     Stage B: rx_is_bus_reset()        (decoder -- reset detect)
     Stage C: rx_pesec_correct()       (ecc -- 26-bit SEC+DED)
     Stage D: rx_process_bus_state()   (decoder -- absolute nibble decode + FSM)

   The previous Stage D (rx_correct_bus_state, H1 differential validation)
   was removed per design directive.  Nibble extraction in the new Stage D
   relies on the TX invariant
        syndrome(current_bus_state) == last_input_nibble
   so the absolute syndrome of the PESEC-corrected data is itself the
   recovered nibble -- no "previous bus state" anchor needs to be
   maintained, and no resynchronisation handshake is required after a
   dropped frame.  Byte-boundary alignment in the FSM is handled
   independently by rx_skip_next_frame in rx_decoder.c.
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
     - PESEC diagnostic counters cleared
     - Nibble FSM rewound to WAIT_HIGH so the next frame starts a byte
     - Byte-alignment skip cleared (explicit reset overrides any
       in-flight nibble-parity realignment)
     - Error LED extinguished (the link is by definition healthy after
       a clean reset handshake)
*/
static void rx_perform_full_reset(void)
{
    rx_errors_detected   = 0;
    rx_pesec_corrections = 0;

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
    rx_errors_detected    = 0;
    rx_pesec_corrections  = 0;

    /* Enable global interrupts LAST. */
    EA = 1;

    /* Boot-message handshake.  Seeing "RX-READY\r\n" on PuTTY confirms
       the UART TX path (SCON mode + Timer 3 baud + TxD pin) is working,
       independently of any bridge input or PESEC logic. */
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
               arriving cleanly but PESEC is rejecting it. */
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
                   Drop the frame and run the byte-alignment recovery
                   protocol:

                   (1) NIBBLE PARITY  (rx_skip_next_frame ^= 1 if WAIT_HIGH)
                       Each uncorrectable drops exactly one nibble; the
                       nibble FSM must remain even-aligned with the TX
                       byte boundaries.  If the FSM was WAIT_HIGH at the
                       time of the drop, the dropped nibble was a HIGH
                       and its LOW partner (the next clean frame) must
                       be skipped to restore even parity.  If the FSM
                       was WAIT_LOW, the FSM reset below already
                       accounts for the dropped LOW (it discards the
                       orphaned stored HIGH), so no extra skip is
                       needed.  Toggling (^=) lets two successive drops
                       in WAIT_HIGH self-cancel: skip 0 -> 1 -> 0.

                   (2) FSM RESET
                       Always rewind to WAIT_HIGH so any in-progress
                       byte (with a stored HIGH) is discarded -- it
                       could not have been completed cleanly anyway.

                   (3) USER-VISIBLE INDICATION
                       '!U' UART marker (always-on, may drop on ring
                       overflow) plus the LED (sticky, survives bursts).

                   NOTE: Because the receiver no longer maintains a
                   "previous bus state" anchor (H1 differential layer
                   was removed), there is no bus-state desync to repair
                   after a dropped frame -- each subsequent clean frame
                   carries its own absolute bus state from which the
                   nibble is decoded directly. */
                rx_signal_error('U');
                rx_led_on();

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
               Stage D: Nibble extraction and byte reassembly
               ============================================================
               After PESEC, raw_data is the 15-bit data field that the
               TX shifted out for this frame.  The TX maintains
                  syndrome(current_bus_state) == last_input_nibble
               as an invariant, so passing raw_data directly to
               rx_process_bus_state lets the syndrome of the absolute
               bus state recover the nibble without any reference to
               previous frames. */
            rx_process_bus_state(raw_data);

            rx_states_processed++;

            /* ============================================================
               LED follows latest frame quality
               ============================================================
               ON  if PESEC corrected anything this frame
               OFF if the frame was processed entirely cleanly

               The UART '!x' markers are real-time but can be dropped
               if the TX ring saturates; the LED is sticky and survives
               bursts.  It clears only when a whole frame passes through
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
