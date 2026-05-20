/*
   File: rx_hw.h
   Module: Group 1  Hardware Abstraction Layer (UART Frame version)
   Public interface for peripheral initialisation and UART data reception.
*/

#ifndef RX_HW_H
#define RX_HW_H

#include "rx_types.h"

/*
   UART FRAME RECEIVE PROTOCOL

   The Arduino Mega sends a 6-byte UART packet for each 26-bit bus word
   over USART1 at 115200 baud, 8N1:

     Byte 0:  0xAA                  Start-of-frame sync
     Byte 1:  H1 data bits  [7:0]
     Byte 2:  bit 7 = overall parity
              bits [6:0] = H1 data [14:8]
     Byte 3:  ECC red bits  [7:0]
     Byte 4:  ECC red bits  [9:8]   (bits 7-2 = 0)
     Byte 5:  CRC-8 over bytes 1..4 (poly 0x07, init 0x00)

   The UART ISR runs a 3-state accumulator (hunt SOF -> collect 4 payload
   bytes -> verify CRC).  On CRC success it assembles the 15-bit data,
   10-bit redundancy, and parity into the staging registers below and
   sets rx_frame_ready = 1.  On CRC failure the frame is silently dropped
   and the state machine returns to SOF hunt.
*/

/* Assembled 15-bit data and 10-bit redundancy from the last good frame.
   Valid only when rx_frame_ready == 1. */
extern volatile uint16_t rx_frame_data;
extern volatile uint16_t rx_frame_red;

/* Overall parity bit (the 26th bus bit).  Valid only when
   rx_frame_ready == 1.  0 or 1: TX-computed even parity of the 25-bit
   (data + redundancy) word. */
extern volatile uint8_t rx_frame_parity;

/* Pulsed to 1 by the UART ISR when a complete, CRC-valid frame is
   ready for the main loop.  The main loop must clear this to 0 after
   consuming the staging registers. */
extern volatile uint8_t rx_frame_ready;

/* Diagnostic counter: number of CRC failures since reset.  Incremented
   from the ISR; read-only from the main loop. */
extern volatile uint16_t rx_frame_crc_errors;


/*
   Peripheral initialisation
*/
void RX_Timer3_Init(void);

/* Configures SCON Mode 1, enables the serial interrupt, and primes the
   TX ring buffer.  Call after RX_Timer3_Init() but before EA=1. */
void RX_UART_Init(void);

/* Byte-oriented debug/output ring buffer.  rx_send_uart_byte() enqueues
   one byte for transmission to PuTTY; the UART ISR drains the ring.
   If the ring is full the byte is dropped silently. */
void rx_send_uart_byte(uint8_t data_byte);


/*
   ERROR-INDICATOR LED -- onboard hardware fallback for ECC events.
   Default pin: P3.4, active-low.  Tweakable via RX_LED_ACTIVE_LEVEL in
   rx_hw.c.  See rx_hw.c for full rationale.
*/
void rx_led_on(void);
void rx_led_off(void);


/*
   UART ERROR MARKER -- emits a 2-byte marker '!' + kind.  Standard
   kind codes used by rx_main.c:
     'U'  PESEC uncorrectable (double error, frame dropped)
     'd'  PESEC corrected single data-bit error
     'r'  PESEC corrected single redundancy-bit error
     'p'  PESEC corrected the overall parity bit itself
   The CALLER is responsible for gating this on the debug strap; the
   function itself is unconditional and non-blocking.  Drop happens
   silently if the TX ring is full; the LED is the always-on backup.
*/
void rx_signal_error(uint8_t kind);


/*
   DEBUG MODE STRAP READOUT
   ---
   Live pin sample: returns 1 if P3.5 is currently tied low externally,
   0 if it floats high.  Called only when '~' arrives in the decoded
   byte stream, to refresh rx_debug_mode_active.  Direct callers should
   prefer the cached flag below to avoid the per-frame pin read.
*/
uint8_t rx_debug_active(void);


/*
   CACHED DEBUG-MODE FLAG
   ---
   Reflects the latched debug state set by the most recent '~' command.
   Read by the main loop (once per CRC-valid frame) and by UART_ISR
   (once per CRC-mismatch event).  Written only by rx_process_bus_state
   when it decodes a '~' byte from the TX stream.

   Initial value: 0 (debug OFF).  Send '~' on the host with P3.5 strap
   to GND to set this to 1; send '~' again with P3.5 floating to clear
   it back to 0.
*/
extern volatile uint8_t xdata rx_debug_mode_active;

#endif
