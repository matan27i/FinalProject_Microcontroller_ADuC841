/*
   File: rx_hw.h
   Module: Group 1 -- Hardware Abstraction Layer (UART Frame version)
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
   UART ERROR MARKER -- always-on (not RX_DEBUG-gated).  Emits a 2-byte
   marker '!' + kind.  Standard kind codes used by rx_main.c:
     'U'  PESEC uncorrectable (double error, frame dropped)
     'd'  PESEC corrected single data-bit error
     'r'  PESEC corrected single redundancy-bit error
     'p'  PESEC corrected the overall parity bit itself
   Non-blocking; if the TX ring is saturated the marker is dropped and
   the LED becomes the only indication of the event.
*/
void rx_signal_error(uint8_t kind);

#endif
