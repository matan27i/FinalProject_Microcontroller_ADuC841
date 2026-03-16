/* =============================================================================
   File: rx_hw.h
   Module: Group 1 -- Hardware Abstraction Layer (SPI Slave version)
   Public interface for peripheral initialisation and SPI data reception.

   MODIFICATION SUMMARY:
     Replaced parallel port bus reading (rx_read_full_bus, RX_Port_Init)
     with SPI slave reception.  The 25-bit bus word now arrives as 4 bytes
     from the Arduino Mega 2560 bridge via hardware SPI.

   Porting guide:
     To target a different MCU, replace the function bodies in rx_hw.c.
     Update the SPI register names and interrupt vectors as needed.
     No changes to rx_decoder or rx_ecc should ever be necessary.
   =============================================================================
*/

#ifndef RX_HW_H
#define RX_HW_H

#include "rx_types.h"

/* ---------------------------------------------------------------------------
   SPI Receive State -- shared between ISR and main loop.
   ---------------------------------------------------------------------------
   The Arduino Mega sends a 4-byte SPI frame for each 25-bit bus word:

     Byte 0:  H1 data bits  [7:0]
     Byte 1:  H1 data bits [14:8]   (bit 7 = 0, unused)
     Byte 2:  ECC red bits  [7:0]
     Byte 3:  ECC red bits  [9:8]   (bits 7-2 = 0, unused)

   The ISR accumulates these bytes.  When all 4 are received, it sets
   rx_spi_frame_ready = 1 and copies the assembled values into the
   staging registers for the main loop to consume.
   ---------------------------------------------------------------------------
*/

/* Assembled 15-bit data and 10-bit redundancy from the last complete
   SPI frame.  Valid only when rx_spi_frame_ready == 1. */
extern volatile uint16_t rx_spi_data;
extern volatile uint16_t rx_spi_red;

/* Pulsed to 1 by the SPI ISR when a complete 4-byte frame has been
   received.  The main loop must clear this to 0 after consuming the
   data.  Set atomically by the ISR; cleared by the main loop. */
extern volatile uint8_t rx_spi_frame_ready;

/* ---------------------------------------------------------------------------
   Peripheral initialisation
   Call these once, in the order shown, before entering the main loop.
   ---------------------------------------------------------------------------
*/

/* Configure Timer 3 as the UART baud-rate generator for 9600 baud.
   (Unchanged from original design.) */
void RX_Timer3_Init(void);

/* Configure the UART for 8N1, 9600 baud, transmit-only operation.
   (Unchanged from original design.) */
void RX_UART_Init(void);

/* Configure the ADuC841 SPI peripheral in slave mode (Mode 0) and
   set up INT0 on P3.2 for frame-sync (falling edge of /SS from Mega). */
void RX_SPI_Slave_Init(void);

/* ---------------------------------------------------------------------------
   rx_send_uart_byte
   ---------------------------------------------------------------------------
   Enqueues one byte into the non-blocking TX output buffer.
   If the buffer is full, the byte is silently dropped.
   (Unchanged from original design.)
   ---------------------------------------------------------------------------
*/
void rx_send_uart_byte(uint8_t data_byte);

/* ---------------------------------------------------------------------------
   rx_uart_pump
   ---------------------------------------------------------------------------
   Drains one byte from the TX buffer into SBUF if TI is set.
   Must be called from the main loop on every iteration.
   (Unchanged from original design.)
   ---------------------------------------------------------------------------
*/
void rx_uart_pump(void);

#endif /* RX_HW_H */
