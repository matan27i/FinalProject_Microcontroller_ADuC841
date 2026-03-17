/* =============================================================================
   File: rx_hw.h
   Module: Group 1 -- Hardware Abstraction Layer (SPI Slave version)
   Public interface for peripheral initialisation and SPI data reception.

   FIX LOG:
     [F2] RX_SPI_Slave_Init() now configures P1.5 as digital input
          before enabling SPI (ADuC841 Port 1 analog-mode anomaly).
     [W4] SPI frame now carries 26 bits: rx_spi_parity added.
   =============================================================================
*/

#ifndef RX_HW_H
#define RX_HW_H

#include "rx_types.h"

/* ---------------------------------------------------------------------------
   SPI Receive State -- shared between ISR and main loop.
   ---------------------------------------------------------------------------
   The Arduino Mega sends a 4-byte SPI frame for each 26-bit bus word:

     Byte 0:  H1 data bits  [7:0]
     Byte 1:  bit 7 = overall parity     <-- NEW (W4)
              bits [6:0] = H1 data [14:8]
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

/* [W4] Overall parity bit extracted from byte 1 bit 7.
   Valid only when rx_spi_frame_ready == 1.
   0 or 1: the TX-computed even parity of the original 25-bit word. */
extern volatile uint8_t rx_spi_parity;

/* Pulsed to 1 by the SPI ISR when a complete 4-byte frame has been
   received.  The main loop must clear this to 0 after consuming. */
extern volatile uint8_t rx_spi_frame_ready;

/* ---------------------------------------------------------------------------
   Peripheral initialisation
   ---------------------------------------------------------------------------
*/
void RX_Timer3_Init(void);
void RX_UART_Init(void);

/* [F2] Configures P1.5 as digital, then enables SPI slave + INT0. */
void RX_SPI_Slave_Init(void);

void rx_send_uart_byte(uint8_t data_byte);
void rx_uart_pump(void);

#endif /* RX_HW_H */
