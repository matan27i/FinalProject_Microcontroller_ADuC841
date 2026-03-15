/* =============================================================================
   File: rx_hw.h
   Module: Group 1 — Hardware Abstraction Layer
   Public interface for peripheral initialisation and raw I/O.

   Porting guide:
     To target a different MCU, replace the function bodies in rx_hw.c.
     Update the DATA_READY pin and port-mapping defines in rx_system.h.
     No changes to rx_decoder or rx_ecc should ever be necessary.
   =============================================================================
*/

#ifndef RX_HW_H
#define RX_HW_H

#include "rx_types.h"

/* ---------------------------------------------------------------------------
   Peripheral initialisation
   Call these once, in the order shown, before entering the main loop.
   ---------------------------------------------------------------------------
*/

/* Enable the global interrupt master switch (EA = 1). */
void RX_GlobalINT(void);

/* Configure Timer 3 as the UART baud-rate generator for 9600 baud. */
void RX_Timer3_Init(void);

/* Configure the UART for 8N1, 9600 baud, transmit-only operation. */
void RX_UART_Init(void);

/* Set up the bus-input port pins and the DATA_READY signal. */
void RX_Port_Init(void);

/* ---------------------------------------------------------------------------
   rx_read_bus_state
   ---------------------------------------------------------------------------
   Reads one 15-bit bus state from the Stage 1 hardware interface and
   returns it with bits 0-14 populated and bit 15 always zero.

   The default implementation in rx_hw.c uses a parallel port (P0 for
   bits 0-7, P1.0-P1.6 for bits 8-14).  Replace with an SPI, I2C, or
   memory-mapped variant as required by your Stage 1 interface.

   Must only be called when DATA_READY is asserted.
   ---------------------------------------------------------------------------
*/
uint16_t rx_read_bus_state(void);

/* ---------------------------------------------------------------------------
   rx_send_uart_byte
   ---------------------------------------------------------------------------
   Transmits one byte to the PC via the UART.

   Uses polling (busy-wait on TI) rather than interrupts, so this call
   blocks for approximately one byte-time (~1.04 ms at 9600 baud).
   Replace with a buffered / DMA variant if throughput becomes a concern.
   ---------------------------------------------------------------------------
*/
void rx_send_uart_byte(uint8_t data_byte);

#endif /* RX_HW_H */
