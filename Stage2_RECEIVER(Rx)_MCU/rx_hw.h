/* =============================================================================
   File: rx_hw.h
   Module: Group 1 -- Hardware Abstraction Layer
   Public interface for peripheral initialisation and raw I/O.

   Porting guide:
     To target a different MCU, replace the function bodies in rx_hw.c.
     Update the pin-mapping defines in rx_system.h.
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

/* Set up all bus-input port pins (data + redundancy) and DATA_READY. */
void RX_Port_Init(void);

/* ---------------------------------------------------------------------------
   rx_read_full_bus
   ---------------------------------------------------------------------------
   Reads the complete 25-bit word (15 data + 10 redundancy) in a single
   atomic snapshot.  Both values are written through output pointers to
   ensure they are captured at the same instant.

   This is the preferred entry point for the main pipeline.  Reading data
   and redundancy in separate calls risks a race if DATA_READY de-asserts
   between the two reads.

   Parameters:
     out_data   Pointer to receive the 15-bit H1 data (bits 0-14).
     out_red    Pointer to receive the 10-bit redundancy (bits 0-9).
   ---------------------------------------------------------------------------
*/
void rx_read_full_bus(uint16_t *out_data, uint16_t *out_red);

/* ---------------------------------------------------------------------------
   rx_send_uart_byte
   ---------------------------------------------------------------------------
   Transmits one byte to the PC via the UART.

   Uses polling (busy-wait on TI) rather than interrupts, so this call
   blocks for approximately one byte-time (~1.04 ms at 9600 baud).
   ---------------------------------------------------------------------------
*/
void rx_send_uart_byte(uint8_t data_byte);

#endif /* RX_HW_H */
