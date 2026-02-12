/* File: rx_peripherals.c
   Peripheral Configuration for ADuC841 (RECEIVER SIDE)
   UART and Timer Setup for H1-Type Bus Decoder
 */

#include <aduc841.h>
#include "rx_header.h"

/* 
   RX_Timer3_Init
   
   Configure Timer 3 as UART baud rate generator for 9600 baud.
   
   ADuC841 Timer 3 configuration for 9600 baud with 11.0592 MHz crystal:
   T3CON settings and T3FD fractional divider per datasheet.
   
   This configuration is IDENTICAL to TX side to ensure baud rate matching.
 */
void RX_Timer3_Init(void)
{
    T3CON &= 0xFE;  /* Disable Timer 3 */
    T3CON |= 0x86;  /* Set mode bits for UART baud generation */
    T3FD = 0x08;    /* Fractional divider for 9600 baud accuracy */
}

/*
   RX_UART_Init
   
   Initialize UART for 8-bit data, no parity, 1 stop bit (8N1).
   Baud rate: 9600 (set by Timer 3).
   
   RECEIVER CONFIGURATION:
   - Mode 1: 8-bit UART, variable baud rate
   - REN: NOT enabled (we're transmitting only, not receiving from PC)
   - TI/RI: Cleared
   - ES: Serial interrupt NOT enabled (we don't receive from UART in RX module)
   
   Note: This RX module receives data from Stage 1 (bus interface),
         and TRANSMITS to PC via UART. We don't use UART receive.
 */
void RX_UART_Init(void)
{
    SM0 = 0;    /* Mode 1: 8-bit UART */
    SM1 = 1;    /* Mode 1: variable baud rate */
    REN = 0;    /* Disable receiver (we only transmit to PC) */
    RI = 0;     /* Clear receive interrupt flag */
    TI = 0;     /* Clear transmit interrupt flag */
    ES = 0;     /* Disable serial interrupt (not needed) */
    
    /* 
       UART is now configured for transmission only.
       Use rx_send_uart_byte() to send data to PC.
     */
}

/*
   RX_GlobalINT
   
   Enable global interrupt master switch.
   
   Note: Even if we're not using UART interrupts, this is required
         if you want to use external interrupts (e.g., INT0 for DATA_READY).
 */
void RX_GlobalINT(void)
{
    EA = 1;     /* Enable all interrupts */
}

/*
   RX_Port_Init
   
   Initialize interface pins to Stage 1 error corrector.
   
   USER CUSTOMIZATION REQUIRED:
   Configure pins based on your hardware interface to Stage 1.
   
   Example configurations:
   
   1. Parallel interface (P0 = low byte, P1 = high bits):
      P0 = 0xFF;  // Set as input (or configure as needed)
      P1 |= 0x7F; // Set P1.0-P1.6 as input
   
   2. Serial interface:
      // Configure SPI/I2C pins
   
   3. Handshake signals:
      // Configure DATA_READY pin as input
      // Configure ACK pin as output
 */
void RX_Port_Init(void)
{
    /*
       EXAMPLE: Parallel interface on P0 and P1
       Modify based on your actual hardware.
     */
    
    /* Set P0 as input (for bus state bits 0-7)
       In 8051, writing 1 to port pin configures it as input */
    P0 = 0xFF;
    
    /* Set P1.0-P1.6 as input (for bus state bits 8-14) */
    P1 |= 0x7F;
    
    /* P3.2 (DATA_READY) is automatically input after reset */
    /* If using as interrupt: Configure IT0/EX0 in main() */
    
    /*
       EXAMPLE: If you need to configure control signals:
       
       sbit ACK_PIN = P3^3;  // Acknowledge to Stage 1
       ACK_PIN = 0;          // Initialize low
     */
}

/*
   OPTIONAL: External interrupt ISR for DATA_READY signal
   
   If Stage 1 uses interrupt-driven signaling instead of polling,
   enable this ISR and configure in main().
 */
#if 0  /* Set to 1 to enable */

/* External interrupt 0 handler - DATA_READY from Stage 1 */
void EXT0_ISR(void) interrupt 0
{
    /* Set flag for main loop */
    rx_data_available = 1;
    
    /* Alternative: Process immediately (be careful with timing)
       uint16_t bus_state = rx_read_bus_state();
       rx_process_bus_state(bus_state);
     */
}

#endif  /* End optional ISR */
