/* File: header.h
 * Master Header File
 * contain all module headers and contains board-level hardware definitions.
 * Include this file in all .c files to gain access to the full system.
 *
 * Target: ADuC841 (8052 single-cycle core, 11.0592 MHz crystal)
 */

#ifndef TX_HEADER_H
#define TX_HEADER_H

#include <aduc841.h>

/* 
 * COMMON TYPE DEFINITIONS
 */
typedef unsigned char  uint8_t;
typedef unsigned int   uint16_t;

/* 
 * SYSTEM-WIDE CONSTANTS
 */

/* H1 Bus Encoder Parameters */
#define HAMMING_R        4                          /* Syndrome bit width (m)       */
#define HAMMING_N        ((1 << HAMMING_R) - 1)     /* Bus width N = 2^R - 1 = 15   */
#define BUS_STATE_MASK   0x7FFF                     /* Mask for bits 0..14          */

/* PESEC ECC Parameters */
#define MAX_PESEC_BLOCKS 5                          /* Max H1-type sub-blocks       */
#define PESEC_RED_MASK   0x03FF                     /* Mask for 10 redundancy bits  */

/* Timer 2 Reload Constants (124 counts = ~11.2 us at 11.0592 MHz) */
#define T2_RELOAD_H      0xFF
#define T2_RELOAD_L      0x84

/* UART Receive Buffer */
#define RX_BUF_SIZE      16                         /* Must be power of 2           */

/* 
 * BOARD-LEVEL HARDWARE DEFINITIONS
 * Shift Register Pin Mapping (active pins on Port 2)
 *   74HC595 daisy-chain interface
 */
sbit SR_CLK   = P2^0;   /* SRCLK   Shift register clock              */
sbit SR_DATA  = P2^1;   /* SER     Serial data input                 */
sbit SR_LATCH = P2^3;   /* RCLK    Storage register clock (latch)    */
sbit SR_CLR   = P2^4;   /* SRCLR   Shift register clear (active low) */

/* 
 * MODULE HEADERS
 * Include order matters: each module may depend on types/constants above.
 */
#include "tx_main.h"        /* Group 1: Main loop & hardware peripherals  */
#include "tx_encoder.h"     /* Group 2: H1-type bus encoding & UART logic */
#include "tx_ecc.h"         /* Group 3: PESEC error correction            */

#endif /* TX_HEADER_H */
