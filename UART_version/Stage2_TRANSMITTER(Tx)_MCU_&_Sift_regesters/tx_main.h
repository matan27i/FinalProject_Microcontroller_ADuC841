/* File: main.h
 * Group 1: Main Loop & Hardware Peripherals
 * Declares state variables owned by main.c (UART flags, Timer2 state,
 * buffer tracking) and prototypes for all hardware initialisation and
 * shift-register output functions.
 */

#ifndef TX_MAIN_H
#define TX_MAIN_H

/* ---- UART Receive Ring Buffer ---- */
extern volatile uint8_t rx_buf[RX_BUF_SIZE];
extern volatile uint8_t rx_head;
extern volatile uint8_t rx_tail;

/* ---- Timer 2 Hardware State ---- */
extern volatile uint8_t counter_t2;
extern volatile uint8_t flag_t2_mod;

/* ---- Hardware Initialisation Prototypes ---- */
void Port_Init(void);
void Init_Timer2(void);
void BaudRate_Init(void);
void UART_Init(void);
void GlobalINT(void);

/* ---- Shift-Register Output ---- */
void output_to_shift_registers(void);

#endif /* TX_MAIN_H */
