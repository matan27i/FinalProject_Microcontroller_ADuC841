/* File: main.h
 * Group 1: Main Loop & Hardware Peripherals
 * ==========================================
 * Declares state variables owned by main.c (UART flags, Timer2 state,
 * buffer tracking) and prototypes for all hardware initialisation and
 * shift-register output functions.
 *
 * Portability: To migrate to a different MCU, only this module and the
 *              board-level pin definitions in header.h need to change.
 */

#ifndef TX_MAIN_H
#define TX_MAIN_H

/* ---- UART / Main-Loop State Variables ---- */
extern volatile bit    buffer_flag;     /* Set when batch terminator received   */
extern volatile uint8_t buffer_count;   /* Nibble count processed in batch      */

/* ---- UART Receive Ring Buffer ---- */
extern volatile uint8_t rx_buf[RX_BUF_SIZE];
extern volatile uint8_t rx_head;        /* Written by ISR                       */
extern volatile uint8_t rx_tail;        /* Read by main loop                    */

/* ---- Timer 2 Hardware State ---- */
extern volatile uint8_t counter_t2;     /* Edge counter used by Timer2 ISR      */
extern volatile uint8_t flag_t2_mod;    /* Timer2 ISR operating mode (0/1/2)    */

/* ---- Hardware Initialisation Prototypes ---- */
void Port_Init(void);
void Init_Timer2(void);
void BaudRate_Init(void);
void UART_Init(void);
void GlobalINT(void);

/* ---- Shift-Register Output ---- */
void output_to_shift_registers(void);

#endif /* TX_MAIN_H */
