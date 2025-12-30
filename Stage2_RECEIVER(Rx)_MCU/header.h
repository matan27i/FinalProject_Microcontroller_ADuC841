/* File: header.h */
#ifndef HEADER_H
#define HEADER_H

typedef unsigned char uint8_t;

#define HAMMING_R 4
#define HAMMING_N 15

extern volatile bit sample_flag;

void Timer3_Init(void);
void UART_Init(void);
void GlobalINT(void);
void Port_Init(void);
void Timer0_Init(void);
void read_X_from_bus(uint8_t *X);
void get_S_from_X(const uint8_t *X_vector, uint8_t hamming_R, uint8_t *S_output);
uint8_t bits_to_decimal(const uint8_t *bits, uint8_t length);
void transmit_decimal_uart(uint8_t value);

#endif
