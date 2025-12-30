/* File: header.h */

#ifndef HEADER_H
#define HEADER_H


typedef unsigned char uint8_t;

// Hamming parameters
#define HAMMING_R   4                    
#define HAMMING_N   ((1 << HAMMING_R) - 1) // N=15

#define Max_in_for_outpins 10 // Maximum chars via input to buffer


extern volatile uint8_t tempsnew;
extern volatile uint8_t Snew;
extern volatile uint8_t buffer_count;   // How many from buufer will send
extern volatile bit buffer_flag;        // Flag to process

// Buffer for the expanded bits Uses 'xdata'
extern volatile uint8_t xdata S_stream_expanded[Max_in_for_outpins * HAMMING_R];

void Timer3_Init (void);
void UART_Init(void);
void GlobalINT(void);

int hexCharToInt(uint8_t tempsnew);
bit ProcessRxData(uint8_t );
uint8_t get_X_from_S(const uint8_t *S_vector, uint8_t hamming_R, uint8_t *X_output);
int get_multi_X_from_S(const uint8_t *S_stream, const uint8_t *R_list, uint8_t packet_count, uint8_t *X_stream);
void Port_Init(void);
void transmit_X_to_bus(const uint8_t *X_stream_output);

#endif
