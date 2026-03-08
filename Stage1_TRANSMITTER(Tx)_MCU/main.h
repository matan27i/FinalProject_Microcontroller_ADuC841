/* File: main.h
   Global variables and states defined in main.c
*/
#ifndef MAIN_H
#define MAIN_H

#include "common_types.h"

/* Stateful bus state: 15-bit vector */
extern volatile uint16_t current_bus_state;

/* Legacy status flags and tracking */
extern volatile bit buffer_flag;        
extern volatile bit tx_flag;            
extern volatile uint8_t buffer_count;   
extern volatile uint8_t tx_temp_byte;   

#endif /* MAIN_H */