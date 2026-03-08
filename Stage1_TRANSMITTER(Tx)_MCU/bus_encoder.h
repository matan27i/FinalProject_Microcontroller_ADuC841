/* File: bus_encoder.h
   Core H1-Type Stateful Bus Encoder functions.
*/
#ifndef BUS_ENCODER_H
#define BUS_ENCODER_H

#include "common_types.h"

/* H1-Type Bus Encoder Core Prototypes */
void process_nibble(uint8_t s_new);
uint8_t compute_syndrome_from_bus(uint16_t bus_state);
uint16_t find_minimal_w(uint8_t s_target);

#endif /* BUS_ENCODER_H */