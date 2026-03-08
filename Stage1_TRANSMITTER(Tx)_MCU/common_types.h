/* File: common_types.h
   Basic type definitions and system-wide constants.
*/
#ifndef COMMON_TYPES_H
#define COMMON_TYPES_H

/* Type Definitions */
typedef unsigned char uint8_t;
typedef unsigned int  uint16_t;

/* H1 Bus Encoder Constants */
#define HAMMING_R       4                       /* Number of syndrome bits (m) */
#define HAMMING_N       ((1 << HAMMING_R) - 1)  /* Bus width N = 2^R - 1 = 15 */
#define BUS_STATE_MASK  0x7FFF                  /* Mask for bits 0..14 */

/* Dynamic PESEC Constants */
#define MAX_PESEC_BLOCKS 5

#endif /* COMMON_TYPES_H */