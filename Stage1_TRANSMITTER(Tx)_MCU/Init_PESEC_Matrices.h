/* File: init_pesec_matrices.h
   Dynamic PESEC matrices allocation and initialization.
*/
#ifndef INIT_PESEC_MATRICES_H
#define INIT_PESEC_MATRICES_H

#include "common_types.h"

/* Dynamic PESEC Global Variables (Extern Declarations) */
extern volatile uint16_t pesec_redundancy_reg;

extern uint8_t PESEC_MAT_D[20]; 
extern uint8_t PESEC_MAT_A[40]; 

extern uint8_t pesec_num_blocks;
extern uint8_t pesec_bit_offsets[MAX_PESEC_BLOCKS];
extern uint8_t pesec_col_offsets[MAX_PESEC_BLOCKS];
extern uint8_t pesec_chunk_masks[MAX_PESEC_BLOCKS];

extern uint8_t pesec_num_d_cols;
extern uint8_t pesec_num_a_cols;

/* Initialization Prototype */
void Init_PESEC_Matrices(uint8_t* m_sizes, uint8_t num_blocks);

#endif /* INIT_PESEC_MATRICES_H */