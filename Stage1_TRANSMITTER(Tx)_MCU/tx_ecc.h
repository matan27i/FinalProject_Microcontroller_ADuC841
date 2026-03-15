/* File: ecc.h
 * Group 3: PESEC Error Correction
 * ================================
 * Declares the PESEC redundancy state, matrix storage, block solver
 * metadata, and prototypes for matrix initialisation and ECC computation.
 *
 * Portability: This module is pure algorithmic logic with no hardware
 *              dependencies.  It calls output_to_shift_registers()
 *              from Group 1 only at the final output stage.
 */

#ifndef TX_ECC_H
#define TX_ECC_H

/* ---- PESEC Redundancy Register ---- */
/* 10-bit redundancy vector, updated differentially each cycle.
 * Reset to 0 together with the bus state on a '=' command.      */
extern volatile uint16_t pesec_redundancy_reg;

/* ---- Dynamic PESEC Matrices ---- */
/* Matrix D: redundancy-bit columns.  Max 20 cols (sum of 2^mi - 1). */
extern uint8_t PESEC_MAT_D[20];

/* Matrix A: data-bit columns (everything in the syndrome space
 *           that is NOT a column of D).  Max 40 cols.              */
extern uint8_t PESEC_MAT_A[40];

/* ---- Block Solver Metadata ---- */
extern uint8_t pesec_num_blocks;                        /* Active block count       */
extern uint8_t pesec_bit_offsets[MAX_PESEC_BLOCKS];     /* Syndrome bit offset/block*/
extern uint8_t pesec_col_offsets[MAX_PESEC_BLOCKS];     /* Column offset in D/block */
extern uint8_t pesec_chunk_masks[MAX_PESEC_BLOCKS];     /* Syndrome chunk mask/block*/

/* ---- Column Counts ---- */
extern uint8_t pesec_num_d_cols;    /* Number of columns in Matrix D */
extern uint8_t pesec_num_a_cols;    /* Number of columns in Matrix A */

/* ---- Prototypes ---- */

/* Build matrices D and A from an array of block sizes.
 * m_sizes[i] = number of syndrome bits for block i.
 * num_blocks = number of H1-type sub-blocks.              */
void Init_PESEC_Matrices(uint8_t* m_sizes, uint8_t num_blocks);

/* Main ECC encode cycle: compute redundancy update and output. */
void ECC(void);

#endif /* TX_ECC_H */
