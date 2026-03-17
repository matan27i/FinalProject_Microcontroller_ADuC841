/* File: ecc.h
 * Group 3: PESEC Error Correction + SEC+DED Overall Parity
 * =========================================================
 * Declares the PESEC redundancy state, overall parity bit, matrix
 * storage, block solver metadata, and prototypes for matrix
 * initialisation and ECC computation.
 *
 * Portability: This module is pure algorithmic logic with no hardware
 *              dependencies.  It calls output_to_shift_registers()
 *              from Group 1 only at the final output stage.
 */

#ifndef TX_ECC_H
#define TX_ECC_H

/* ---- PESEC Redundancy Register ---- */
/* 10-bit redundancy vector, updated differentially each cycle.
 * Reset to 0 together with the bus state on a '=' command.
 * [N2] Not volatile: only accessed from main-loop call chain. */
extern uint16_t pesec_redundancy_reg;

/* ---- [W4] Overall Even Parity Bit (SEC+DED) ---- */
/* Computed from all 25 bits (15 data + 10 redundancy) after each
 * redundancy update.  Shifted out as the 26th bit by
 * output_to_shift_registers() in tx_main.c.
 * 0 = even popcount, 1 = odd popcount. */
extern uint8_t pesec_overall_parity;

/* ---- Dynamic PESEC Matrices ---- */
extern uint8_t PESEC_MAT_D[20];
extern uint8_t PESEC_MAT_A[40];

/* ---- Block Solver Metadata ---- */
extern uint8_t pesec_num_blocks;
extern uint8_t pesec_bit_offsets[MAX_PESEC_BLOCKS];
extern uint8_t pesec_col_offsets[MAX_PESEC_BLOCKS];
extern uint8_t pesec_chunk_masks[MAX_PESEC_BLOCKS];

/* ---- Column Counts ---- */
extern uint8_t pesec_num_d_cols;
extern uint8_t pesec_num_a_cols;

/* ---- Prototypes ---- */
void Init_PESEC_Matrices(uint8_t* m_sizes, uint8_t num_blocks);
void ECC(void);

#endif /* TX_ECC_H */
