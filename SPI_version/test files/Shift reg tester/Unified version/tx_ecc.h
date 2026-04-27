/* =============================================================================
   File: tx_ecc.h
   Module: Group 3 -- TX-Specific PESEC Encoding + SEC+DED Parity
   =============================================================================

   Declares the TX-specific PESEC encoding state (redundancy register,
   overall parity bit) and the ECC() encode-cycle prototype.

   All shared PESEC definitions (matrices, syndrome helpers, parity,
   constants, result codes) are provided by pesec_core.h.

   Portability:
     This module is pure algorithmic logic with no hardware
     dependencies.  It calls output_to_shift_registers() from
     Group 1 only at the final output stage.
   =============================================================================
*/

#ifndef TX_ECC_H
#define TX_ECC_H

/* Shared PESEC core provides: matrices, metadata, syndrome functions,
   parity, constants, and result codes. */
#include "pesec_core.h"

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

/* ---- TX Encode-Cycle Prototype ---- */
void ECC(void);

#endif /* TX_ECC_H */
