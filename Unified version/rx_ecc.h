/* =============================================================================
   File: rx_ecc.h
   Module: Group 3 -- RX-Specific Error Correction (SEC+DED + H1 Differential)
   =============================================================================

   Public interface for the RX-specific error-correction logic:

     Layer 1 -- PESEC SEC+DED (26-bit word):
       rx_pesec_correct() uses the shared PESEC core (syndromes, parity)
       plus the overall parity bit to detect and correct errors via the
       4-case SEC+DED truth table.

     Layer 2 -- H1 differential (15-bit data):
       Stateful transition-weight validation after PESEC correction.

   All shared PESEC definitions (matrices, metadata, syndrome helpers,
   parity, constants, result codes) are provided by pesec_core.h.
   =============================================================================
*/

#ifndef RX_ECC_H
#define RX_ECC_H

/* Shared PESEC core provides: matrices, metadata, syndrome functions,
   parity, constants, and result codes. */
#include "pesec_core.h"

/* =========================================================================
   RX-SPECIFIC STATE VARIABLES  (H1 differential + statistics)
   ========================================================================= */
extern uint16_t PESEC_MEM rx_previous_bus_state;
extern uint16_t PESEC_MEM rx_errors_corrected;
extern uint16_t PESEC_MEM rx_errors_detected;
extern uint16_t PESEC_MEM rx_pesec_corrections;

/* =========================================================================
   PESEC ERROR CORRECTION  (SEC+DED UPGRADED)
   =========================================================================
   rx_pesec_correct
   ---------------------------------------------------------------------------
   [W4] Performs SEC+DED error handling on the 26-bit received word using
   the shared PESEC parity-check matrix [A | D] plus the overall parity bit.

   SEC+DED truth table:
     S==0, P_OK   -> PESEC_NO_ERROR         (no error)
     S!=0, P_FAIL -> PESEC_CORRECTED_DATA    or PESEC_CORRECTED_RED
                     (single error, corrected)
     S!=0, P_OK   -> PESEC_UNCORRECTABLE    (double error detected)
     S==0, P_FAIL -> PESEC_CORRECTED_PARITY (parity bit itself flipped)

   Parameters:
     data_bits       Pointer to 15-bit data word; corrected in place.
     red_bits        Pointer to 10-bit redundancy word; corrected in place.
     overall_parity  Received overall parity bit (0 or 1) from 26th bus bit.

   Returns:
     PESEC_NO_ERROR         (0)
     PESEC_CORRECTED_DATA   (1)
     PESEC_CORRECTED_RED    (2)
     PESEC_UNCORRECTABLE    (3)
     PESEC_CORRECTED_PARITY (4)
   ---------------------------------------------------------------------------
*/
uint8_t rx_pesec_correct(uint16_t *data_bits, uint16_t *red_bits,
                         uint8_t overall_parity);

/* =========================================================================
   H1 DIFFERENTIAL ERROR CORRECTION
   ========================================================================= */
uint16_t find_minimal_w(uint8_t s_target);
uint16_t rx_correct_bus_state(uint16_t corrected_data);

#endif /* RX_ECC_H */
