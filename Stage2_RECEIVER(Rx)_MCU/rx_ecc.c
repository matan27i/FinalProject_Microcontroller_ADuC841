/* =============================================================================
   File: rx_ecc.c
   Module: Group 3 — Error Correction
   Implementation of find_minimal_w() and rx_correct_bus_state().

   Dependencies:
     rx_decoder.h  ->  rx_compute_syndrome() (H1 matrix multiply)
     rx_types.h    ->  uint8_t, uint16_t, HAMMING_N, BUS_STATE_MASK

   Portability:
     No hardware registers are accessed here.  The only external call is
     rx_compute_syndrome(), which is also pure arithmetic.  This entire
     module can be compiled and unit-tested on any ISO C89 host.
   =============================================================================
*/

#include "rx_system.h"

/* ---------------------------------------------------------------------------
   Module-private state variables
   Defined here; declared extern in rx_ecc.h.
   ---------------------------------------------------------------------------
*/

/* Last accepted (error-corrected) bus state.
   Initialised to 0 to match the TX encoder's own starting state, ensuring
   that the very first received transition is evaluated correctly. */
volatile uint16_t rx_previous_bus_state = 0;

/* Running count of single-bit corrections applied. */
volatile uint16_t rx_errors_corrected = 0;

/* Reserved for future multi-bit error accounting. */
volatile uint16_t rx_errors_detected = 0;


/* ---------------------------------------------------------------------------
   find_minimal_w
   ---------------------------------------------------------------------------
   Returns the unique minimum-Hamming-weight 15-bit vector w such that
       H * w^T  =  s_target,
   exploiting the H1-type matrix structure.

   H1 column structure:
     Column i (1-indexed, i = 1 .. 15) of H is the 4-bit binary value i.
     Consequence: setting bit (i-1) of a 15-bit word produces syndrome i.

   Weight-0 case (s_target = 0):
     The zero syndrome corresponds to the all-zeros transition, meaning no
     bit needs to change.  Return 0.

   Weight-1 case (s_target in {1 .. 15}):
     The single bit at position (s_target - 1) maps to column s_target of
     H, whose value is s_target itself.  So:
         H * (1 << (s_target - 1))^T  =  s_target  = s_target.  QED.
     This weight-1 solution is unique: no two single-bit vectors share the
     same syndrome because each column of H is a distinct non-zero value.
   ---------------------------------------------------------------------------
*/
uint16_t find_minimal_w(uint8_t s_target)
{
    s_target &= 0x0F;   /* Guard: keep only the 4-bit syndrome */

    if (s_target == 0)
    {
        return 0;       /* No transition required */
    }

    /*
       Set exactly one bit at zero-based position (s_target - 1).
       Cast to uint16_t before shifting to avoid undefined behaviour on
       8-bit platforms where (uint8_t)(1 << n) would truncate for n >= 8.
    */
    return ((uint16_t)1u << (s_target - 1));
}


/* ---------------------------------------------------------------------------
   rx_correct_bus_state
   ---------------------------------------------------------------------------
   Single-bit error correction using the TX encoder's weight invariant.
   Full mathematical justification is in rx_ecc.h.
   ---------------------------------------------------------------------------
*/
uint16_t rx_correct_bus_state(uint16_t raw_bus_state)
{
    uint16_t delta;           /* Observed transition from previous state */
    uint8_t  syndrome_delta;  /* Syndrome of that transition */
    uint16_t expected_delta;  /* What the TX encoder would have produced */
    uint16_t corrected_state;

    raw_bus_state &= BUS_STATE_MASK;   /* Strip any noise above bit 14 */

    /* Step 1: Compute the observed transition vector.
       XOR cancels all bits that are the same in both states, leaving
       a 1 only where the two states differ. */
    delta = raw_bus_state ^ rx_previous_bus_state;

    /* Step 2: Compute the syndrome of the observed transition.
       H * delta^T  tells us which column of H is "responsible" for this
       transition.  For a clean channel delta has weight 0 or 1, and
       syndrome_delta equals the TX encoder's intended syndrome_target.
       A single-bit channel error corrupts delta but the syndrome still
       points to the nearest valid weight-1 transition. */
    syndrome_delta = rx_compute_syndrome(delta);

    /* Step 3: Reconstruct the intended (minimal-weight) transition.
       find_minimal_w maps the syndrome back to the canonical weight-0 or
       weight-1 vector that the TX encoder would have XOR'd in. */
    expected_delta = find_minimal_w(syndrome_delta);

    /* Step 4: Accept or correct.
       If what we received matches what the encoder sent, no error occurred.
       Otherwise, reconstruct the corrected state by applying the expected
       transition to the last known-good state, discarding the corrupted
       raw reading entirely. */
    if (delta == expected_delta)
    {
        corrected_state = raw_bus_state;
    }
    else
    {
        corrected_state = (rx_previous_bus_state ^ expected_delta)
                          & BUS_STATE_MASK;
        rx_errors_corrected++;
    }

    /* Update the differential baseline for the next call. */
    rx_previous_bus_state = corrected_state;

    return corrected_state;
}
