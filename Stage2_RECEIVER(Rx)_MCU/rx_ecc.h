/* =============================================================================
   File: rx_ecc.h
   Module: Group 3 — Error Correction
   Public interface and extern declarations.

   This module is self-contained: its only dependency is rx_types.h.
   It does NOT include any hardware headers and does NOT call into the
   decoder or hardware modules, making it independently unit-testable
   on any host platform.
   =============================================================================
*/

#ifndef RX_ECC_H
#define RX_ECC_H

#include "rx_types.h"

/* ---------------------------------------------------------------------------
   Module state — defined in rx_ecc.c, exposed for monitoring only.

   rx_previous_bus_state
     The last bus state accepted as correct.  Initialised to 0 to match
     the TX encoder's startup state (current_bus_state = 0 before the
     first nibble is transmitted).  Updated on every call to
     rx_correct_bus_state() so the differential error detector always
     works relative to the most recent known-good state.

   rx_errors_corrected
     Running count of single-bit errors that were silently repaired.

   rx_errors_detected
     Placeholder for future multi-bit / uncorrectable error accounting.
     Currently unused; kept for diagnostic extensibility.
   ---------------------------------------------------------------------------
*/
extern volatile uint16_t rx_previous_bus_state;
extern volatile uint16_t rx_errors_corrected;
extern volatile uint16_t rx_errors_detected;

/* ---------------------------------------------------------------------------
   find_minimal_w
   ---------------------------------------------------------------------------
   Returns the unique minimum-Hamming-weight 15-bit vector w such that
       H * w^T  =  s_target
   where H is the H1-type matrix defined in rx_types.h.

   Derivation:
     Column i (1-indexed) of H equals the binary value i.
     Therefore setting exactly one bit at position (s_target - 1) yields
         syndrome = (s_target - 1 + 1) = s_target.
     No weight-1 vector can achieve a different syndrome, and no
     weight-0 vector can achieve a non-zero syndrome, so this solution
     is both minimal and unique for all s_target in {0 .. 15}.

   Special case s_target = 0:
     The zero syndrome means the bus state is already in a valid codeword;
     no transition is needed, so w = 0 is returned.

   Parameters:
     s_target  4-bit syndrome value (upper nibble bits are masked off).
   Returns:
     15-bit minimal-weight vector w  (exactly one bit set, or zero).
   ---------------------------------------------------------------------------
*/
uint16_t find_minimal_w(uint8_t s_target);

/* ---------------------------------------------------------------------------
   rx_correct_bus_state
   ---------------------------------------------------------------------------
   Applies single-bit error correction to one raw bus reading using a
   stateful differential approach.

   Background — the TX encoder invariant:
     The transmitter updates its bus state with:
         current_bus_state ^= find_minimal_w(target_syndrome)
     This guarantees that any two consecutive bus states differ by AT MOST
     one bit (Hamming weight of the transition is 0 or 1).

   How this receiver exploits that invariant:
     1. Compute delta = raw_bus_state XOR rx_previous_bus_state.
        This is the OBSERVED transition from the last known-good state.
     2. Compute syndrome_delta = H * delta^T.
        For a clean weight-0 or weight-1 transition, syndrome_delta equals
        the syndrome that the TX encoder targeted.
     3. Compute expected_delta = find_minimal_w(syndrome_delta).
        This reconstructs what the TX encoder actually sent.
     4. If delta == expected_delta, no bit was flipped; pass through.
        If delta != expected_delta, exactly one bit was corrupted in transit.
        The corrected state is:  rx_previous_bus_state XOR expected_delta.

   Why this works:
     A single-bit channel error flips one bit of delta, giving it Hamming
     weight 2 or 0 instead of 0 or 1.  Re-deriving the expected transition
     from the syndrome of the corrupted delta recovers the intended delta,
     because syndrome_delta is the same whether one bit is corrupted in the
     final state or in the transition vector (XOR is linear).

   Limitation:
     Two or more simultaneous bit errors produce an uncorrectable
     (but usually detectable) condition.  The function currently falls back
     to using the expected delta, which may or may not be the right state.

   Parameters:
     raw_bus_state  15-bit value read directly from the physical bus.
   Returns:
     15-bit corrected bus state; also updates rx_previous_bus_state.
   ---------------------------------------------------------------------------
*/
uint16_t rx_correct_bus_state(uint16_t raw_bus_state);

#endif /* RX_ECC_H */
