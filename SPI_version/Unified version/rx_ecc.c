/* =============================================================================
   File: rx_ecc.c
   Module: Group 3 -- RX-Specific Error Correction (SEC+DED + H1 Differential)
   =============================================================================

   Implementation of:
     - [W4] SEC+DED error correction using shared PESEC syndromes + parity
     - H1 differential state tracking (secondary validation layer)

   DRY REFACTOR:
     Matrix construction, syndrome computation, and parity calculation
     have been extracted into the shared pesec_core module.  This file
     now calls pesec_init_matrices(), get_data_syndrome(),
     get_red_syndrome(), and calculate_overall_parity() from pesec_core.c.

   SEC+DED UPGRADE (W4):
     rx_pesec_correct() accepts an overall_parity parameter.  The TX
     computes P = parity(data[14:0]) XOR parity(red[9:0]) and shifts
     it out as the 26th bus bit.  The RX recomputes parity locally and
     compares it with the received P.  Combined with the PESEC syndrome,
     this yields a 4-case truth table (see pesec_core.h for result codes).

   Dependencies:
     pesec_core.h  ->  shared matrices, syndromes, parity, constants
     rx_decoder.h  ->  rx_compute_syndrome()

   Portability:
     No hardware registers are accessed here.
   =============================================================================
*/

#include "rx_system.h"

/* =========================================================================
   RX-SPECIFIC STATE VARIABLES
   =========================================================================
   [MEM] Placed in XDATA via PESEC_MEM (PESEC_USE_XDATA defined in
   the RX build) to free DATA space.
   ========================================================================= */

/* H1 differential state */
uint16_t PESEC_MEM rx_previous_bus_state = 0;
uint16_t PESEC_MEM rx_errors_corrected   = 0;
uint16_t PESEC_MEM rx_errors_detected    = 0;

/* PESEC correction statistics */
uint16_t PESEC_MEM rx_pesec_corrections  = 0;


/* =========================================================================
   PESEC ERROR CORRECTION  (SEC+DED UPGRADED)
   =========================================================================

   [W4] UPGRADED FROM SEC TO SEC+DED

   Uses the shared get_data_syndrome(), get_red_syndrome(), and
   calculate_overall_parity() from pesec_core.c.

   The SEC+DED algorithm:

     1. Compute PESEC syndrome S via the shared syndrome functions.
     2. Recompute the overall parity of the received 25 data+red bits
        using the shared calculate_overall_parity().
     3. Compare recomputed parity with the received overall parity bit.
     4. Apply the 4-case truth table:

        CASE 1: S == 0, parity OK
          The codeword is valid.  No error occurred.
          Return: PESEC_NO_ERROR.

        CASE 2: S != 0, parity FAIL (odd total parity)
          Single-bit error.  The PESEC syndrome S identifies the errored
          bit position.  Proceed with column-matching correction.
          Return: PESEC_CORRECTED_DATA or PESEC_CORRECTED_RED.

        CASE 3: S != 0, parity OK (even total parity)
          Double (or even-count) error.  Cannot correct.
          Return: PESEC_UNCORRECTABLE.

        CASE 4: S == 0, parity FAIL (odd total parity)
          The parity bit itself flipped.  Data and redundancy are clean.
          Return: PESEC_CORRECTED_PARITY.

   Parameters:
     data_bits       Pointer to 15-bit data; corrected in place if needed.
     red_bits        Pointer to 10-bit redundancy; corrected in place.
     overall_parity  The received overall parity bit (0 or 1) from the
                     26th bus bit, as extracted by the SPI ISR.
   ========================================================================= */
uint8_t rx_pesec_correct(uint16_t *data_bits, uint16_t *red_bits,
                         uint8_t overall_parity)
{
    uint8_t syndrome;
    uint8_t parity_check;
    uint8_t i;
    uint8_t search_limit;

    /* Step 1: Compute full PESEC syndrome over the received 25 bits.
       Uses shared get_data_syndrome() and get_red_syndrome(). */
    syndrome = get_data_syndrome(*data_bits) ^ get_red_syndrome(*red_bits);

    /* Step 2: Recompute parity of the received 25 data+redundancy bits
       using the shared calculate_overall_parity(), and compare with the
       received overall parity bit.
       parity_check = 0 means parity is consistent (even total).
       parity_check = 1 means parity mismatch (odd total = error). */
    parity_check = calculate_overall_parity(*data_bits & BUS_STATE_MASK)
                 ^ calculate_overall_parity(*red_bits & PESEC_RED_MASK)
                 ^ (overall_parity & 0x01u);

    /* Step 3: Apply the SEC+DED truth table. */

    if (syndrome == 0)
    {
        if (parity_check == 0)
        {
            /* CASE 1: S==0, parity OK -> no error. */
            return PESEC_NO_ERROR;
        }
        else
        {
            /* CASE 4: S==0, parity FAIL -> only the parity bit flipped.
               Data and redundancy are intact.  Increment the correction
               counter since a bit error was detected and accounted for. */
            rx_pesec_corrections++;
            return PESEC_CORRECTED_PARITY;
        }
    }
    else
    {
        if (parity_check == 0)
        {
            /* CASE 3: S!=0, parity OK -> double (or even-count) error.
               The syndrome points to a wrong position; correcting would
               make things worse.  Flag as uncorrectable. */
            rx_errors_detected++;
            return PESEC_UNCORRECTABLE;
        }

        /* CASE 2: S!=0, parity FAIL -> single error.  Proceed with
           standard PESEC column-matching correction. */
    }

    /* --- Single-error correction (Case 2 only reaches here) --- */

    /* Search data-bit columns (Matrix A).
       Only check the first HAMMING_N columns (physical data bits). */
    search_limit = pesec_num_a_cols;
    if (search_limit > HAMMING_N)
    {
        search_limit = HAMMING_N;
    }

    for (i = 0; i < search_limit; i++)
    {
        if (PESEC_MAT_A[i] == syndrome)
        {
            *data_bits ^= ((uint16_t)1u << i);
            *data_bits &= BUS_STATE_MASK;
            rx_pesec_corrections++;
            return PESEC_CORRECTED_DATA;
        }
    }

    /* Search redundancy-bit columns (Matrix D). */
    for (i = 0; i < pesec_num_d_cols; i++)
    {
        if (PESEC_MAT_D[i] == syndrome)
        {
            *red_bits ^= ((uint16_t)1u << i);
            *red_bits &= PESEC_RED_MASK;
            rx_pesec_corrections++;
            return PESEC_CORRECTED_RED;
        }
    }

    /* Syndrome is non-zero, parity says single error, but no column
       matches.  This should not happen with the H1-type construction
       (every non-zero 5-bit value is a column), but guard against it. */
    rx_errors_detected++;
    return PESEC_UNCORRECTABLE;
}


/* =========================================================================
   H1 DIFFERENTIAL ERROR CORRECTION  (unchanged from previous version)
   ========================================================================= */

uint16_t find_minimal_w(uint8_t s_target)
{
    s_target &= 0x0F;

    if (s_target == 0)
    {
        return 0;
    }

    return ((uint16_t)1u << (s_target - 1));
}


uint16_t rx_correct_bus_state(uint16_t corrected_data)
{
    uint16_t delta;
    uint8_t  syndrome_delta;
    uint16_t expected_delta;
    uint16_t result;

    corrected_data &= BUS_STATE_MASK;

    delta = corrected_data ^ rx_previous_bus_state;

    syndrome_delta = rx_compute_syndrome(delta);

    expected_delta = find_minimal_w(syndrome_delta);

    if (delta == expected_delta)
    {
        result = corrected_data;
    }
    else
    {
        result = (rx_previous_bus_state ^ expected_delta) & BUS_STATE_MASK;
        rx_errors_corrected++;
    }

    rx_previous_bus_state = result;

    return result;
}
