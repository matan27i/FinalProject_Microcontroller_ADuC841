/*
   File: rx_ecc.c
   Module: Group 3  Error Correction (PESEC + H1 + SEC+DED)

   Implementation of:
     - PESEC matrix construction (mirroring the TX encoder)
     - PESEC syndrome computation and single-bit error correction
     - SEC+DED overall parity check for double-error detection
     - H1 differential state tracking (secondary validation layer)

   SEC+DED UPGRADE:
     rx_pesec_correct() now accepts an overall_parity parameter.  The
     TX computes P = parity(data[14:0]) XOR parity(red[9:0]) and shifts
     it out as the 26th bus bit.  The RX recomputes parity locally and
     compares it with the received P.  Combined with the PESEC syndrome,
     this yields a 4-case truth table:

       Syndrome | Parity Check | Meaning
       ---------|--------------|------------------------------------------
         == 0   |     OK       | No error (PESEC_NO_ERROR)
         != 0   |     FAIL     | Single error -> correct (PESEC_CORRECTED_*)
         != 0   |     OK       | Double error -> uncorrectable (PESEC_UNCORRECTABLE)
         == 0   |     FAIL     | Parity bit itself flipped (PESEC_CORRECTED_PARITY)

     "Parity check OK" means: received_P XOR parity(received_25) == 0.
     "Parity check FAIL" means the overall parity is odd, indicating an
     odd number of bit flips (1 flip = correctable single error).

   Dependencies:
     rx_decoder.h  ->  rx_compute_syndrome()
     rx_types.h    ->  uint8_t, uint16_t, constants

   Portability:
     No hardware registers are accessed here.
   
*/

#include "rx_system.h"

/* 
   MODULE STATE VARIABLES
  */

/* --- PESEC matrices (built at startup, read-only thereafter) ---
   [MEM] Placed in XDATA to free DATA space. */
uint8_t xdata PESEC_MAT_D[20];
uint8_t xdata PESEC_MAT_A[40];

/* --- Block solver metadata ---
   [MEM] XDATA. */
uint8_t xdata pesec_num_blocks;
uint8_t xdata pesec_bit_offsets[MAX_PESEC_BLOCKS];
uint8_t xdata pesec_col_offsets[MAX_PESEC_BLOCKS];
uint8_t xdata pesec_chunk_masks[MAX_PESEC_BLOCKS];

/* --- Column counts --- */
uint8_t xdata pesec_num_d_cols;
uint8_t xdata pesec_num_a_cols;

/* --- H1 differential state ---
   [MEM] XDATA. */
uint16_t xdata rx_previous_bus_state = 0;
uint16_t xdata rx_errors_corrected   = 0;
uint16_t xdata rx_errors_detected    = 0;

/* --- PESEC correction statistics --- */
uint16_t xdata rx_pesec_corrections  = 0;


/* 
   PESEC MATRIX INITIALISATION
    */
void rx_init_pesec_matrices(uint8_t *m_sizes, uint8_t num_blocks)
{
    uint8_t i, b, val;
    uint8_t is_in_d;
    uint8_t total_m = 0;
    uint8_t current_col_offset = 0;
    uint8_t max_syndrome;

    pesec_num_blocks = num_blocks;
    pesec_num_d_cols = 0;
    pesec_num_a_cols = 0;

    for (b = 0; b < num_blocks; b++)
    {
        pesec_bit_offsets[b] = total_m;
        pesec_col_offsets[b] = current_col_offset;
        pesec_chunk_masks[b] = ((1 << m_sizes[b]) - 1) << total_m;

        for (i = 1; i < (1 << m_sizes[b]); i++)
        {
            PESEC_MAT_D[pesec_num_d_cols++] = (i << total_m);
        }

        total_m += m_sizes[b];
        current_col_offset += ((1 << m_sizes[b]) - 1);
    }

    max_syndrome = (1 << total_m) - 1;

    for (val = 1; val <= max_syndrome; val++)
    {
        is_in_d = 0;

        for (i = 0; i < pesec_num_d_cols; i++)
        {
            if (PESEC_MAT_D[i] == val)
            {
                is_in_d = 1;
                break;
            }
        }

        if (is_in_d == 0)
        {
            PESEC_MAT_A[pesec_num_a_cols++] = val;
        }
    }
}


/* 
   PESEC SYNDROME COMPUTATION (INTERNAL HELPERS)
    */

static uint8_t calc_pesec_syndrome_generic(uint16_t reg_val,
                                           uint8_t *mat_ptr,
                                           uint8_t  num_cols)
{
    uint8_t syndrome = 0;
    uint8_t idx;

    for (idx = 0; idx < num_cols; idx++)
    {
        if ((reg_val >> idx) & 1u)
        {
            syndrome ^= mat_ptr[idx];
        }
    }

    return syndrome;
}


static uint8_t get_data_syndrome(uint16_t data_val)
{
    return calc_pesec_syndrome_generic(data_val, PESEC_MAT_A,
                                       pesec_num_a_cols);
}


static uint8_t get_red_syndrome(uint16_t red_val)
{
    return calc_pesec_syndrome_generic(red_val, PESEC_MAT_D,
                                       pesec_num_d_cols);
}


/* 
   PARITY COMPUTATION HELPER
   
   Computes the population parity (popcount mod 2) of a 16-bit value
   using XOR folding.  Identical to the TX encoder's implementation.

   Returns: 0 if even number of set bits, 1 if odd.
    */
static uint8_t parity16(uint16_t v)
{
    uint8_t x;

    x = (uint8_t)(v >> 8) ^ (uint8_t)(v);
    x ^= x >> 4;
    x ^= x >> 2;
    x ^= x >> 1;

    return (x & 1u);
}


/* 
   PESEC ERROR CORRECTION  (SEC+DED UPGRADED)
   

   [W4] UPGRADED FROM SEC TO SEC+DED

   The original SEC-only algorithm:
     1. Compute syndrome S.
     2. If S == 0: no error.
     3. If S != 0: search for matching column, correct if found.

   The SEC+DED algorithm adds an overall parity check to distinguish
   single errors (correctable) from double errors (detectable only):

     1. Compute PESEC syndrome S.
     2. Recompute the overall parity of the received 25 data+red bits.
     3. Compare recomputed parity with the received overall parity bit.
     4. Apply the 4-case truth table:

        CASE 1: S == 0, parity OK
          The codeword is valid.  No error occurred.
          Return: PESEC_NO_ERROR.

        CASE 2: S != 0, parity FAIL (odd total parity)
          An odd number of bits flipped.  For practical channel error
          rates, this is overwhelmingly a SINGLE-bit error.  The PESEC
          syndrome S identifies the errored bit position.  Proceed with
          standard column-matching correction.
          Return: PESEC_CORRECTED_DATA or PESEC_CORRECTED_RED.

        CASE 3: S != 0, parity OK (even total parity)
          An even number of bits flipped (2, 4, ...).  The PESEC syndrome
          is non-zero because the two (or more) errors do not cancel in
          syndrome space, but the overall parity IS even because two flips
          preserve parity.  This is a DOUBLE (or higher even) error that
          PESEC cannot correct -- applying the syndrome would "correct"
          to the wrong codeword.
          Return: PESEC_UNCORRECTABLE.

        CASE 4: S == 0, parity FAIL (odd total parity)
          The syndrome is zero, meaning all 25 data+red bits form a valid
          PESEC codeword.  But the parity is wrong, so exactly one bit
          flipped -- and that bit is the overall parity bit itself.
          The data and redundancy are clean; no correction needed.
          Return: PESEC_CORRECTED_PARITY.

   Parameters:
     data_bits       Pointer to 15-bit data; corrected in place if needed.
     red_bits        Pointer to 10-bit redundancy; corrected in place.
     overall_parity  The received overall parity bit (0 or 1) from the
                     26th bus bit, as extracted by the SPI ISR.
   */
uint8_t rx_pesec_correct(uint16_t *data_bits, uint16_t *red_bits,
                         uint8_t overall_parity)
{
    uint8_t syndrome;
    uint8_t parity_check;
    uint8_t i;
    uint8_t search_limit;

    /* Step 1: Compute full PESEC syndrome over the received 25 bits. */
    syndrome = get_data_syndrome(*data_bits) ^ get_red_syndrome(*red_bits);

    /* Step 2: Recompute parity of the received 25 data+redundancy bits
       and compare with the received overall parity bit.
       parity_check = 0 means parity is consistent (even total).
       parity_check = 1 means parity mismatch (odd total = error). */
    parity_check = parity16(*data_bits & BUS_STATE_MASK)
                 ^ parity16(*red_bits & PESEC_RED_MASK)
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


/* 
   H1 DIFFERENTIAL ERROR CORRECTION  (unchanged from previous version)
    */

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
