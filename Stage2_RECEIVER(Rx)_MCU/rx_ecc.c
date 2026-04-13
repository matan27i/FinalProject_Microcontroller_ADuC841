/* =============================================================================
   File: rx_ecc.c
   Module: Group 3 -- Error Correction (PESEC + H1)

   Implementation of:
     - PESEC matrix construction (mirroring the TX encoder)
     - PESEC syndrome computation and single-bit error correction
     - H1 differential state tracking (secondary validation layer)

   Dependencies:
     rx_decoder.h  ->  rx_compute_syndrome()  (H1 matrix multiply)
     rx_types.h    ->  uint8_t, uint16_t, constants

   Portability:
     No hardware registers are accessed here.  This entire module can be
     compiled and unit-tested on any ISO C89 host.
   =============================================================================
*/

#include "rx_system.h"

/* =========================================================================
   MODULE STATE VARIABLES
   ========================================================================= */

/* --- PESEC matrices (built at startup, read-only thereafter) --- */
uint8_t PESEC_MAT_D[20];       /* Redundancy-bit columns         */
uint8_t PESEC_MAT_A[40];       /* Data-bit columns               */

/* --- Block solver metadata --- */
uint8_t pesec_num_blocks;
uint8_t pesec_bit_offsets[MAX_PESEC_BLOCKS];
uint8_t pesec_col_offsets[MAX_PESEC_BLOCKS];
uint8_t pesec_chunk_masks[MAX_PESEC_BLOCKS];

/* --- Column counts --- */
uint8_t pesec_num_d_cols;
uint8_t pesec_num_a_cols;

/* --- H1 differential state ---
   [N1] volatile removed: no ISRs access these; main-loop only. */
uint16_t rx_previous_bus_state = 0;
uint16_t rx_errors_corrected   = 0;
uint16_t rx_errors_detected    = 0;

/* --- PESEC correction statistics --- */
uint16_t rx_pesec_corrections  = 0;


/* =========================================================================
   PESEC MATRIX INITIALISATION
   =========================================================================
   This function is a direct port of the TX encoder's Init_PESEC_Matrices().
   It must be called with the SAME block-size configuration so that the
   receiver's matrices match the transmitter's exactly.

   For config {m1=3, m2=2} the construction proceeds as follows:

   Block 0 (m=3, total_m starts at 0):
     Columns: i << 0 for i = 1..7  ->  D = {1, 2, 3, 4, 5, 6, 7}
     chunk_mask = ((1<<3)-1) << 0 = 0x07
     bit_offset = 0, col_offset = 0
     total_m advances to 3

   Block 1 (m=2, total_m = 3):
     Columns: i << 3 for i = 1..3  ->  D += {8, 16, 24}
     chunk_mask = ((1<<2)-1) << 3 = 0x18
     bit_offset = 3, col_offset = 7
     total_m advances to 5

   max_syndrome = 2^5 - 1 = 31

   Matrix A = {val in 1..31 : val NOT in D}
            = {9,10,11,12,13,14,15, 17,18,19,20,21,22,23, 25,26,27,28,29,30,31}
            = 21 columns

   Only A[0]..A[14] correspond to physical data bits 0..14.
   A[15]..A[20] map to virtual positions that are always zero in the
   15-bit bus state, so they never contribute to the syndrome and never
   need correction.
   ========================================================================= */
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

    /* --- Build Matrix D and pre-calculate block solver metadata --- */
    for (b = 0; b < num_blocks; b++)
    {
        pesec_bit_offsets[b] = total_m;
        pesec_col_offsets[b] = current_col_offset;
        pesec_chunk_masks[b] = ((1 << m_sizes[b]) - 1) << total_m;

        /* Each block contributes columns 1..(2^m - 1) shifted by total_m.
           These are the syndrome-space values for the redundancy bits
           belonging to this block. */
        for (i = 1; i < (1 << m_sizes[b]); i++)
        {
            PESEC_MAT_D[pesec_num_d_cols++] = (i << total_m);
        }

        total_m += m_sizes[b];
        current_col_offset += ((1 << m_sizes[b]) - 1);
    }

    max_syndrome = (1 << total_m) - 1;

    /* --- Build Matrix A: syndrome-space values NOT present in D ---
       These become the columns for the data bits.  The first HAMMING_N
       columns map to physical bus bits 0..14. */
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


/* =========================================================================
   PESEC SYNDROME COMPUTATION (INTERNAL HELPERS)
   =========================================================================
   These are static helpers that compute partial syndromes by XOR-ing
   together the matrix columns selected by set bits in a register value.

   Identical in structure to the TX encoder's syndrome helpers, ensuring
   mathematical equivalence.
   ========================================================================= */

/* ---------------------------------------------------------------------------
   calc_pesec_syndrome_generic
   ---------------------------------------------------------------------------
   Generic helper: for each set bit in reg_val, XOR the corresponding
   column from mat_ptr into the accumulator.

   Parameters:
     reg_val    Bit vector selecting columns (bit i selects mat_ptr[i]).
     mat_ptr    Pointer to the column array (either A or D).
     num_cols   Number of columns in the array.
   Returns:
     XOR accumulation of selected columns (the partial syndrome).
   ---------------------------------------------------------------------------
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


/* Syndrome contribution from the 15-bit data vector via Matrix A.
   Only the first min(HAMMING_N, pesec_num_a_cols) bits matter because
   the bus state has at most 15 physical bits. */
static uint8_t get_data_syndrome(uint16_t data_val)
{
    return calc_pesec_syndrome_generic(data_val, PESEC_MAT_A,
                                       pesec_num_a_cols);
}


/* Syndrome contribution from the 10-bit redundancy vector via Matrix D. */
static uint8_t get_red_syndrome(uint16_t red_val)
{
    return calc_pesec_syndrome_generic(red_val, PESEC_MAT_D,
                                       pesec_num_d_cols);
}


/* =========================================================================
   PESEC ERROR CORRECTION
   =========================================================================

   MATHEMATICAL BACKGROUND
   ~~~~~~~~~~~~~~~~~~~~~~~
   The PESEC codeword is the concatenation [data(15) | redundancy(10)].
   The parity-check matrix H_pesec = [A | D] has the property that for
   any valid codeword C:

       H_pesec * C^T  =  A * data^T  XOR  D * redundancy^T  =  0

   When the received word has a single-bit error at position k, the
   syndrome becomes:

       S  =  column_k  (the k-th column of [A | D])

   Because every column of [A | D] is a unique non-zero value in the
   5-bit syndrome space (by construction of the H1-type sub-blocks),
   S uniquely identifies the error position:

     - If S matches A[i] (i < 15): the error is in data bit i.
     - If S matches D[j] (j < 10): the error is in redundancy bit j.

   For multi-bit errors, S will (in general) not match any single column,
   and the function returns PESEC_UNCORRECTABLE.

   CORRECTION PROCEDURE
   ~~~~~~~~~~~~~~~~~~~~
   1. Compute S = get_data_syndrome(data) XOR get_red_syndrome(red).
   2. If S == 0: return PESEC_NO_ERROR.
   3. Search A[0..14] for a match: if found at index i, flip data bit i.
   4. Search D[0..9] for a match: if found at index j, flip red bit j.
   5. No match: return PESEC_UNCORRECTABLE.
   ========================================================================= */
uint8_t rx_pesec_correct(uint16_t *data_bits, uint16_t *red_bits)
{
    uint8_t syndrome;
    uint8_t i;
    uint8_t search_limit;

    /* Step 1: Compute full PESEC syndrome. */
    syndrome = get_data_syndrome(*data_bits) ^ get_red_syndrome(*red_bits);

    /* Step 2: Zero syndrome means valid codeword, no error. */
    if (syndrome == 0)
    {
        return PESEC_NO_ERROR;
    }

    /* Step 3: Search data-bit columns (Matrix A).
       Only check the first HAMMING_N columns because the bus state has
       exactly 15 physical data bits.  Columns beyond index 14 correspond
       to virtual bit positions that cannot be in error. */
    search_limit = pesec_num_a_cols;
    if (search_limit > HAMMING_N)
    {
        search_limit = HAMMING_N;
    }

    for (i = 0; i < search_limit; i++)
    {
        if (PESEC_MAT_A[i] == syndrome)
        {
            /* Error is in data bit i.  Flip it. */
            *data_bits ^= ((uint16_t)1u << i);
            *data_bits &= BUS_STATE_MASK;
            rx_pesec_corrections++;
            return PESEC_CORRECTED_DATA;
        }
    }

    /* Step 4: Search redundancy-bit columns (Matrix D). */
    for (i = 0; i < pesec_num_d_cols; i++)
    {
        if (PESEC_MAT_D[i] == syndrome)
        {
            /* Error is in redundancy bit i.  Flip it. */
            *red_bits ^= ((uint16_t)1u << i);
            *red_bits &= PESEC_RED_MASK;
            rx_pesec_corrections++;
            return PESEC_CORRECTED_RED;
        }
    }

    /* Step 5: Syndrome is non-zero but matches no single column.
       This indicates a multi-bit error that cannot be corrected. */
    rx_errors_detected++;
    return PESEC_UNCORRECTABLE;
}


/* =========================================================================
   H1 DIFFERENTIAL ERROR CORRECTION
   =========================================================================
   This layer operates on the 15-bit data AFTER PESEC correction.
   It exploits the TX encoder's invariant that consecutive bus states
   differ by at most one bit (weight-0 or weight-1 transition).

   After PESEC has corrected any channel errors in the 25-bit word,
   the data bits should be clean.  This layer serves as:
     (a) a secondary validation check,
     (b) the differential baseline tracker for the decode pipeline.

   If PESEC correction was successful, this function will almost always
   pass the data through unchanged.  It only applies a correction if
   the transition somehow has weight > 1 (e.g., PESEC missed a multi-bit
   error, or a synchronisation glitch occurred).
   ========================================================================= */

/* ---------------------------------------------------------------------------
   find_minimal_w
   ---------------------------------------------------------------------------
   H1-type minimum-weight vector for a given syndrome target.
   Identical to the TX encoder's implementation.

   s_target = 0  ->  return 0          (no transition)
   s_target != 0 ->  return 1 << (s-1) (single bit at position s-1)
   ---------------------------------------------------------------------------
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


/* ---------------------------------------------------------------------------
   rx_correct_bus_state
   ---------------------------------------------------------------------------
   Full explanation in rx_ecc.h.  Summary:

   1. delta = corrected_data XOR rx_previous_bus_state
   2. syndrome_delta = H1 * delta^T
   3. expected_delta = find_minimal_w(syndrome_delta)
   4. If delta == expected_delta: accept as-is.
      Else: reconstruct from previous + expected_delta.
   5. Update rx_previous_bus_state.
   ---------------------------------------------------------------------------
*/
uint16_t rx_correct_bus_state(uint16_t corrected_data)
{
    uint16_t delta;
    uint8_t  syndrome_delta;
    uint16_t expected_delta;
    uint16_t result;

    corrected_data &= BUS_STATE_MASK;

    /* Step 1: Observed transition from previous known-good state. */
    delta = corrected_data ^ rx_previous_bus_state;

    /* Step 2: H1 syndrome of the transition.
       rx_compute_syndrome() is defined in rx_decoder.c and performs
       the on-the-fly H * delta^T computation. */
    syndrome_delta = rx_compute_syndrome(delta);

    /* Step 3: Reconstruct the canonical transition the TX would send. */
    expected_delta = find_minimal_w(syndrome_delta);

    /* Step 4: Accept or correct. */
    if (delta == expected_delta)
    {
        result = corrected_data;
    }
    else
    {
        /* The transition weight was not 0 or 1.  This should be rare
           after PESEC correction.  Reconstruct the intended state. */
        result = (rx_previous_bus_state ^ expected_delta) & BUS_STATE_MASK;
        rx_errors_corrected++;
    }

    /* Step 5: Advance the differential baseline. */
    rx_previous_bus_state = result;

    return result;
}
