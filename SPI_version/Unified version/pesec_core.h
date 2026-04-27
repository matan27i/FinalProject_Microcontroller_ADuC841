/* =============================================================================
   File: pesec_core.h
   Module: Shared PESEC Error-Correction Core (DRY refactor)
   =============================================================================

   Purpose:
     Single authoritative location for every PESEC definition, constant,
     data structure, and algorithmic primitive shared between the TX
     encoder and the RX decoder.  Both sides include this header and
     link against pesec_core.c, eliminating duplicated logic.

   Memory placement:
     The PESEC matrices and metadata can live in either DATA (TX) or
     XDATA (RX) memory.  Define PESEC_USE_XDATA before including this
     header (or via compiler flag -DPESEC_USE_XDATA) to place all
     module-owned storage in XDATA.  If left undefined the default
     memory space is used.

   Portability:
     This file contains NO hardware-specific includes.  When porting to
     a platform that supplies <stdint.h> (ANSI C99+), delete the two
     typedefs below and replace them with:
         #include <stdint.h>
   =============================================================================
*/

#ifndef PESEC_CORE_H
#define PESEC_CORE_H

/* =========================================================================
   PORTABLE INTEGER TYPES  (substitute for <stdint.h> on Keil C51)
   =========================================================================
   Guarded so that an enclosing header (tx_header.h / rx_types.h) can
   provide its own definitions without conflict.
   ========================================================================= */
#ifndef UINT8_T_DEFINED
#define UINT8_T_DEFINED
typedef unsigned char  uint8_t;
#endif

#ifndef UINT16_T_DEFINED
#define UINT16_T_DEFINED
typedef unsigned int   uint16_t;
#endif

/* =========================================================================
   MEMORY-SPACE QUALIFIER
   =========================================================================
   Define PESEC_USE_XDATA (e.g. -DPESEC_USE_XDATA on the compiler
   command line, or #define before the first #include) to place all
   module-owned arrays and metadata into the 8051 XDATA segment.
   Otherwise they reside in the default DATA segment.
   ========================================================================= */
#ifdef PESEC_USE_XDATA
  #define PESEC_MEM xdata
#else
  #define PESEC_MEM
#endif

/* =========================================================================
   H1-TYPE HAMMING BUS GEOMETRY
   ========================================================================= */
#define HAMMING_R        4                                 /* Syndrome width (m)   */
#define HAMMING_N        ((1 << HAMMING_R) - 1)            /* Bus width 2^R-1 = 15 */
#define BUS_STATE_MASK   ((uint16_t)0x7FFFu)               /* Bits [14:0]          */

/* =========================================================================
   PESEC ERROR-CORRECTION PARAMETERS
   ========================================================================= */
#define PESEC_RED_BITS      10
#define PESEC_RED_MASK      ((uint16_t)0x03FFu)            /* 10 redundancy bits   */
#define PESEC_TOTAL_BITS    (HAMMING_N + PESEC_RED_BITS + 1) /* 26: +1 parity      */
#define MAX_PESEC_BLOCKS    5                              /* Max H1-type sub-blocks */

/* =========================================================================
   PESEC CORRECTION RESULT CODES  (returned by rx_pesec_correct)
   =========================================================================
   [W4] Includes PESEC_CORRECTED_PARITY for the case where only the
   overall parity bit itself flipped (syndrome==0, parity mismatch).
   ========================================================================= */
#define PESEC_NO_ERROR          0  /* Syndrome zero, parity OK: no error       */
#define PESEC_CORRECTED_DATA    1  /* Single-bit error corrected in data       */
#define PESEC_CORRECTED_RED     2  /* Single-bit error corrected in redundancy */
#define PESEC_UNCORRECTABLE     3  /* Multi-bit error, correction failed       */
#define PESEC_CORRECTED_PARITY  4  /* [W4] Parity bit itself was flipped       */

/* =========================================================================
   PESEC MATRIX STORAGE AND BLOCK-SOLVER METADATA
   =========================================================================
   Defined in pesec_core.c.  Built once at startup by
   pesec_init_matrices(); thereafter read-only.
   ========================================================================= */
extern uint8_t PESEC_MEM PESEC_MAT_D[20];   /* Redundancy-bit columns */
extern uint8_t PESEC_MEM PESEC_MAT_A[40];   /* Data-bit columns       */

extern uint8_t PESEC_MEM pesec_num_blocks;
extern uint8_t PESEC_MEM pesec_bit_offsets[MAX_PESEC_BLOCKS];
extern uint8_t PESEC_MEM pesec_col_offsets[MAX_PESEC_BLOCKS];
extern uint8_t PESEC_MEM pesec_chunk_masks[MAX_PESEC_BLOCKS];

extern uint8_t PESEC_MEM pesec_num_d_cols;
extern uint8_t PESEC_MEM pesec_num_a_cols;

/* =========================================================================
   SHARED FUNCTION PROTOTYPES
   ========================================================================= */

/* ---------------------------------------------------------------------------
   pesec_init_matrices
   ---------------------------------------------------------------------------
   Builds the PESEC parity-check sub-matrices D (redundancy columns)
   and A (data columns) from the supplied block-size configuration.
   Must be called once at startup before any encode/decode operation.

   Parameters:
     m_sizes     Array of sub-block bit widths (e.g. {3, 2}).
     num_blocks  Number of entries in m_sizes.
   ---------------------------------------------------------------------------
*/
void pesec_init_matrices(uint8_t *m_sizes, uint8_t num_blocks);

/* ---------------------------------------------------------------------------
   Syndrome computation over data bits (Matrix A) and redundancy bits
   (Matrix D).  These are the core building blocks for both TX encoding
   and RX error correction.
   ---------------------------------------------------------------------------
*/
uint8_t get_data_syndrome(uint16_t data_val);
uint8_t get_red_syndrome(uint16_t red_val);

/* ---------------------------------------------------------------------------
   calculate_overall_parity
   ---------------------------------------------------------------------------
   Computes the population parity (popcount mod 2) of a 16-bit value
   using branch-free XOR folding optimised for the 8051 instruction set.

   Returns: 0 if even number of set bits, 1 if odd.
   ---------------------------------------------------------------------------
*/
uint8_t calculate_overall_parity(uint16_t v);

#endif /* PESEC_CORE_H */
