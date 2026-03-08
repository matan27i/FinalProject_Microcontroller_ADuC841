#include <aduc841.h>
#include "header.h"


/* Init_PESEC_Matrices
  Generates Matrix D and Matrix A dynamically based on an array of block sizes.
*/
/* Dynamic PESEC Variables */
#define MAX_PESEC_BLOCKS 5

volatile uint16_t pesec_redundancy_reg = 0;

/* Arrays to store dynamic matrices (Allocated ONLY HERE) */
uint8_t PESEC_MAT_D[20]; 
uint8_t PESEC_MAT_A[40]; 

/* Block management arrays for O(1) solving */
uint8_t pesec_num_blocks;
uint8_t pesec_bit_offsets[MAX_PESEC_BLOCKS];
uint8_t pesec_col_offsets[MAX_PESEC_BLOCKS];
uint8_t pesec_chunk_masks[MAX_PESEC_BLOCKS];

uint8_t pesec_num_d_cols;
uint8_t pesec_num_a_cols;


void Init_PESEC_Matrices(uint8_t* m_sizes, uint8_t num_blocks) 
{
    uint8_t i, b, val;
    uint8_t is_in_d;
    uint8_t total_m = 0;
    uint8_t current_col_offset = 0;
    uint8_t max_syndrome;
    
    pesec_num_blocks = num_blocks;
    pesec_num_d_cols = 0;
    pesec_num_a_cols = 0;
    
    /* --- Build Matrix D and Pre-calculate Solver Data --- */
    for (b = 0; b < num_blocks; b++) 
    {
        /* Store offsets and masks for fast delta solving */
        pesec_bit_offsets[b] = total_m;
        pesec_col_offsets[b] = current_col_offset;
        pesec_chunk_masks[b] = ((1 << m_sizes[b]) - 1) << total_m;
        
        /* Build Matrix D block columns */
        for (i = 1; i < (1 << m_sizes[b]); i++) 
        {
            PESEC_MAT_D[pesec_num_d_cols++] = (i << total_m);
        }
        
        /* Advance global offsets for the next block */
        total_m += m_sizes[b];
        current_col_offset += ((1 << m_sizes[b]) - 1);
    }
    
    max_syndrome = (1 << total_m) - 1;
    
    /* --- Build Matrix A (Everything not in D) --- */
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