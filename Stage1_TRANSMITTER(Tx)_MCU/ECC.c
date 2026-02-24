/* ECC (Error Correcting Code)
  Implements the PESEC Encoder block.
 */
 
#include <aduc841.h>
#include "header.h"



/* Generic function to compute syndrome for a given matrix */
uint8_t calc_pesec_synd(uint16_t reg_val, uint8_t* mat_ptr, uint8_t cols) 
{
    uint8_t synd_res = 0;
    uint8_t idx;
    
    for (idx = 0; idx < cols; idx++) 
    {
        if ((reg_val >> idx) & 1) 
        {
            synd_res ^= mat_ptr[idx];
        }
    }
    return synd_res;
}

/* Wrapper for the data bits (Matrix A) */
uint8_t get_data_pesec_synd(uint16_t data_val) 
{
    /* Using dynamic column count instead of fixed 15 */
    return calc_pesec_synd(data_val, PESEC_MAT_A, pesec_num_a_cols);
}

/* Wrapper for the redundancy bits (Matrix D) */
uint8_t get_red_pesec_synd(uint16_t red_val) 
{
    /* Using dynamic column count instead of fixed 10 */
    return calc_pesec_synd(red_val, PESEC_MAT_D, pesec_num_d_cols);
}

/* Fast solver that adapts to any number of blocks */
uint16_t solve_pesec_delta(uint8_t synd_gap) 
{
    uint16_t delta_vec = 0;
    uint8_t b, chunk, val;
    
    /* Iterate through each H1-type block */
    for (b = 0; b < pesec_num_blocks; b++) 
    {
        /* Extract the syndrome chunk for this specific block */
        chunk = synd_gap & pesec_chunk_masks[b];
        
        if (chunk != 0) 
        {
            /* Shift down to base value */
            val = chunk >> pesec_bit_offsets[b];
            
            /* Flip the correct bit in the physical redundancy register */
            delta_vec |= (1 << (pesec_col_offsets[b] + val - 1));
        }
    }
    
    return delta_vec;
}

/* Main ECC Function */
void ECC(void) 
{
    uint8_t synd_data;
    uint8_t synd_red;
    uint8_t synd_diff;
    uint16_t delta_vector;

    /* Step 1: Calculate syndrome of the new data */
    synd_data = get_data_pesec_synd(current_bus_state);
    
    /* Step 2: Calculate syndrome of the current redundancy bits */
    synd_red = get_red_pesec_synd(pesec_redundancy_reg);
    
    /* Step 3: Find the difference */
    synd_diff = synd_data ^ synd_red;
    
    /* Step 4: Solve for the optimal hardware flip */
    delta_vector = solve_pesec_delta(synd_diff);
    
    /* Step 5: Apply the differential update */
    pesec_redundancy_reg ^= delta_vector;
    
    /* Step 6: Output all 25 bits to the daisy-chained registers */
    output_to_shift_registers();
}