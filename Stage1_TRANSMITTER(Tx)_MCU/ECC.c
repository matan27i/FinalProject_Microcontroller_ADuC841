/* ECC (Error Correcting Code)
  Implements the PESEC Encoder block.
 */
 
#include <aduc841.h>
#include "header.h"

/* Global variable initialization */
volatile uint16_t pesec_redundancy_reg = 0;

/* Matrix Definitions */
const uint8_t PESEC_MAT_D[10] = {1, 2, 3, 4, 5, 6, 7, 8, 16, 24};
const uint8_t PESEC_MAT_A[15] = {9, 10, 11, 12, 13, 14, 15, 17, 18, 19, 20, 21, 22, 23, 25};

/* Generic function to compute syndrome for a given matrix */
uint8_t calc_pesec_synd(uint16_t reg_val, const uint8_t* mat_ptr, uint8_t cols) 
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
    return calc_pesec_synd(data_val, PESEC_MAT_A, 15);
}

/* Wrapper for the redundancy bits (Matrix D) */
uint8_t get_red_pesec_synd(uint16_t red_val) 
{
    return calc_pesec_synd(red_val, PESEC_MAT_D, 10);
}

/* O(1) solver for the minimal change vector */
uint16_t solve_pesec_delta(uint8_t synd_gap) 
{
    uint16_t delta_vec = 0;
    uint8_t low_chunk = synd_gap & 0x07;
    uint8_t high_chunk = synd_gap & 0x18;

    if (low_chunk != 0) 
    {
        delta_vec |= (1 << (low_chunk - 1));
    }
    
    if (high_chunk != 0) 
    {
        delta_vec |= (1 << (6 + (high_chunk >> 3)));
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
    
    /* Step 6: Send the new redundancy state to the hardware */
output_to_shift_registers();
}