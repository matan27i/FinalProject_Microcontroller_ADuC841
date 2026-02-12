/* File: rx_decoder.c
   H1-Type Stateful Bus Decoder - Core Algorithm Implementation (RECEIVER SIDE)
   Target: ADuC841 (8052 single-cycle core)
 
   H1-TYPE MATRIX STRUCTURE (4×15):
   The H1-type check matrix H has dimensions 4×15.
   Column index i (1-based, i = 1..15) contains the 4-bit binary representation of i.
   
   SYNDROME COMPUTATION (DECODING):
   S = H * x^T is computed by XOR-ing together the column indices (1..15)
   for every bit position that is set in x.
   
   If bit j (0-indexed) is set in x, it contributes column (j+1) to the syndrome.
   syndrome = XOR of all (j+1) where x[j] = 1
   
   DECODER PRINCIPLE:
   The TX side encodes data such that: H * bus_state^T = data_nibble
   Therefore, the RX side simply computes: data_nibble = H * bus_state^T
   
   This is the INVERSE operation of the TX encoder:
   - TX: Given data_nibble, find bus_state such that syndrome = data_nibble
   - RX: Given bus_state, compute syndrome to recover data_nibble
   
   CRITICAL: The syndrome computation function is IDENTICAL between TX and RX.
            Both use the same H1 matrix structure and XOR-based computation.
 
 */

#include <aduc841.h>
#include "rx_header.h"

/* ---------------------------------------------------------------------------
   rx_compute_syndrome
   ---------------------------------------------------------------------------
   Computes S = H * bus_state^T on-the-fly using bitwise XOR.
   
   IDENTICAL to compute_syndrome_from_bus() in TX side (bus_encoder.c).
   
   The H1-type matrix column i (1-indexed) equals i in binary.
   So: syndrome = XOR of all column indices (j+1) for bits j that are set.
   
   Implementation: Loop through bits 0..14 of bus_state. For each set bit at
   position j, XOR the syndrome with (j+1).
   
   No lookup tables. No stored matrix. Pure algorithmic computation.
   
   Example:
     bus_state = 0x0015 (binary: 000 0000 0001 0101)
     Set bits: positions 0, 2, 4 (0-indexed)
     Column indices: 1, 3, 5
     syndrome = 1 ^ 3 ^ 5 = 0111 (binary) = 0x7
     Result: data_nibble = 0x7
   ---------------------------------------------------------------------------
 */
uint8_t rx_compute_syndrome(uint16_t bus_state)
{
    uint8_t syndrome = 0;
    uint8_t col_idx;
    uint16_t temp_state;
    
    /* Mask to only use bits 0..14 (15-bit bus) */
    temp_state = bus_state & BUS_STATE_MASK;
    
    /* Iterate through bit positions 0..14.
       If bit j is set, XOR in column index (j+1).
       Column index (j+1) is the 4-bit binary representation of (j+1).
     */
    for (col_idx = 1; col_idx <= HAMMING_N; col_idx++)
    {
        /* Check if bit (col_idx - 1) is set */
        if (temp_state & 0x0001)
        {
            /* XOR the column index (which IS the column value in H1-type matrix) */
            syndrome ^= col_idx;
        }
        /* Shift to next bit */
        temp_state >>= 1;
    }
    
    /* Return 4-bit syndrome (which equals the data nibble) */
    return (syndrome & 0x0F);
}

/* ---------------------------------------------------------------------------
   rx_process_bus_state
   ---------------------------------------------------------------------------
   Core decoder function. Processes one 15-bit corrected bus state from Stage 1.
   
   Implements a 2-state FSM for nibble reassembly:
   
   State: RX_STATE_WAIT_HIGH (expecting high nibble)
     1. Compute syndrome → extract data nibble
     2. Store as high nibble
     3. Transition to RX_STATE_WAIT_LOW
   
   State: RX_STATE_WAIT_LOW (expecting low nibble)
     1. Compute syndrome → extract data nibble
     2. Combine with stored high nibble: byte = (high << 4) | low
     3. Transmit complete byte via UART
     4. Increment byte counter
     5. Transition back to RX_STATE_WAIT_HIGH
   
   CRITICAL TIMING:
   This function should be called at the same rate that the TX sends nibbles.
   If TX sends nibbles every 10ms, this should be called every 10ms.
   
   SYNCHRONIZATION:
   If sync is lost (wrong characters received), call rx_reset_state_machine()
   to resynchronize to the high nibble boundary.
   ---------------------------------------------------------------------------
 */
void rx_process_bus_state(uint16_t bus_state)
{
    uint8_t data_nibble;
    uint8_t complete_byte;
    
    /* Step 1: Extract data nibble from bus state via syndrome computation */
    data_nibble = rx_compute_syndrome(bus_state);
    
    /* Step 2: Process according to current FSM state */
    switch (rx_current_state)
    {
        case RX_STATE_WAIT_HIGH:
            /* First nibble in pair: This is the HIGH nibble (bits 7-4) */
            
            /* Store high nibble for next cycle */
            rx_stored_high_nibble = data_nibble & 0x0F;
            
            /* Transition to waiting for low nibble */
            rx_current_state = RX_STATE_WAIT_LOW;
            
            break;
            
        case RX_STATE_WAIT_LOW:
            /* Second nibble in pair: This is the LOW nibble (bits 3-0) */
            
            /* Combine with stored high nibble to form complete byte
               Byte format: [high_nibble : low_nibble]
               Example: high=0x4, low=0x1 → byte=0x41='A'
             */
            complete_byte = (rx_stored_high_nibble << 4) | (data_nibble & 0x0F);
            
            /* Transmit reconstructed byte to PC via UART */
            rx_send_uart_byte(complete_byte);
            
            /* Update statistics */
            rx_bytes_decoded++;
            rx_byte_ready = 1;  /* Signal that byte was sent (optional) */
            
            /* Reset state machine for next byte */
            rx_current_state = RX_STATE_WAIT_HIGH;
            rx_stored_high_nibble = 0;  /* Clear for safety */
            
            break;
            
        default:
            /* Safety: Reset to known state if corrupted */
            rx_reset_state_machine();
            break;
    }
}

/* ---------------------------------------------------------------------------
   rx_reset_state_machine
   ---------------------------------------------------------------------------
   Reset decoder FSM to initial state.
   
   Call this function to resynchronize if:
   - Wrong characters are being decoded (sync lost)
   - Timeout occurs (missing nibble)
   - System startup
   - After error condition
   
   This forces the decoder to expect a HIGH nibble next, which should
   resync with the TX side at the next character boundary.
   ---------------------------------------------------------------------------
 */
void rx_reset_state_machine(void)
{
    rx_current_state = RX_STATE_WAIT_HIGH;
    rx_stored_high_nibble = 0;
    rx_byte_ready = 0;
}

/* ---------------------------------------------------------------------------
   rx_send_uart_byte
   ---------------------------------------------------------------------------
   Transmit one byte via UART to PC.
   
   BLOCKING TRANSMISSION:
   Waits for TI (transmit interrupt flag) to be set before loading SBUF.
   This ensures previous transmission is complete.
   
   Protocol: Standard 8051 UART transmission
   1. Wait for TI = 1 (previous byte sent)
   2. Clear TI
   3. Load byte into SBUF
   4. Hardware automatically sends the byte
   5. Hardware sets TI when transmission complete
   
   Timing @ 9600 baud:
   - 1 byte = 10 bits (1 start + 8 data + 1 stop)
   - Transmission time = 10 bits / 9600 bps ≈ 1.04 ms
   
   CRITICAL: UART must be initialized (RX_UART_Init) before calling this.
   ---------------------------------------------------------------------------
 */
void rx_send_uart_byte(uint8_t data_byte)
{
    /* Wait for previous transmission to complete
       TI is set by hardware when byte transmission finishes
     */
    while (!TI)
    {
        /* Busy wait - can be replaced with low-power sleep if needed */
    }
    
    /* Clear transmit interrupt flag */
    TI = 0;
    
    /* Load byte into UART transmit buffer
       Hardware automatically starts transmission
     */
    SBUF = data_byte;
    
    /* Note: TI will be set automatically when transmission completes */
}

/* ---------------------------------------------------------------------------
   OPTIONAL: Fast syndrome computation using lookup table
   ---------------------------------------------------------------------------
   For maximum performance, you can pre-compute all syndromes and store
   them in a lookup table. This trades 32 KB of code memory for O(1) speed.
   
   Uncomment this section if you need maximum decoding speed.
 */
#if 0  /* Set to 1 to enable lookup table */

/* Pre-computed syndrome lookup table for all 32768 possible bus states
   Each entry is the syndrome for that bus state index.
   
   WARNING: This uses 32 KB of code memory!
   Only enable if you have sufficient memory and need maximum speed.
 */
code const uint8_t syndrome_lut[32768] = {
    /* 0x0000 */ 0x0,
    /* 0x0001 */ 0x1,
    /* 0x0002 */ 0x2,
    /* 0x0003 */ 0x3,
    /* ... continue for all 32768 values ... */
    /* Generate this table with a Python script or C program */
};

/* Fast syndrome lookup (O(1) instead of O(n)) */
uint8_t rx_compute_syndrome_fast(uint16_t bus_state)
{
    return syndrome_lut[bus_state & BUS_STATE_MASK];
}

#endif  /* End lookup table */
