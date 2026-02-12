/* File: rx_input.c
   Stage 1 Interface - Bus State Input Handler (RECEIVER SIDE)
   Target: ADuC841 (8052 single-cycle core)
   
   This file handles the hardware interface to Stage 1 (error corrector).
   Stage 1 provides corrected 15-bit bus states that need to be read
   and processed by the decoder.
   
   IMPLEMENTATION OPTIONS:
   
   Option 1: PARALLEL PORT INTERFACE
     - Bus state bits 0-7 on P0
     - Bus state bits 8-14 on P1.0-P1.6
     - DATA_READY signal on P3.2
   
   Option 2: SERIAL INTERFACE (SPI/I2C)
     - Shift in 15 bits from Stage 1
     - Handshake protocol
   
   Option 3: SHARED MEMORY/REGISTER
     - Direct read from memory-mapped register
     - ADuC841 external memory interface
   
   USER CUSTOMIZATION REQUIRED:
   Modify rx_read_bus_state() based on your actual hardware interface.
 */

#include <aduc841.h>
#include <intrins.h>  /* For _nop_() if needed for timing */
#include "rx_header.h"

/* ---------------------------------------------------------------------------
   rx_read_bus_state - PARALLEL PORT IMPLEMENTATION (Example)
   ---------------------------------------------------------------------------
   Reads 15-bit bus state from parallel port interface.
   
   HARDWARE CONFIGURATION (Example):
   - P0 (8-bit) = bus state bits 0-7 (LSB)
   - P1.0-P1.6 (7-bit) = bus state bits 8-14 (MSB)
   - P3.2 (DATA_READY) = signal from Stage 1
   
   TIMING:
   - Stage 1 asserts DATA_READY when new corrected bus state is available
   - RX reads ports
   - Optional: RX asserts ACK signal back to Stage 1
   
   CRITICAL: Ensure port pins are configured as inputs in RX_Port_Init().
   
   Modify this function based on your actual Stage 1 interface.
   ---------------------------------------------------------------------------
 */
uint16_t rx_read_bus_state(void)
{
    uint16_t bus_state;
    uint8_t low_byte;
    uint8_t high_byte;
    
    /* --- PARALLEL PORT IMPLEMENTATION --- */
    
    /* Read low byte (bits 0-7) from P0 */
    low_byte = P0;
    
    /* Read high byte (bits 8-14) from P1.0-P1.6 
       Mask to only use lower 7 bits of P1
     */
    high_byte = P1 & 0x7F;
    
    /* Combine into 15-bit bus state */
    bus_state = low_byte;                    /* Bits 0-7 */
    bus_state |= ((uint16_t)high_byte << 8); /* Bits 8-14 */
    
    /* Ensure only 15 bits are used */
    bus_state &= BUS_STATE_MASK;
    
    /* Optional: Add setup time delay if needed for signal settling
       Uncomment if your hardware requires settling time:
       _nop_();
       _nop_();
     */
    
    return bus_state;
}

/* ---------------------------------------------------------------------------
   ALTERNATIVE IMPLEMENTATIONS
   ---------------------------------------------------------------------------
   Uncomment the appropriate implementation based on your hardware.
 */

/* --- OPTION 2: SERIAL SHIFT-IN INTERFACE --- */
#if 0  /* Set to 1 to enable serial interface */

/* Pin definitions for serial interface */
sbit SERIAL_DATA = P2^0;  /* Serial data from Stage 1 */
sbit SERIAL_CLK  = P2^1;  /* Clock output to Stage 1 */
sbit SERIAL_LOAD = P2^2;  /* Load pulse to Stage 1 */

/* Delay for serial clock timing (~100 kHz) */
static void serial_delay(void)
{
    uint8_t i = 9;  /* Adjust for desired clock speed */
    while (i--) {
        _nop_();
    }
}

/* Read 15-bit bus state via serial shift-in */
uint16_t rx_read_bus_state(void)
{
    uint16_t bus_state = 0;
    uint8_t bit_count;
    
    /* Assert LOAD to latch parallel data into shift register */
    SERIAL_LOAD = 1;
    serial_delay();
    SERIAL_LOAD = 0;
    serial_delay();
    
    /* Shift in 15 bits, MSB first */
    for (bit_count = 0; bit_count < 15; bit_count++)
    {
        /* Clock low */
        SERIAL_CLK = 0;
        serial_delay();
        
        /* Read data bit */
        bus_state <<= 1;
        if (SERIAL_DATA) {
            bus_state |= 0x0001;
        }
        
        /* Clock high */
        SERIAL_CLK = 1;
        serial_delay();
    }
    
    /* Return clock to low (idle state) */
    SERIAL_CLK = 0;
    
    return (bus_state & BUS_STATE_MASK);
}

#endif  /* End serial interface */

/* --- OPTION 3: MEMORY-MAPPED INTERFACE --- */
#if 0  /* Set to 1 to enable memory-mapped interface */

/* Define Stage 1 output register address
   Adjust based on your hardware memory map
 */
#define STAGE1_DATA_REG  ((volatile uint16_t xdata *)0x8000)

/* Read 15-bit bus state from memory-mapped register */
uint16_t rx_read_bus_state(void)
{
    uint16_t bus_state;
    
    /* Read from Stage 1 output register
       The 'xdata' keyword accesses external data memory
     */
    bus_state = *STAGE1_DATA_REG;
    
    /* Mask to 15 bits */
    bus_state &= BUS_STATE_MASK;
    
    return bus_state;
}

#endif  /* End memory-mapped interface */

/* --- OPTION 4: SPI SLAVE INTERFACE --- */
#if 0  /* Set to 1 to enable SPI interface */

/* ADuC841 SPI pins (adjust if needed) */
sbit SPI_SS   = P1^4;  /* Slave select (input from Stage 1) */
sbit SPI_MOSI = P1^5;  /* Master out, slave in */
sbit SPI_MISO = P1^6;  /* Master in, slave out (not used) */
sbit SPI_SCK  = P1^7;  /* Serial clock (from Stage 1) */

/* Read 15-bit bus state via SPI */
uint16_t rx_read_bus_state(void)
{
    uint16_t bus_state = 0;
    uint8_t bit_count;
    
    /* Wait for chip select to go low */
    while (SPI_SS);
    
    /* Receive 15 bits via SPI (MSB first) */
    for (bit_count = 0; bit_count < 15; bit_count++)
    {
        /* Wait for clock rising edge */
        while (!SPI_SCK);
        
        /* Sample data on rising edge */
        bus_state <<= 1;
        if (SPI_MOSI) {
            bus_state |= 0x0001;
        }
        
        /* Wait for clock falling edge */
        while (SPI_SCK);
    }
    
    /* Wait for chip select to go high (end of transaction) */
    while (!SPI_SS);
    
    return (bus_state & BUS_STATE_MASK);
}

#endif  /* End SPI interface */

/* ---------------------------------------------------------------------------
   OPTIONAL: Handshake with Stage 1
   ---------------------------------------------------------------------------
   If your Stage 1 requires acknowledgment after reading, implement here.
 */
#if 0  /* Set to 1 to enable handshake */

sbit ACK_PIN = P3^3;  /* Acknowledge signal to Stage 1 */

/* Send acknowledgment pulse to Stage 1 */
void rx_send_ack(void)
{
    ACK_PIN = 1;
    _nop_();
    _nop_();
    _nop_();
    _nop_();
    ACK_PIN = 0;
}

/* Modified read function with ACK */
uint16_t rx_read_bus_state_with_ack(void)
{
    uint16_t bus_state;
    
    /* Read bus state (using any method above) */
    bus_state = rx_read_bus_state();
    
    /* Send ACK to Stage 1 */
    rx_send_ack();
    
    return bus_state;
}

#endif  /* End handshake */

/* ---------------------------------------------------------------------------
   DEBUGGING HELPER FUNCTIONS
   ---------------------------------------------------------------------------
   Optional functions for testing and debugging.
 */
#if 0  /* Set to 1 to enable debug functions */

/* Test pattern: Return fixed bus states for testing */
uint16_t rx_read_test_pattern(void)
{
    static uint8_t test_index = 0;
    
    /* Test patterns (example: encoding ASCII 'A' = 0x41)
       High nibble: 0x4
       Low nibble: 0x1
     */
    code const uint16_t test_states[] = {
        0x0010,  /* Syndrome = 0x4 (high nibble of 'A') */
        0x0001,  /* Syndrome = 0x1 (low nibble of 'A') */
        0x0018,  /* Syndrome = 0x4 (high nibble of 'A') */
        0x0001,  /* Syndrome = 0x1 (low nibble of 'A') */
    };
    
    uint16_t state = test_states[test_index];
    test_index = (test_index + 1) % 4;
    
    return state;
}

/* Verify bus state is valid (all bits in legal range) */
bit rx_validate_bus_state(uint16_t bus_state)
{
    /* Check that only bits 0-14 are set */
    if (bus_state & ~BUS_STATE_MASK) {
        return 0;  /* Invalid: bit 15 set */
    }
    
    return 1;  /* Valid */
}

#endif  /* End debug functions */
