/* File: main.c
 * Group 1: Main Loop & Hardware Peripherals
 * Target: ADuC841 (8052 single-cycle core, 11.0592 MHz crystal)
 *
 * Contains:
 *   - State variables for UART reception and Timer2 hardware
 *   - All peripheral initialisation (Timers, UART, Ports)
 *   - Interrupt Service Routines (UART, Timer2)
 *   - Shift-register output driver (26-bit: 1 parity + 10 ECC + 15 data)
 *   - Main entry point and event loop
 *
 * FIX LOG:
 *   [C1] Added PLLCON write (CD=0) so core runs at full 11.0592 MHz.
 *   [M1] Moved GlobalINT() to end of init sequence.
 *   [M2] Corrected Timer 2 reload to T2_RELOAD constants.
 *   [M4] Replaced single-byte UART capture with ring buffer.
 *   [N1] Removed redundant EA=1 from Init_Timer2().
 *   [N3] Replaced hardcoded reload values with T2_RELOAD_H/L constants.
 *   [W4] output_to_shift_registers() now shifts 26 bits: the overall
 *        parity bit (from tx_ecc.c) is shifted out first, followed by
 *        the 10 ECC redundancy bits and 15 data bits.
 */

#include "tx_header.h"

/*
 * STATE VARIABLE DEFINITIONS  (owned by this module)
 */

/* UART / main-loop flags */
volatile bit    buffer_flag  = 0;
volatile uint8_t buffer_count = 0;

/* [M4] UART receive ring buffer */
volatile uint8_t rx_buf[RX_BUF_SIZE];
volatile uint8_t rx_head = 0;
volatile uint8_t rx_tail = 0;

/* Timer 2 shift-register driver state */
volatile uint8_t counter_t2   = 0;
volatile uint8_t flag_t2_mod  = 0;

/* PESEC block configuration (passed to ECC init at startup) */
static uint8_t pesec_config[] = {3, 2};

/* 
 * HARDWARE INITIALISATION
 */

void Port_Init(void)
{
    SR_CLK   = 0;
    SR_LATCH = 1;
    SR_CLR   = 1;
}

void Init_Timer2(void)
{
    T2CON  = 0x00;
    RCAP2H = T2_RELOAD_H;
    RCAP2L = T2_RELOAD_L;
    ET2    = 1;
    TR2    = 0;
}

/* [C1] Timer 1 Mode 2 (8-bit autoreload) for 9600 baud. */
void BaudRate_Init(void)
{
    TMOD &= 0x0F;
    TMOD |= 0x20;
    TH1   = 0xDC;
    TL1   = 0xDC;
    TR1   = 1;
}

void UART_Init(void)
{
    SM0 = 0;
    SM1 = 1;
    REN = 1;
    RI  = 0;
    TI  = 0;
    ES  = 1;
}

void GlobalINT(void)
{
    EA = 1;
}

/* 
 * INTERRUPT ROUTINES
 */

void UART_ISR(void) interrupt 4
{
    if (RI)
    {
        uint8_t next_head;
        RI = 0;

        next_head = (rx_head + 1) % RX_BUF_SIZE;
        if (next_head != rx_tail)
        {
            rx_buf[rx_head] = SBUF;
            rx_head = next_head;
        }
    }
    if (TI)
    {
        TI = 0;
    }
}

void Timer2_ISR(void) interrupt 5
{
    TF2 = 0;

    if (flag_t2_mod == 0)
    {
        if (counter_t2 < 2)
        {
            SR_CLK = !SR_CLK;
            counter_t2++;
        }
        else
        {
            TR2    = 0;
            SR_CLK = 0;
        }
    }
    else if (flag_t2_mod == 1)
    {
        if (counter_t2 < 4)
        {
            SR_CLR   = 0;
            SR_LATCH = 0;
            SR_CLK   = !SR_CLK;
            counter_t2++;
        }
        else
        {
            TR2        = 0;
            SR_CLK     = 0;
            SR_LATCH   = 1;
            SR_CLR     = 1;
            flag_t2_mod = 0;
        }
    }
    else if (flag_t2_mod == 2)
    {
        if (counter_t2 < 2)
        {
            SR_LATCH = !SR_LATCH;
            counter_t2++;
        }
        else
        {
            TR2         = 0;
            flag_t2_mod = 0;
        }
    }
}

/* 
 * SHIFT-REGISTER OUTPUT DRIVER
 * [W4] Serialises the full 26-bit word (1 parity + 10 ECC + 15 data,
 * MSB-first) into daisy-chained 74HC595 shift registers, then pulses
 * the latch.
 *
 * SHIFT ORDER (first bit shifted -> farthest chain position):
 *   Phase 0:  1 overall parity bit  (pesec_overall_parity)
 *   Phase 1: 10 ECC redundancy bits (pesec_redundancy_reg, MSB-first)
 *   Phase 2: 15 data bits           (current_bus_state, MSB-first)
 *   Phase 3:  Latch pulse
 *
 * The parity bit is shifted first so it arrives at the output pin
 * connected to the Arduino Mega's PC7 (pin 30).  The ECC and data
 * bits retain their original mapping.
 */
void output_to_shift_registers(void)
{
    uint16_t state_copy;
    uint16_t ecc_copy;
    uint8_t  parity_copy;
    signed char bit_pos;
    uint8_t bit_value;

    /* Snapshot all three values for consistency during output. */
    state_copy  = current_bus_state & BUS_STATE_MASK;
    ecc_copy    = pesec_redundancy_reg & PESEC_RED_MASK;
    parity_copy = pesec_overall_parity & 0x01u;

    /* [FIX] Ensure Mode-0 (single clock-pulse) for all bit shifts.
     * Guards against stale flag_t2_mod if this function is ever
     * entered from an unexpected context. */
    flag_t2_mod = 0;

    /* --- Phase 0: [W4] Shift out 1 overall parity bit --- */
    SR_DATA = parity_copy;

    counter_t2 = 0;
    TH2 = T2_RELOAD_H;
    TL2 = T2_RELOAD_L;
    TR2 = 1;
    while (TR2 == 1);

    /* --- Phase 1: Shift out 10 ECC redundancy bits (MSB-first) --- */
    for (bit_pos = 9; bit_pos >= 0; bit_pos--)
    {
        bit_value = (uint8_t)((ecc_copy >> bit_pos) & 0x0001);
        SR_DATA   = bit_value;

        counter_t2 = 0;
        TH2 = T2_RELOAD_H;
        TL2 = T2_RELOAD_L;
        TR2 = 1;
        while (TR2 == 1);
    }

    /* --- Phase 2: Shift out 15 data bits (MSB-first) --- */
    for (bit_pos = (HAMMING_N - 1); bit_pos >= 0; bit_pos--)
    {
        bit_value = (uint8_t)((state_copy >> bit_pos) & 0x0001);
        SR_DATA   = bit_value;

        counter_t2 = 0;
        TH2 = T2_RELOAD_H;
        TL2 = T2_RELOAD_L;
        TR2 = 1;
        while (TR2 == 1);
    }

    /* --- Phase 3: Latch pulse (transfer shift reg -> output reg) --- */
    flag_t2_mod = 2;
    counter_t2  = 0;
    TH2 = T2_RELOAD_H;
    TL2 = T2_RELOAD_L;
    TR2 = 1;
    while (TR2 == 1);
}

/* 
 * MAIN ENTRY POINT
 */
void main(void)
{
    /* [FIX] ADuC841: CD=0 for full-speed core (11.0592 MHz).
     * Per datasheet p.49, all non-CD bits in PLLCON are reserved
     * on the ADuC841 and must be written as 0.  The previous
     * read-modify-write (PLLCON & 0xF8) left reserved bits 6
     * and 4 set from the power-on default of 0x53. */
    PLLCON = 0x00;

    Port_Init();
    BaudRate_Init();
    UART_Init();
    Init_Timer2();

    Init_PESEC_Matrices(pesec_config, 2);

    GlobalINT();

    while (1)
    {
        if (rx_tail != rx_head)
        {
            uint8_t rx_byte = rx_buf[rx_tail];
            rx_tail = (rx_tail + 1) % RX_BUF_SIZE;
            tx_handler(rx_byte);
        }

        if (buffer_flag)
        {
            ES = 0;
            buffer_flag  = 0;
            buffer_count = 0;
            ES = 1;
        }
    }
}
