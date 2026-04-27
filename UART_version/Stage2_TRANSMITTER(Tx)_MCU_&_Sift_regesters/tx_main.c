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
 *   [W4] output_to_shift_registers() now shifts 26 bits.
 *   [W5] Reordered the 26-bit shift to red -> parity -> data so each
 *        field lands at the documented Mega bridge pin (PC7 = parity,
 *        PK = red[0..7], PF[1:0] = red[8..9]).  The previous "parity
 *        first" order placed parity in chip-4 QB and shifted every
 *        red bit one position off.
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
 * [W4] Serialises the full 26-bit word (10 ECC + 1 parity + 15 data) into
 * 4 daisy-chained 74HC595 shift registers, then pulses the latch.
 *
 * SHIFT ORDER (first bit shifted -> farthest from SER input -> deepest chip):
 *   Phase 0: 10 ECC redundancy bits (pesec_redundancy_reg, MSB-first)
 *   Phase 1:  1 overall parity bit  (pesec_overall_parity)
 *   Phase 2: 15 data bits           (current_bus_state, MSB-first)
 *   Phase 3:  Latch pulse
 *
 * After 26 shifts + latch, the chain holds:
 *   Chip 1 (closest to SER):  QA..QH = data[0..7]   -> PA0..PA7
 *   Chip 2:                   QA..QG = data[8..14]  -> PC0..PC6
 *                             QH     = parity       -> PC7
 *   Chip 3:                   QA..QH = red[0..7]    -> PK0..PK7
 *   Chip 4 (deepest):         QA     = red[8]       -> PF0
 *                             QB     = red[9]       -> PF1
 *                             QC..QH = stale (unused by bridge)
 *
 * [BUG FIX] An earlier revision shifted parity in Phase 0 (FIRST), which
 * placed it at the deepest chip-4 QB position rather than chip-2 QH.  The
 * Mega bridge then read red[0] as parity, parity as red[9], and the rest
 * of the redundancy bits one position off.  See the inline comment in the
 * function body for the full position mapping.
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

    /* --- Phase 0: Shift out 10 ECC redundancy bits (MSB-first) ---
     *
     * [BUG FIX] The previous revision shifted the overall parity bit FIRST
     * (Phase 0), assuming "parity shifted first arrives at PC7 / chip-2 QH."
     * That assumption is wrong for a 4-chip 26-bit daisy chain: the FIRST
     * bit shifted ends up in the DEEPEST chip (chip 4), not in chip 2.
     *
     * With the old order the physical pin assignments came out misaligned:
     *     PC7  (chip 2 QH) = red[0]    <- bridge thinks this is parity
     *     PK0..PK7         = red[1..8] <- bridge thinks red[0..7]
     *     PF0  (chip 4 QA) = red[9]    <- bridge thinks red[8]
     *     PF1  (chip 4 QB) = parity    <- bridge thinks red[9]
     *
     * The correct order to land each field at the documented Mega pin --
     *     PA0..PA7  = data[0..7]    (chip 1, positions 1..8 from input)
     *     PC0..PC6  = data[8..14]   (chip 2, positions 9..15)
     *     PC7       = parity        (chip 2 QH, position 16)
     *     PK0..PK7  = red[0..7]     (chip 3, positions 17..24)
     *     PF0       = red[8]        (chip 4 QA, position 25)
     *     PF1       = red[9]        (chip 4 QB, position 26)
     * is to shift in the order: red[9..0] FIRST (oldest, lands deepest),
     * THEN parity (the 11th bit, lands at chip-2 QH = PC7),
     * THEN data[14..0] LAST (newest, lands at chip 1 + chip 2 QA..QG). */
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

    /* --- Phase 1: Shift out the 1 overall parity bit ---
     * Position 16 from input = chip 2 QH = PC7 (Mega pin 30). */
    SR_DATA = parity_copy;

    counter_t2 = 0;
    TH2 = T2_RELOAD_H;
    TL2 = T2_RELOAD_L;
    TR2 = 1;
    while (TR2 == 1);

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

    /* [W5] Enable on-chip 2 KB XRAM.  The PESEC matrices in tx_ecc.c
     * live in XDATA, so MOVX must be able to address the internal RAM
     * window.  CFG841 bit 0 (XRAMEN) routes 0x0000..0x07FF to the
     * on-chip XRAM rather than off-chip; without this, MOVX silently
     * misses and the matrices read as 0xFF. */
    CFG841 |= 0x01;

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
            /* No ES guard needed: UART_ISR does not touch buffer_flag or
             * buffer_count, so single-byte writes are atomic on the 8052
             * and cannot race with the ISR. */
            buffer_flag  = 0;
            buffer_count = 0;
        }
    }
}
