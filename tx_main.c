/* File: main.c
 * Group 1: Main Loop & Hardware Peripherals
 * ==========================================
 * Target: ADuC841 (8052 single-cycle core, 11.0592 MHz crystal)
 *
 * Contains:
 *   - State variables for UART reception and Timer2 hardware
 *   - All peripheral initialisation (Timers, UART, Ports)
 *   - Interrupt Service Routines (UART, Timer2)
 *   - Shift-register output driver (25-bit: 15 data + 10 ECC)
 *   - Main entry point and event loop
 *
 * Portability: To retarget to another MCU, replace the register-level
 *              code in this file and update the pin definitions in header.h.
 *              Groups 2 and 3 are hardware-independent.
 *
 * FIX LOG:
 *   [C1] Added PLLCON write (CD=0) so core runs at full 11.0592 MHz.
 *        Replaced Timer 3 baud-rate generator with Timer 1 Mode 2
 *        (Timer 3 cannot produce 9600 baud at 11.0592 MHz).
 *   [M1] Moved GlobalINT() to end of init sequence.
 *   [M2] Corrected Timer 2 reload from 0xD4CD to T2_RELOAD constants
 *        (0xFF84 = 124 counts = ~11.2 us at 11.0592 MHz).
 *   [M4] Replaced single-byte UART capture with ring buffer to prevent
 *        data loss during shift-register output.
 *   [N1] Removed redundant EA=1 from Init_Timer2().
 *   [N3] Replaced hardcoded 0xD4/0xCD with T2_RELOAD_H/L constants.
 */

#include "tx_header.h"

/* =========================================================================
 * STATE VARIABLE DEFINITIONS  (owned by this module)
 * ========================================================================= */

/* UART / main-loop flags */
volatile bit    buffer_flag  = 0;   /* Batch terminator received             */
volatile uint8_t buffer_count = 0;  /* Nibbles processed in current batch    */

/* [M4] UART receive ring buffer — replaces single-byte tx_temp_byte/tx_flag */
volatile uint8_t rx_buf[RX_BUF_SIZE];
volatile uint8_t rx_head = 0;       /* Next write position (ISR)             */
volatile uint8_t rx_tail = 0;       /* Next read position (main loop)        */

/* Timer 2 shift-register driver state */
volatile uint8_t counter_t2   = 0;  /* Edge counter for ISR                  */
volatile uint8_t flag_t2_mod  = 0;  /* ISR mode: 0=clock, 1=clear, 2=latch  */

/* PESEC block configuration (passed to ECC init at startup) */
static uint8_t pesec_config[] = {3, 2};

/* =========================================================================
 * HARDWARE INITIALISATION
 * ========================================================================= */

/* Port_Init — Set shift-register control pins to safe idle state */
void Port_Init(void)
{
    SR_CLK   = 0;  /* Clock idle low   */
    SR_LATCH = 1;  /* Latch idle high  */
    SR_CLR   = 1;  /* Clear deasserted */
}

/* Init_Timer2 — Configure Timer 2 for shift-register clock generation.
 * [M2] Reload value 0xFF84 gives 124 counts = ~11.2 us at 11.0592 MHz.
 * [N1] EA is NOT set here; GlobalINT() handles it after all inits.
 * Timer is NOT started here; individual output routines start it. */
void Init_Timer2(void)
{
    T2CON  = 0x00;
    RCAP2H = T2_RELOAD_H;
    RCAP2L = T2_RELOAD_L;
    ET2    = 1;     /* Enable Timer 2 interrupt */
    TR2    = 0;     /* Stopped until needed     */
}

/* [C1] BaudRate_Init — Timer 1 Mode 2 (8-bit autoreload) for 9600 baud.
 * At fCORE = 11.0592 MHz, SMOD = 0:
 *   Baud = fCORE / (32 * (256 - TH1)) = 11059200 / (32 * 36) = 9600
 *   TH1  = 256 - 36 = 220 = 0xDC
 *
 * Replaces Timer3_Init which cannot produce 9600 baud at this clock. */
void BaudRate_Init(void)
{
    TMOD &= 0x0F;      /* Clear Timer 1 mode bits, preserve Timer 0 */
    TMOD |= 0x20;      /* Timer 1: Mode 2 (8-bit autoreload) */
    TH1   = 0xDC;      /* Reload value for 9600 baud */
    TL1   = 0xDC;
    TR1   = 1;          /* Start Timer 1 */
}

/* UART_Init — 8N1, variable baud rate (set by Timer 1) */
void UART_Init(void)
{
    SM0 = 0;  /* Mode 1: 8-bit UART      */
    SM1 = 1;  /* Variable baud rate      */
    REN = 1;  /* Enable receiver         */
    RI  = 0;  /* Clear receive flag      */
    TI  = 0;  /* Clear transmit flag     */
    ES  = 1;  /* Enable serial interrupt */
}

/* GlobalINT — Master interrupt enable */
void GlobalINT(void)
{
    EA = 1;
}

/* =========================================================================
 * INTERRUPT SERVICE ROUTINES
 * ========================================================================= */

/* [M4] UART ISR — Store received byte into ring buffer.
 * Drops the byte silently if buffer is full. */
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
        /* else: buffer full, byte is dropped */
    }
    if (TI)
    {
        TI = 0;
    }
}

/* Timer2 ISR (vector 5) — Drives shift-register clock/latch/clear sequences.
 *
 * Mode 0 (flag_t2_mod == 0): Toggle SR_CLK twice  -> one full clock cycle
 *                             (shifts one bit into 74HC595).
 * Mode 1 (flag_t2_mod == 1): Assert SR_CLR low + SR_LATCH low, toggle CLK
 *                             4 times, then restore -> hardware reset.
 * Mode 2 (flag_t2_mod == 2): Toggle SR_LATCH twice -> one latch pulse
 *                             (transfers shift reg -> output reg).
 */
void Timer2_ISR(void) interrupt 5
{
    TF2 = 0;  /* Clear overflow flag (required for Timer 2) */

    if (flag_t2_mod == 0)
    {
        /* --- Mode 0: Clock one bit --- */
        if (counter_t2 < 2)
        {
            SR_CLK = !SR_CLK;
            counter_t2++;
        }
        else
        {
            TR2    = 0;
            SR_CLK = 0;   /* Ensure idle-low */
        }
    }
    else if (flag_t2_mod == 1)
    {
        /* --- Mode 1: Hardware clear sequence --- */
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
            SR_CLK     = 0;   /* Force clock idle-low after clear */
            SR_LATCH   = 1;
            SR_CLR     = 1;
            flag_t2_mod = 0;
        }
    }
    else if (flag_t2_mod == 2)
    {
        /* --- Mode 2: Latch pulse --- */
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

/* =========================================================================
 * SHIFT-REGISTER OUTPUT DRIVER
 * =========================================================================
 * Serialises the full 25-bit word (10 ECC + 15 data, MSB-first) into
 * daisy-chained 74HC595 shift registers, then pulses the latch.
 *
 * Uses Timer 2 ISR for clock generation (non-blocking per bit, but the
 * function blocks until all bits are shifted out).
 *
 * [N3] Uses T2_RELOAD_H/L constants instead of hardcoded values.
 * ========================================================================= */
void output_to_shift_registers(void)
{
    uint16_t state_copy;
    uint16_t ecc_copy;
    signed char bit_pos;
    uint8_t bit_value;

    /* Snapshot both registers for consistency during output */
    state_copy = current_bus_state & BUS_STATE_MASK;
    ecc_copy   = pesec_redundancy_reg & PESEC_RED_MASK;

    /* --- Phase 1: Shift out 10 ECC redundancy bits (MSB-first) --- */
    for (bit_pos = 9; bit_pos >= 0; bit_pos--)
    {
        bit_value = (uint8_t)((ecc_copy >> bit_pos) & 0x0001);
        SR_DATA   = bit_value;

        counter_t2 = 0;
        TH2 = T2_RELOAD_H;
        TL2 = T2_RELOAD_L;
        TR2 = 1;
        while (TR2 == 1);   /* Wait for one clock cycle */
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
        while (TR2 == 1);   /* Wait for one clock cycle */
    }

    /* --- Phase 3: Latch pulse (transfer shift reg -> output reg) --- */
    flag_t2_mod = 2;
    counter_t2  = 0;
    TH2 = T2_RELOAD_H;
    TL2 = T2_RELOAD_L;
    TR2 = 1;
    while (TR2 == 1);   /* Wait for latch pulse to complete */
}

/* =========================================================================
 * MAIN ENTRY POINT
 * ========================================================================= */
void main(void)
{
    /* [C1] Set core clock to full crystal speed.
     * PLLCON default = 0x53 -> CD=3 -> fCORE = 11.0592/8 = 1.38 MHz.
     * We need CD=0 -> fCORE = 11.0592 MHz for correct Timer 2 timing.
     * On ADuC841, core_clk = crystal_freq / 2^CD (datasheet p. 49). */
    PLLCON = (PLLCON & 0xF8);  /* CD = 0: fCORE = 11.0592 MHz */

    /* [M1] Initialise all peripherals BEFORE enabling interrupts.
     * This prevents ISR entry with unconfigured peripheral registers. */
    Port_Init();
    BaudRate_Init();
    UART_Init();
    Init_Timer2();

    /* ECC Initialisation (Group 3) */
    Init_PESEC_Matrices(pesec_config, 2);

    /* [M1] Enable interrupts LAST, after all peripherals are ready */
    GlobalINT();

    /* --- Event Loop --- */
    while (1)
    {
        /* [M4] Drain UART ring buffer one byte at a time */
        if (rx_tail != rx_head)
        {
            uint8_t rx_byte = rx_buf[rx_tail];
            rx_tail = (rx_tail + 1) % RX_BUF_SIZE;
            tx_handler(rx_byte);
        }

        /* Handle batch terminator */
        if (buffer_flag)
        {
            ES = 0;             /* Enter critical section  */
            buffer_flag  = 0;
            buffer_count = 0;
            ES = 1;             /* Exit critical section   */
        }
    }
}
