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
 */

#include "tx_header.h"

/* =========================================================================
 * STATE VARIABLE DEFINITIONS  (owned by this module)
 * ========================================================================= */

/* UART / main-loop flags */
volatile bit    buffer_flag  = 0;   /* Batch terminator received             */
volatile bit    tx_flag      = 0;   /* UART ISR has captured a byte          */
volatile uint8_t buffer_count = 0;  /* Nibbles processed in current batch    */
volatile uint8_t tx_temp_byte = 0;  /* Raw byte from UART ISR               */

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
 * Reload value 0xD4CD gives ~11.2 µs period at 11.0592 MHz.
 * Timer is NOT started here; individual output routines start it. */
void Init_Timer2(void)
{
    T2CON  = 0x00;
    RCAP2H = 0xD4;
    RCAP2L = 0xCD;
    ET2    = 1;     /* Enable Timer 2 interrupt */
    EA     = 1;
    TR2    = 0;     /* Stopped until needed     */
}

/* Timer3_Init — Baud-rate generator for 9600 baud (11.0592 MHz xtal) */
void Timer3_Init(void)
{
    T3CON &= 0xFE;  /* Disable Timer 3        */
    T3CON |= 0x86;  /* Mode for UART baud gen */
    T3FD   = 0x08;  /* Fractional divider      */
}

/* UART_Init — 8N1, variable baud rate (set by Timer 3) */
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

/* UART ISR (vector 4) — Capture received byte, signal main loop */
void UART_ISR(void) interrupt 4
{
    if (RI)
    {
        RI = 0;
        tx_temp_byte = SBUF;
        tx_flag = 1;
    }
    if (TI)
    {
        TI = 0;
    }
}

/* Timer2 ISR (vector 5) — Drives shift-register clock/latch/clear sequences.
 *
 * Mode 0 (flag_t2_mod == 0): Toggle SR_CLK twice  → one full clock cycle
 *                             (shifts one bit into 74HC595).
 * Mode 1 (flag_t2_mod == 1): Assert SR_CLR low + SR_LATCH low, toggle CLK
 *                             4 times, then restore → hardware reset.
 * Mode 2 (flag_t2_mod == 2): Toggle SR_LATCH twice → one latch pulse
 *                             (transfers shift reg → output reg).
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
            SR_CLK     = 0;   /* FIX: force clock idle-low after clear */
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
        TH2 = 0xD4;
        TL2 = 0xCD;
        TR2 = 1;
        while (TR2 == 1);   /* Wait for one clock cycle */
    }

    /* --- Phase 2: Shift out 15 data bits (MSB-first) --- */
    for (bit_pos = (HAMMING_N - 1); bit_pos >= 0; bit_pos--)
    {
        bit_value = (uint8_t)((state_copy >> bit_pos) & 0x0001);
        SR_DATA   = bit_value;

        counter_t2 = 0;
        TH2 = 0xD4;
        TL2 = 0xCD;
        TR2 = 1;
        while (TR2 == 1);   /* Wait for one clock cycle */
    }

    /* --- Phase 3: Latch pulse (transfer shift reg → output reg) --- */
    flag_t2_mod = 2;
    counter_t2  = 0;
    TH2 = 0xD4;
    TL2 = 0xCD;
    TR2 = 1;
    while (TR2 == 1);   /* Wait for latch pulse to complete */
}

/* =========================================================================
 * MAIN ENTRY POINT
 * ========================================================================= */
void main(void)
{
    /* --- Hardware Initialisation --- */
    GlobalINT();
    Timer3_Init();
    UART_Init();
    Port_Init();
    Init_Timer2();

    /* --- ECC Initialisation (Group 3) --- */
    Init_PESEC_Matrices(pesec_config, 2);

    /* --- Event Loop --- */
    while (1)
    {
        /* Handle UART reception */
        if (tx_flag == 1)
        {
            tx_flag = 0;
            tx_handler(tx_temp_byte);
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
