/* =============================================================================
   File: rx_testbench.ino
   Arduino Mega 2560 -- Software TX Pipeline Testbench for rx_system
   =============================================================================

   PURPOSE:
     Software-simulates the ADuC841 TX encoding pipeline (H1-type Hamming
     + PESEC SEC+DED) for a fixed message, wraps each 26-bit encoded word
     into the exact UART frame format used by the real bridge, and streams
     the frames on USART1 to an ADuC841 running rx_system firmware.

     This replaces the full hardware chain (TX ADuC841 + 74HC595 shift
     registers + bridge Mega) with a single Arduino, so you can verify
     that the RX decoder (PESEC + H1 differential + nibble FSM + UART
     output) is alive and correctly reconstructs the original ASCII
     payload on PuTTY.

   EXPECTED PUTTY OUTPUT (with RX_DEBUG=1 in rx_types.h):
     On RX boot:                         "RX-READY\r\n"
     Each message pass (every ~3 s):     "R" (reset frame acknowledged)
                                         "FFGFFuFFCFFcFFiFFGFFaFFNFFg"
       where each "F" is a CRC-valid-frame marker and the letters are the
       decoded payload bytes emitted by the nibble FSM after the second
       nibble of each character completes the byte.

     If RX_DEBUG=0, only the bare payload "GuCciGaNg" prints each pass.

   WIRE FORMAT (identical to bridge_mega2560.ino):
     Byte 0: 0xAA               Start-of-frame
     Byte 1: data[7:0]
     Byte 2: parity<<7 | data[14:8]
     Byte 3: red[7:0]
     Byte 4: red[9:8]
     Byte 5: CRC-8 (poly 0x07, init 0x00) over bytes 1..4

   UART: 115200 8N1, U2X=1 (same as bridge and rx_hw.c).

   PIN WIRING (Mega -> ADuC841 SAR Eval Board J4):
     Mega Pin 18 (TX1, PD3)  ->  J4-2 (RXD, ADuC P3.0)
     Mega GND                ->  J4-4 (GND)

     To run the testbench, move the ADuC's J4-2 RXD input from the
     USB-UART bridge to Mega pin 18.  Leave J4-3 (TXD, ADuC P3.1)
     connected to the USB-UART bridge so PuTTY sees the decoded output.

   TIMING:
     A frame takes ~520 us on the wire (6 bytes * 10 bits / 115200).
     We insert a 5 ms inter-frame delay so the RX main loop can finish
     PESEC + H1 + nibble processing and drain any debug bytes out of
     its TX ring before the next frame arrives.  This is overkill but
     makes the output trace easy to read on PuTTY.
============================================================================= */

#include <avr/io.h>


/* -----------------------------------------------------------------------------
   FRAME / ENCODER CONSTANTS  (must match rx_hw.c and the TX firmware)
----------------------------------------------------------------------------- */
#define UART_SOF           0xAAu
#define UART_CRC_POLY      0x07u
#define UART_BAUD          115200UL
#define UART_UBRR_VAL      ((F_CPU / (8UL * UART_BAUD)) - 1UL)   /* U2X=1 */

#define HAMMING_N          15
#define BUS_STATE_MASK     ((uint16_t)0x7FFFu)
#define PESEC_RED_BITS     10
#define PESEC_RED_MASK     ((uint16_t)0x03FFu)
#define MAX_PESEC_BLOCKS   5


/* -----------------------------------------------------------------------------
   ENCODER STATE  (mirrors tx_encoder.c / tx_ecc.c)
----------------------------------------------------------------------------- */
static uint16_t current_bus_state    = 0;
static uint16_t pesec_redundancy_reg = 0;
static uint8_t  pesec_overall_parity = 0;

/* PESEC matrices -- built at startup from pesec_config. */
static uint8_t PESEC_MAT_D[20];
static uint8_t PESEC_MAT_A[40];
static uint8_t pesec_num_blocks;
static uint8_t pesec_bit_offsets[MAX_PESEC_BLOCKS];
static uint8_t pesec_col_offsets[MAX_PESEC_BLOCKS];
static uint8_t pesec_chunk_masks[MAX_PESEC_BLOCKS];
static uint8_t pesec_num_d_cols;
static uint8_t pesec_num_a_cols;

/* Block configuration -- MUST match pesec_config[] in tx_main.c/rx_main.c. */
static const uint8_t pesec_config[] = {3, 2};


/* -----------------------------------------------------------------------------
   USART1 LOW-LEVEL
   Bypassing the Arduino Serial1 class so the register setup matches
   bridge_mega2560.ino exactly (U2X=1, TX-only, polled).
----------------------------------------------------------------------------- */
static void uart1_init(void)
{
    UCSR1A = (1u << U2X1);
    UBRR1H = (uint8_t)(UART_UBRR_VAL >> 8);
    UBRR1L = (uint8_t)(UART_UBRR_VAL);
    UCSR1C = (1u << UCSZ11) | (1u << UCSZ10);      /* 8N1 */
    UCSR1B = (1u << TXEN1);                        /* TX only */
}

static inline void uart1_write_byte(uint8_t b)
{
    while (!(UCSR1A & (1u << UDRE1)))
    {
        /* spin until transmit data register is empty */
    }
    UDR1 = b;
}


/* -----------------------------------------------------------------------------
   CRC-8 (poly 0x07, init 0x00) -- identical to bridge crc8 and rx_hw.c
----------------------------------------------------------------------------- */
static uint8_t crc8(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0u;
    uint8_t i, j;

    for (i = 0u; i < len; i++)
    {
        crc ^= data[i];
        for (j = 0u; j < 8u; j++)
        {
            crc = (crc & 0x80u)
                  ? (uint8_t)((crc << 1) ^ UART_CRC_POLY)
                  : (uint8_t)(crc << 1);
        }
    }
    return crc;
}


/* -----------------------------------------------------------------------------
   PESEC MATRIX BUILDER
   Verbatim port of tx_ecc.c::Init_PESEC_Matrices() so the Matrix A and D
   column orderings on the Mega side are byte-for-byte identical to what
   the receiver builds in rx_ecc.c::rx_init_pesec_matrices().
----------------------------------------------------------------------------- */
static void init_pesec_matrices(const uint8_t *m_sizes, uint8_t num_blocks)
{
    uint8_t i, b, val, is_in_d;
    uint8_t total_m = 0;
    uint8_t current_col_offset = 0;
    uint8_t max_syndrome;

    pesec_num_blocks = num_blocks;
    pesec_num_d_cols = 0;
    pesec_num_a_cols = 0;

    for (b = 0; b < num_blocks; b++)
    {
        pesec_bit_offsets[b] = total_m;
        pesec_col_offsets[b] = current_col_offset;
        pesec_chunk_masks[b] = ((1u << m_sizes[b]) - 1u) << total_m;

        for (i = 1u; i < (1u << m_sizes[b]); i++)
        {
            PESEC_MAT_D[pesec_num_d_cols++] = (uint8_t)(i << total_m);
        }

        total_m            += m_sizes[b];
        current_col_offset += ((1u << m_sizes[b]) - 1u);
    }

    max_syndrome = (uint8_t)((1u << total_m) - 1u);

    for (val = 1u; val <= max_syndrome; val++)
    {
        is_in_d = 0u;
        for (i = 0u; i < pesec_num_d_cols; i++)
        {
            if (PESEC_MAT_D[i] == val) { is_in_d = 1u; break; }
        }
        if (!is_in_d)
        {
            PESEC_MAT_A[pesec_num_a_cols++] = val;
        }
    }
}


/* -----------------------------------------------------------------------------
   PESEC SYNDROME / DELTA / PARITY HELPERS   (ported from tx_ecc.c)
----------------------------------------------------------------------------- */
static uint8_t calc_pesec_synd(uint16_t reg_val,
                               const uint8_t *mat_ptr,
                               uint8_t cols)
{
    uint8_t synd = 0u;
    uint8_t idx;

    for (idx = 0u; idx < cols; idx++)
    {
        if ((reg_val >> idx) & 1u) synd ^= mat_ptr[idx];
    }
    return synd;
}

static uint16_t solve_pesec_delta(uint8_t synd_gap)
{
    uint16_t delta_vec = 0u;
    uint8_t  b, chunk, val;

    for (b = 0u; b < pesec_num_blocks; b++)
    {
        chunk = synd_gap & pesec_chunk_masks[b];
        if (chunk)
        {
            val = (uint8_t)(chunk >> pesec_bit_offsets[b]);
            delta_vec |= ((uint16_t)1u << (pesec_col_offsets[b] + val - 1u));
        }
    }
    return delta_vec;
}

static uint8_t parity16(uint16_t v)
{
    uint8_t x = (uint8_t)(v >> 8) ^ (uint8_t)v;
    x ^= x >> 4;
    x ^= x >> 2;
    x ^= x >> 1;
    return x & 1u;
}


/* -----------------------------------------------------------------------------
   H1 SYNDROME + MINIMAL-W  (ported from tx_encoder.c)
----------------------------------------------------------------------------- */
static uint8_t compute_syndrome_from_bus(uint16_t bus_state)
{
    uint8_t  synd = 0u;
    uint8_t  col_idx;
    uint16_t temp = bus_state & BUS_STATE_MASK;

    for (col_idx = 1u; col_idx <= HAMMING_N; col_idx++)
    {
        if (temp & 1u) synd ^= col_idx;
        temp >>= 1;
    }
    return synd;
}

static uint16_t find_minimal_w(uint8_t s_target)
{
    s_target &= 0x0Fu;
    if (s_target == 0u) return 0u;
    return ((uint16_t)1u << (s_target - 1u));
}


/* -----------------------------------------------------------------------------
   ECC + process_nibble  (ports tx_ecc.c::ECC and tx_encoder.c::process_nibble)

   Every call advances the encoder state by one nibble and refreshes the
   redundancy + overall parity exactly like the ADuC TX firmware would.
----------------------------------------------------------------------------- */
static void run_ecc(void)
{
    uint8_t  synd_data = calc_pesec_synd(current_bus_state,
                                         PESEC_MAT_A, pesec_num_a_cols);
    uint8_t  synd_red  = calc_pesec_synd(pesec_redundancy_reg,
                                         PESEC_MAT_D, pesec_num_d_cols);
    uint16_t delta     = solve_pesec_delta(synd_data ^ synd_red);

    pesec_redundancy_reg ^= delta;
    pesec_redundancy_reg &= PESEC_RED_MASK;

    pesec_overall_parity = parity16(current_bus_state    & BUS_STATE_MASK)
                         ^ parity16(pesec_redundancy_reg & PESEC_RED_MASK);
}

static void process_nibble(uint8_t nibble)
{
    uint8_t  s_old    = compute_syndrome_from_bus(current_bus_state);
    uint8_t  s_target = (uint8_t)((nibble & 0x0Fu) ^ s_old);
    uint16_t w        = find_minimal_w(s_target);

    current_bus_state ^= w;
    current_bus_state &= BUS_STATE_MASK;

    run_ecc();
}


/* -----------------------------------------------------------------------------
   FRAME EMISSION
   Packs (current_bus_state, pesec_redundancy_reg, pesec_overall_parity)
   into the 4-byte payload, CRCs it, and shoves it out on USART1.
----------------------------------------------------------------------------- */
static void emit_frame(void)
{
    uint8_t payload[4];
    uint8_t crc;

    payload[0] = (uint8_t)(current_bus_state & 0xFFu);
    payload[1] = (uint8_t)(((current_bus_state >> 8) & 0x7Fu)
                           | ((pesec_overall_parity & 0x01u) << 7));
    payload[2] = (uint8_t)(pesec_redundancy_reg & 0xFFu);
    payload[3] = (uint8_t)((pesec_redundancy_reg >> 8) & 0x03u);

    crc = crc8(payload, 4u);

    uart1_write_byte(UART_SOF);
    uart1_write_byte(payload[0]);
    uart1_write_byte(payload[1]);
    uart1_write_byte(payload[2]);
    uart1_write_byte(payload[3]);
    uart1_write_byte(crc);
}


/* -----------------------------------------------------------------------------
   BUS-RESET FRAME
   All-zero 26-bit word (data=0, red=0, parity=0).  The RX detects this
   via rx_is_bus_reset() and resets its decoder / H1 state.  Sending it
   at the start of every pass guarantees a clean starting condition even
   if the previous pass was truncated or the ADuC missed frames.
----------------------------------------------------------------------------- */
static void emit_reset_frame(void)
{
    current_bus_state    = 0;
    pesec_redundancy_reg = 0;
    pesec_overall_parity = 0;
    emit_frame();
}


/* -----------------------------------------------------------------------------
   CHARACTER ENCODER
   Splits a byte into high then low nibble, runs each through the full
   H1 + PESEC pipeline, and emits a frame after each.  Two frames per
   ASCII byte -- matching the TX's tx_handler() behaviour.
----------------------------------------------------------------------------- */
static void encode_char(uint8_t ch)
{
    process_nibble((uint8_t)((ch >> 4) & 0x0Fu));
    emit_frame();
    delay(5);                           /* let RX main loop drain */

    process_nibble((uint8_t)(ch & 0x0Fu));
    emit_frame();
    delay(5);
}


/* =============================================================================
   ARDUINO ENTRY POINTS
============================================================================= */

void setup(void)
{
    uart1_init();
    init_pesec_matrices(pesec_config, sizeof(pesec_config));

    /* Give the ADuC841 time to finish booting and print "RX-READY\r\n"
       on its own UART so the PuTTY trace stays readable. */
    delay(1000);
}

void loop(void)
{
    static const char MSG[] = "GuCciGaNg";
    uint8_t i;

    /* Start each pass with a bus-reset to clear the RX pipeline. */
    emit_reset_frame();
    delay(10);

    for (i = 0u; MSG[i] != '\0'; i++)
    {
        encode_char((uint8_t)MSG[i]);
    }

    /* Hold quiet for 3 s so the output is easy to read between passes. */
    delay(3000);
}
