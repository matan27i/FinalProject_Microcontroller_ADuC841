/* =============================================================================
   File: bridge_mega2560.ino
   Stage 2: Arduino Mega 2560 -- Parallel-to-UART Bridge
   =====================================================================

   PURPOSE:
     Captures the 26-bit encoded word (15 data + 10 ECC + 1 overall parity)
     from the TX stage's latched 74HC595 shift-register outputs and forwards
     it to the RX ADuC841 over asynchronous UART on USART1.

   WHY UART (MOTIVATION FOR THIS REFACTOR):
     The previous revision used hardware SPI master.  SPI relies on a
     continuously clocked SCK line and a dedicated /SS strobe; on long
     jumper wires in a noisy environment, any glitch on SCK causes the
     slave to shift a spurious bit and desync the entire frame for the
     rest of the session.  Asynchronous UART is self-clocked (start bit
     resyncs the receiver on every byte), so transient noise corrupts at
     most one byte rather than permanently shifting the stream.

     To make the link robust against isolated byte corruption we wrap
     each 4-byte payload in a 6-byte framed packet with a sync header
     and a CRC-8.  The RX drops frames whose CRC does not match, which
     combined with the PESEC SEC+DED already present in the codeword
     gives belt-and-braces protection.

   WIRE FRAME FORMAT (6 bytes, LSB-first UART, 8N1):
   ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
     Byte 0:  0xAA                       Start-of-frame sync byte
     Byte 1:  H1 data bits  [7:0]        (from PINA)
     Byte 2:  bit 7 = overall parity     (from PINC bit 7)
              bits [6:0] = H1 data [14:8]
     Byte 3:  ECC red bits  [7:0]        (from PINK)
     Byte 4:  ECC red bits  [9:8]        (from PINF & 0x03)
     Byte 5:  CRC-8 over bytes 1..4      (poly 0x07, init 0x00)

     A SOF of 0xAA (10101010b) keeps the receiver's byte aligner busy
     through the first stop-bit / idle time when the line was floating.
     A hunted SOF plus CRC-8 gives worst-case resync in one frame (520 us
     at 115200 baud).

   BAUD RATE:
     115200 baud, 8N1, U2X1 = 1 (double-speed mode).  At F_CPU = 16 MHz
     this gives an actual rate of 117647 baud (+2.1% error).  The ADuC841
     at 11.0592 MHz core uses Timer 3 at DIV=2 + T3FD=0x20 for an exact
     115200.  Combined error is ~2.1%, within UART tolerance of +-2.5%.

   BUS WIRING -- 26-BIT PARALLEL INPUT (unchanged from SPI version):
     Signal              Mega Pin(s)     AVR Port       Bits
     H1 data bit  0      Pin 22          PA0            [0]
     H1 data bit  1      Pin 23          PA1            [1]
     H1 data bit  2      Pin 24          PA2            [2]
     H1 data bit  3      Pin 25          PA3            [3]
     H1 data bit  4      Pin 26          PA4            [4]
     H1 data bit  5      Pin 27          PA5            [5]
     H1 data bit  6      Pin 28          PA6            [6]
     H1 data bit  7      Pin 29          PA7            [7]

     H1 data bit  8      Pin 37          PC0            [0]
     H1 data bit  9      Pin 36          PC1            [1]
     H1 data bit 10      Pin 35          PC2            [2]
     H1 data bit 11      Pin 34          PC3            [3]
     H1 data bit 12      Pin 33          PC4            [4]
     H1 data bit 13      Pin 32          PC5            [5]
     H1 data bit 14      Pin 31          PC6            [6]
     Overall parity      Pin 30          PC7            [7]

     ECC red bit  0      Pin A8  (62)    PK0            [0]
     ECC red bit  1      Pin A9  (63)    PK1            [1]
     ECC red bit  2      Pin A10 (64)    PK2            [2]
     ECC red bit  3      Pin A11 (65)    PK3            [3]
     ECC red bit  4      Pin A12 (66)    PK4            [4]
     ECC red bit  5      Pin A13 (67)    PK5            [5]
     ECC red bit  6      Pin A14 (68)    PK6            [6]
     ECC red bit  7      Pin A15 (69)    PK7            [7]

     ECC red bit  8      Pin A0  (54)    PF0            [0]
     ECC red bit  9      Pin A1  (55)    PF1            [1]

   CONTROL SIGNALS:
     TX RCLK (latch)     Pin 2           PE4 / INT4     Rising-edge ISR

   UART MASTER OUTPUT (to RX ADuC841 RxD / P3.0):
     TX1                 Pin 18          PD3 (USART1 TXD1)
     GND                 common ground is required between Mega and ADuC
   ============================================================================= */

#include <avr/io.h>
#include <avr/interrupt.h>


/* ---- configuration ---- */
#define UART_SOF       0xAAu
#define UART_BAUD      115200UL

/* Double-speed mode: UBRR = (F_CPU / (8 * BAUD)) - 1.
   For F_CPU=16e6, BAUD=115200: UBRR = 16.36 -> 16 (actual 117647, +2.1%). */
#define UART_UBRR_VAL  ((F_CPU / (8UL * UART_BAUD)) - 1UL)


/* =========================================================================
   INTERRUPT-SAFE RING BUFFER (kept from SPI version)
   =========================================================================
   Capacity: 8 frames.  Single-producer (INT4 ISR) / single-consumer
   (main loop) pattern.  Power-of-2 size allows fast bitmask modulo.
   ========================================================================= */

#define RING_CAP       8u
#define RING_MASK      (RING_CAP - 1u)

typedef struct
{
    uint8_t d[4];     /* 4-byte payload captured from shift registers */
} frame_t;

static volatile frame_t ring_buf[RING_CAP];
static volatile uint8_t ring_head = 0;   /* Next write (ISR)  */
static volatile uint8_t ring_tail = 0;   /* Next read  (loop) */


/* ---------------------------------------------------------------------------
   ISR: INT4 (Pin 2) -- RCLK Rising Edge
   ---------------------------------------------------------------------------
   Pushes a complete 4-byte payload into the ring buffer.  If the buffer
   is full, the frame is dropped.  PINC bit 7 carries the overall parity;
   we keep it untouched in byte 1 so the RX unpacks the fields itself.
*/
ISR(INT4_vect)
{
    uint8_t next_head = (ring_head + 1u) & RING_MASK;

    if (next_head == ring_tail)
    {
        return;
    }

    ring_buf[ring_head].d[0] = PINA;               /* data [7:0]          */
    ring_buf[ring_head].d[1] = PINC;               /* data[14:8] + parity */
    ring_buf[ring_head].d[2] = PINK;               /* ECC red [7:0]       */
    ring_buf[ring_head].d[3] = PINF & 0x03u;       /* ECC red [9:8]       */

    ring_head = next_head;
}


/* ---------------------------------------------------------------------------
   CRC-8 (polynomial 0x07, init 0x00, no reflection)
   ---------------------------------------------------------------------------
   Standard CRC-8/SMBUS.  Strong enough to catch all single-bit errors
   and every burst up to 8 bits within a 4-byte payload.  Runs in ~130
   AVR cycles for 4 bytes = ~8 us at 16 MHz: negligible overhead next to
   the 520 us UART transmission time.
*/
static uint8_t crc8(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0u;
    uint8_t i, j;

    for (i = 0u; i < len; i++)
    {
        crc ^= data[i];
        for (j = 0u; j < 8u; j++)
        {
            if (crc & 0x80u)
            {
                crc = (uint8_t)((crc << 1) ^ 0x07u);
            }
            else
            {
                crc = (uint8_t)(crc << 1);
            }
        }
    }

    return crc;
}


/* ---------------------------------------------------------------------------
   uart1_init -- USART1 at 115200 8N1, TX-only, polled
   ---------------------------------------------------------------------------
   USART1 was chosen because USART0 is shared with the USB-serial bridge
   used for sketch upload and debug.  TX-only because the bridge never
   needs to read a reply from the RX ADuC841.
*/
static void uart1_init(void)
{
    /* Double-speed mode for lower baud error at 115200. */
    UCSR1A = (1u << U2X1);

    /* Baud rate register (16-bit). */
    UBRR1H = (uint8_t)(UART_UBRR_VAL >> 8);
    UBRR1L = (uint8_t)(UART_UBRR_VAL);

    /* 8-bit character size: UCSZ12=0 here, UCSZ11=1, UCSZ10=1 below. */
    UCSR1C = (1u << UCSZ11) | (1u << UCSZ10);

    /* Enable transmitter only; no RX, no interrupts. */
    UCSR1B = (1u << TXEN1);
}


/* ---------------------------------------------------------------------------
   uart1_send_byte -- blocking, polled single-byte write
   ---------------------------------------------------------------------------
   At 115200 baud (actually 117647), a byte takes ~85 us on the wire.  The
   UDRE1 flag goes high as soon as the previous byte has moved into the
   shift register, so writes can be pipelined with ~0 idle time between
   bytes in a frame.
*/
static inline void uart1_send_byte(uint8_t b)
{
    while (!(UCSR1A & (1u << UDRE1)))
    {
        /* busy-wait for transmit data register empty */
    }
    UDR1 = b;
}


/* ---------------------------------------------------------------------------
   uart1_transmit_frame -- send one 6-byte framed packet
   ---------------------------------------------------------------------------
   Frame layout (see file header comment for full definition):
     [SOF][d0][d1][d2][d3][CRC]
   Total wire time at 115200 baud: 6 * 10 bits / 115200 = ~520 us.
*/
static void uart1_transmit_frame(uint8_t d0, uint8_t d1,
                                 uint8_t d2, uint8_t d3)
{
    uint8_t payload[4];
    uint8_t crc;

    payload[0] = d0;
    payload[1] = d1;
    payload[2] = d2;
    payload[3] = d3;

    crc = crc8(payload, 4u);

    uart1_send_byte(UART_SOF);
    uart1_send_byte(payload[0]);
    uart1_send_byte(payload[1]);
    uart1_send_byte(payload[2]);
    uart1_send_byte(payload[3]);
    uart1_send_byte(crc);
}


/* ---------------------------------------------------------------------------
   configure_bus_inputs
   ---------------------------------------------------------------------------
   PC7 is used as the overall parity input.  Unchanged from the SPI
   version: all parallel-capture ports are inputs with pull-ups disabled.
*/
static void configure_bus_inputs(void)
{
    DDRA  = 0x00u;    PORTA = 0x00u;
    DDRC  = 0x00u;    PORTC = 0x00u;
    DDRK  = 0x00u;    PORTK = 0x00u;
    DDRF  &= ~0x03u;  PORTF &= ~0x03u;
}


/* ---------------------------------------------------------------------------
   configure_rclk_interrupt -- INT4 on rising edge, Pin 2
*/
static void configure_rclk_interrupt(void)
{
    DDRE  &= ~(1u << DDE4);
    PORTE &= ~(1u << PE4);

    EICRB = (EICRB & ~0x03u) | 0x03u;   /* INT4: rising edge */
    EIMSK |= (1u << INT4);
}


/* ==========================================================================
   ARDUINO ENTRY POINTS
   ========================================================================== */

void setup(void)
{
    cli();

    configure_bus_inputs();
    uart1_init();
    configure_rclk_interrupt();

    ring_head = 0;
    ring_tail = 0;

    sei();
}


void loop(void)
{
    /* Drain one frame from the ring buffer per iteration and push it
       out as a 6-byte UART packet.  cli/sei protects the multi-byte
       copy from concurrent ISR writes. */
    if (ring_tail != ring_head)
    {
        uint8_t d0, d1, d2, d3;
        uint8_t local_tail;

        cli();
        local_tail = ring_tail;
        d0 = ring_buf[local_tail].d[0];
        d1 = ring_buf[local_tail].d[1];
        d2 = ring_buf[local_tail].d[2];
        d3 = ring_buf[local_tail].d[3];
        ring_tail = (local_tail + 1u) & RING_MASK;
        sei();

        uart1_transmit_frame(d0, d1, d2, d3);
    }
}
