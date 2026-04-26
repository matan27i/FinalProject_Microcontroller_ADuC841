/* =============================================================================
   File: tx_testbench.ino
   Mode: Continuous Bridge -- TX-system testbench
   =============================================================================

   ROLE IN THE TX-SYSTEM TESTBENCH:
     The Mega 2560 sits between the TX ADuC841's 74HC595 shift-register chain
     and the PC's USB serial monitor.  Each rising edge of RCLK (the latch
     pulse from the ADuC) snapshots the 26-bit parallel bus and pushes it
     into a small ring buffer.  The main loop drains the ring and prints a
     human-readable line per frame on USART0 (USB Serial Monitor).

     Data flow:
       Python host_sender.py
         |  USB-UART, 9600 8N1, raw bytes (one byte per character)
         v
       ADuC841 -- tx_system firmware
         |  H1 encode (15-bit) + PESEC (10-bit red) + overall parity (1-bit)
         |  serially shifted into a 4-chip 74HC595 daisy chain, latched on RCLK
         v
       4 x 74HC595  ===  26 parallel output lines  ===
         v
       Mega 2560 -- this sketch
         |  USART0, 250000 8N1, ASCII text (one line per latched frame)
         v
       PC -- Arduino IDE Serial Monitor

   OUTPUT FORMAT (one line per latched frame):
     #NNNN RAW=<26 binary chars>  D=0xHHHH P=B E=0xHHH [RESET]
       NNNN     monotonic frame counter (0001..)
       RAW=...  26 bits, MSB-first, ordered: data[14:0] | parity | red[9:0]
       D=...    parsed 15-bit H1 data word (0x0000..0x7FFF)
       P=...    parsed overall parity bit (0 or 1)
       E=...    parsed 10-bit PESEC redundancy (0x000..0x3FF)
       [RESET]  appended only when data=0, red=0, parity=0
                (i.e. the TX firmware acknowledged a '=' reset command)

   WHY 250000 baud ON USART0:
     One frame line is ~50 bytes.  At 115200 baud each line takes ~4.3 ms
     to send -- close to the back-to-back nibble-pair timing the TX produces
     when the host is sending characters with no pacing.  At 250000 baud
     each line is ~2 ms, leaving comfortable headroom.  The Arduino IDE
     Serial Monitor supports 250000 in its baud-rate dropdown; pick that
     to read the stream.

   WIRING (74HC595 daisy chain -> Mega):
     Signal              Mega Pin(s)     AVR Port       Bits
     H1 data bit  0..7   Pin 22..29      PA0..PA7       [0..7]
     H1 data bit  8..14  Pin 37..31      PC0..PC6       [0..6]
     Overall parity      Pin 30          PC7            [7]
     ECC red bit  0..7   Pin A8..A15     PK0..PK7       [0..7]
     ECC red bit  8..9   Pin A0..A1      PF0..PF1       [0..1]
     RCLK (latch)        Pin 2           PE4 / INT4     Rising-edge ISR
     GND                 GND             -              common with ADuC

   USART1 IS UNUSED in this mode -- the RX ADuC841 is not part of the
   TX-system testbench loop.
============================================================================= */

#include <avr/io.h>
#include <avr/interrupt.h>


/* -----------------------------------------------------------------------------
   RING BUFFER  (single-producer ISR / single-consumer main loop)
   ---
   Power-of-two capacity for fast bitmask modulo.  16 slots covers the
   worst-case burst (2 frames per character at host_sender's 10 ms pacing,
   while one 50-byte print at 250000 baud takes ~2 ms) with margin.
----------------------------------------------------------------------------- */
#define RING_CAP    16u
#define RING_MASK   (RING_CAP - 1u)

typedef struct
{
    uint8_t pina;     /* H1 data [7:0]                  */
    uint8_t pinc;     /* parity<<7 | H1 data [14:8]     */
    uint8_t pink;     /* PESEC redundancy [7:0]         */
    uint8_t pinf;     /* PESEC redundancy [9:8] in 1:0  */
} frame_t;

static volatile frame_t  ring_buf[RING_CAP];
static volatile uint8_t  ring_head       = 0;   /* producer */
static volatile uint8_t  ring_tail       = 0;   /* consumer */
static volatile uint16_t dropped_frames  = 0;   /* incremented when ring full */


/* -----------------------------------------------------------------------------
   ISR: INT4 -- continuous RCLK capture
   ---
   Every rising edge on pin 2 snapshots the four parallel ports into the
   next free ring slot.  If the ring is full the frame is dropped and a
   counter is bumped so the consumer can warn the user.
*/
ISR(INT4_vect)
{
    uint8_t next_head = (uint8_t)((ring_head + 1u) & RING_MASK);

    if (next_head == ring_tail)
    {
        dropped_frames++;
        return;
    }

    ring_buf[ring_head].pina = PINA;
    ring_buf[ring_head].pinc = PINC;
    ring_buf[ring_head].pink = PINK;
    ring_buf[ring_head].pinf = (uint8_t)(PINF & 0x03u);

    ring_head = next_head;
}


/* -----------------------------------------------------------------------------
   PORT / INTERRUPT SETUP
----------------------------------------------------------------------------- */
static void configure_bus_inputs(void)
{
    DDRA  = 0x00u;    PORTA = 0x00u;
    DDRC  = 0x00u;    PORTC = 0x00u;
    DDRK  = 0x00u;    PORTK = 0x00u;
    DDRF  &= ~0x03u;  PORTF &= ~0x03u;
}

static void configure_rclk_interrupt(void)
{
    DDRE  &= ~(1u << DDE4);
    PORTE &= ~(1u << PE4);

    EICRB = (uint8_t)((EICRB & ~0x03u) | 0x03u);   /* INT4: rising edge */
    EIFR  =  (1u << INTF4);                         /* clear stale pending */
    EIMSK |= (1u << INT4);
}


/* -----------------------------------------------------------------------------
   PRINT HELPERS
----------------------------------------------------------------------------- */
static void print_bin(uint16_t value, uint8_t n_bits)
{
    int8_t i;
    for (i = (int8_t)n_bits - 1; i >= 0; i--)
    {
        Serial.print((uint8_t)((value >> i) & 1u));
    }
}

static void print_hex_padded(uint16_t value, uint8_t digits)
{
    int8_t i;
    for (i = (int8_t)digits - 1; i >= 0; i--)
    {
        uint8_t nibble = (uint8_t)((value >> (i * 4)) & 0x0Fu);
        Serial.print(nibble, HEX);
    }
}

static void print_seq_padded(uint16_t seq)
{
    if (seq < 1000u) Serial.print('0');
    if (seq < 100u)  Serial.print('0');
    if (seq < 10u)   Serial.print('0');
    Serial.print(seq);
}


/* -----------------------------------------------------------------------------
   FRAME FORMATTER
----------------------------------------------------------------------------- */
static void print_frame(uint16_t seq, const frame_t *f)
{
    /* Reconstruct logical fields from the four port snapshots, using the
       same field-extraction layout the RX firmware uses in rx_hw.c. */
    uint16_t data_word = (uint16_t)f->pina
                       | ((uint16_t)(f->pinc & 0x7Fu) << 8);
    uint8_t  parity_b  = (uint8_t)((f->pinc >> 7) & 0x01u);
    uint16_t red_word  = (uint16_t)f->pink
                       | ((uint16_t)(f->pinf & 0x03u) << 8);

    Serial.print('#');
    print_seq_padded(seq);

    Serial.print(F(" RAW="));
    print_bin(data_word, 15);
    Serial.print(parity_b);
    print_bin(red_word, 10);

    Serial.print(F("  D=0x"));
    print_hex_padded(data_word, 4);
    Serial.print(F(" P="));
    Serial.print(parity_b);
    Serial.print(F(" E=0x"));
    print_hex_padded(red_word, 3);

    if (data_word == 0u && red_word == 0u && parity_b == 0u)
    {
        Serial.print(F(" [RESET]"));
    }

    Serial.println();
}


/* =============================================================================
   ARDUINO ENTRY POINTS
============================================================================= */

void setup(void)
{
    cli();
    configure_bus_inputs();
    configure_rclk_interrupt();
    ring_head      = 0;
    ring_tail      = 0;
    dropped_frames = 0;

    /* Bring up Serial and print banner BEFORE unmasking interrupts so the
       header always appears before the first captured frame. */
    Serial.begin(250000);
    while (!Serial)
    {
        /* USB-CDC enumeration wait (no-op on classic Mega). */
    }

    Serial.println();
    Serial.println(F("=== tx_testbench: continuous TX-bus sampler ==="));
    Serial.println(F("USART0 = 250000 8N1.  Set Serial Monitor to 250000 baud."));
    Serial.println(F("Streaming one line per RCLK rising edge..."));
    Serial.println();

    sei();
}

void loop(void)
{
    static uint16_t seq                  = 0;
    static uint16_t last_dropped_seen    = 0;
    frame_t snapshot;
    uint8_t local_tail;

    /* --- Drain one frame per loop iteration ---
       ring_tail is only modified here, ring_head only in the ISR.  The
       producer drops new frames when full rather than overwriting the
       consumer's slot, so the read of ring_buf[local_tail] is race-free
       even without disabling interrupts. */
    if (ring_tail != ring_head)
    {
        local_tail   = ring_tail;
        snapshot.pina = ring_buf[local_tail].pina;
        snapshot.pinc = ring_buf[local_tail].pinc;
        snapshot.pink = ring_buf[local_tail].pink;
        snapshot.pinf = ring_buf[local_tail].pinf;
        ring_tail    = (uint8_t)((local_tail + 1u) & RING_MASK);

        seq++;
        print_frame(seq, &snapshot);
    }

    /* --- Drop notifier ---
       Read the 16-bit volatile counter atomically.  If it has grown since
       last report, print a warning so the user can slow host_sender.py
       or shorten the print format. */
    {
        uint16_t cur;
        cli();
        cur = dropped_frames;
        sei();

        if (cur != last_dropped_seen)
        {
            last_dropped_seen = cur;
            Serial.print(F("[!] dropped="));
            Serial.println(cur);
        }
    }
}
