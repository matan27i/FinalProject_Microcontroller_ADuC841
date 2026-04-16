/* =============================================================================
   File: bridge_mega2560.ino
   Stage 2: Arduino Mega 2560 -- Parallel-to-SPI Bridge
   =====================================================================

   PURPOSE:
     Captures the 26-bit encoded word (15 data + 10 ECC + 1 overall parity)
     from the TX stage's latched 74HC595 shift-register outputs and forwards
     it to the RX ADuC841 via hardware SPI.

   FIX LOG:
     [W3] SPI clock reduced from fosc/4 (4 MHz) to fosc/16 (1 MHz) for
          reliable ADuC841 slave communication.  The ADuC841 at 11.0592 MHz
          requires SCK <= fCORE/4 ~= 2.76 MHz for safe SPI slave sampling;
          1 MHz provides comfortable margin.
     [F3] Replaced single-variable ISR-to-loop handoff with an 8-slot
          interrupt-safe ring buffer.  Prevents frame loss when the SPI
          transmit time (~34 us at 1 MHz) overlaps a new RCLK edge under
          burst conditions.
     [W4] SEC+DED upgrade: reads the 26th bit (overall parity) from PC7
          (Mega pin 30) and packs it into bit 7 of SPI byte 1.

   BUS WIRING -- 26-BIT PARALLEL INPUT:
   ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
     Signal              Mega Pin(s)     AVR Port       Bits
     ~~~~~~~~~~~~~~~~~~  ~~~~~~~~~~~~~~  ~~~~~~~~~~~~   ~~~~~~~~~
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

     Overall parity      Pin 30          PC7            [7]  <-- NEW (W4)

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

   SPI MASTER OUTPUT (to RX ADuC841):
     MOSI                Pin 51          PB2            SPI data out
     SCK                 Pin 52          PB1            SPI clock out
     SS                  Pin 53          PB0            Slave select (active low)

   SPI FRAME FORMAT (4 bytes, MSB first within each byte):
   ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
     Byte 0:  H1 data bits  [7:0]       (PINA direct)
     Byte 1:  bit 7 = overall parity    (PINC bit 7)      <-- NEW (W4)
              bits [6:0] = H1 data bits [14:8]
     Byte 2:  ECC red bits  [7:0]       (PINK direct)
     Byte 3:  ECC red bits  [9:8]       (PINF & 0x03)
   ============================================================================= */

#include <avr/io.h>
#include <avr/interrupt.h>


/* =========================================================================
   [F3] INTERRUPT-SAFE RING BUFFER
   =========================================================================
   Capacity: 8 frames.  Must be a power of 2 for fast modulo via bitmask.

   The ISR (producer) writes to ring_buf[ring_head] and advances ring_head.
   The main loop (consumer) reads from ring_buf[ring_tail] and advances
   ring_tail.  This single-producer / single-consumer pattern is inherently
   safe for the data payload (disjoint indices).  cli/sei guards are added
   around the consumer's read-and-advance as an extra safety measure to
   prevent any theoretical reordering of volatile accesses.
   ========================================================================= */

#define RING_CAP       8u
#define RING_MASK      (RING_CAP - 1u)   /* 0x07: fast modulo for power-of-2 */

typedef struct
{
    uint8_t d[4];     /* 4-byte SPI frame payload */
} spi_frame_t;

static volatile spi_frame_t ring_buf[RING_CAP];
static volatile uint8_t     ring_head = 0;   /* Next write position (ISR)  */
static volatile uint8_t     ring_tail = 0;   /* Next read position  (loop) */


/* ---------------------------------------------------------------------------
   ISR: INT4 (Pin 2) -- RCLK Rising Edge
   ---------------------------------------------------------------------------
   [F3] Pushes a complete 4-byte frame into the ring buffer.  If the
   buffer is full (ISR has lapped the consumer), the frame is dropped.

   [W4] Reads all 8 bits of PINC: bits [6:0] = data[14:8],
   bit 7 = overall parity.  The parity bit is packed into SPI byte 1
   bit 7, occupying the previously-unused high bit.

   Port read timing: 4 PINx reads = 4 AVR cycles = 250 ns at 16 MHz.
   ---------------------------------------------------------------------------
*/
ISR(INT4_vect)
{
    uint8_t next_head = (ring_head + 1u) & RING_MASK;

    if (next_head == ring_tail)
    {
        return;   /* Ring full -- drop this frame. */
    }

    /* Snapshot all ports in rapid succession.
       PINC is read as a full byte: bits [6:0] = data[14:8],
       bit 7 = overall parity.  No masking here; the RX ADuC841
       separates the fields when it unpacks the SPI frame. */
    ring_buf[ring_head].d[0] = PINA;               /* data [7:0]          */
    ring_buf[ring_head].d[1] = PINC;               /* data[14:8] + parity */
    ring_buf[ring_head].d[2] = PINK;               /* ECC red [7:0]       */
    ring_buf[ring_head].d[3] = PINF & 0x03u;       /* ECC red [9:8]       */

    ring_head = next_head;
}


/* ---------------------------------------------------------------------------
   spi_master_init
   ---------------------------------------------------------------------------
   [W3] SPI clock = fosc/16 = 16 MHz / 16 = 1 MHz.

   SPCR bit field:
     SPE=1, MSTR=1, CPOL=0, CPHA=0 (Mode 0), DORD=0 (MSB first).
     SPR1=0, SPR0=1 -> base divisor 16.

   SPSR:
     SPI2X=0 -> no doubling.  Combined with SPR=01 gives fosc/16.

   The ADuC841 slave samples on the rising SCK edge (Mode 0).
   Max reliable slave SCK is fCORE/4 = 11.0592/4 = 2.76 MHz.
   1 MHz provides ~2.8x margin.
   ---------------------------------------------------------------------------
*/
static void spi_master_init(void)
{
    DDRB |= (1u << DDB0) | (1u << DDB1) | (1u << DDB2);
    DDRB &= ~(1u << DDB3);

    PORTB |= (1u << PB0);        /* SS idle high. */

    /* [W3] fosc/16: SPR1=0, SPR0=1, SPI2X=0. */
    SPCR = (1u << SPE) | (1u << MSTR) | (1u << SPR0);
    SPSR &= ~(1u << SPI2X);
}


/* ---------------------------------------------------------------------------
   spi_send_byte
   ---------------------------------------------------------------------------
   Blocking single-byte SPI transmit.  At 1 MHz, each byte takes 8 us.
   ---------------------------------------------------------------------------
*/
static inline void spi_send_byte(uint8_t b)
{
    SPDR = b;
    while (!(SPSR & (1u << SPIF)))
        ;
    (void)SPDR;
}


/* ---------------------------------------------------------------------------
   spi_transmit_frame
   ---------------------------------------------------------------------------
   Sends one 4-byte SPI frame to the RX ADuC841.
   Total SPI time at 1 MHz: ~34 us (4 * 8 us + SS overhead).
   ---------------------------------------------------------------------------
*/
static void spi_transmit_frame(uint8_t d0, uint8_t d1,
                               uint8_t d2, uint8_t d3)
{
    PORTB &= ~(1u << PB0);       /* Assert /SS low. */

    __asm__ __volatile__("nop\n\t" "nop\n\t");

    spi_send_byte(d0);            /* Byte 0: data [7:0]                */
    spi_send_byte(d1);            /* Byte 1: data [14:8] | parity<<7   */
    spi_send_byte(d2);            /* Byte 2: ECC red [7:0]             */
    spi_send_byte(d3);            /* Byte 3: ECC red [9:8]             */

    PORTB |= (1u << PB0);        /* Deassert /SS high. */
}


/* ---------------------------------------------------------------------------
   configure_bus_inputs
   ---------------------------------------------------------------------------
   [W4] PC7 (pin 30) is now an input for the overall parity bit.
   All other port directions are unchanged from the 25-bit version.
   ---------------------------------------------------------------------------
*/
static void configure_bus_inputs(void)
{
    DDRA  = 0x00u;    PORTA = 0x00u;   /* PA: data [7:0]                    */
    DDRC  = 0x00u;    PORTC = 0x00u;   /* PC: data [14:8] + parity on PC7   */
    DDRK  = 0x00u;    PORTK = 0x00u;   /* PK: ECC red [7:0]                 */
    DDRF  &= ~0x03u;  PORTF &= ~0x03u; /* PF0-1: ECC red [9:8]             */
}


/* ---------------------------------------------------------------------------
   configure_rclk_interrupt
   ---------------------------------------------------------------------------
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
    spi_master_init();
    configure_rclk_interrupt();

    ring_head = 0;
    ring_tail = 0;

    sei();
}


void loop(void)
{
    /* [F3] Drain one frame from the ring buffer per iteration.
       cli/sei guard the read-and-advance to prevent any ISR
       interaction during the multi-byte copy. */
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

        spi_transmit_frame(d0, d1, d2, d3);
    }
}
