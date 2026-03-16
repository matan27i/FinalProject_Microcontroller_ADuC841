/* =============================================================================
   File: bridge_mega2560.ino
   Stage 2: Arduino Mega 2560 -- Parallel-to-SPI Bridge
   =====================================================================

   PURPOSE:
     Captures the 25-bit encoded word (15 data + 10 ECC) from the TX
     stage's latched 74HC595 shift-register outputs and forwards it to
     the RX ADuC841 via hardware SPI.

   TIMING MODEL:
     The TX ADuC841 shifts 25 bits at ~11.2 us/bit = ~280 us per word.
     After shifting, it pulses RCLK (latch) high, making the 25 parallel
     outputs stable.  This sketch detects the RCLK rising edge via a
     hardware interrupt, captures all 25 bits using direct port reads
     (~4 clock cycles total), then transmits 4 bytes via SPI at 4 MHz
     (~8 us total).  The entire capture-to-transmit latency is well
     under the ~280 us inter-word period.

   BUS WIRING -- 25-BIT PARALLEL INPUT:
   ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
     The TX 74HC595 daisy chain presents 25 latched outputs.  Connect
     them to the Mega as follows:

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
   ~~~~~~~~~~~~~~~~~
     TX RCLK (latch)     Pin 2           PE4 / INT4     Rising-edge ISR

   SPI MASTER OUTPUT (to RX ADuC841):
   ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
     MOSI                Pin 51          PB2            SPI data out
     SCK                 Pin 52          PB1            SPI clock out
     SS                  Pin 53          PB0            Slave select (active low)

     Connect Mega pin 51 -> ADuC841 MOSI (SPI data in)
     Connect Mega pin 52 -> ADuC841 SCLOCK (SPI clock in)
     Connect Mega pin 53 -> ADuC841 /SS (SPI slave select)
                         AND ADuC841 P3.2 / INT0 (sync edge detect)

   SPI FRAME FORMAT (4 bytes, MSB first within each byte):
   ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
     Byte 0:  H1 data bits  [7:0]     (PINA direct)
     Byte 1:  H1 data bits [14:8]     (PINC & 0x7F, bit 7 unused = 0)
     Byte 2:  ECC red bits  [7:0]     (PINK direct)
     Byte 3:  ECC red bits  [9:8]     (PINF & 0x03, bits 7-2 unused = 0)

   ============================================================================= */

#include <avr/io.h>
#include <avr/interrupt.h>

/* ---------------------------------------------------------------------------
   Captured bus data -- written by ISR, read by main loop.
   Declared volatile because shared between ISR and main context.
   ---------------------------------------------------------------------------
*/
static volatile uint8_t cap_data_lo;    /* H1 data [7:0]   from PINA */
static volatile uint8_t cap_data_hi;    /* H1 data [14:8]  from PINC */
static volatile uint8_t cap_ecc_lo;     /* ECC red [7:0]   from PINK */
static volatile uint8_t cap_ecc_hi;     /* ECC red [9:8]   from PINF */
static volatile uint8_t cap_ready;      /* 1 = new capture available */


/* ---------------------------------------------------------------------------
   ISR: INT4 (Pin 2) -- RCLK Rising Edge
   ---------------------------------------------------------------------------
   Fires when the TX latches new data onto the 74HC595 parallel outputs.
   Captures all 25 bits using four direct port reads.  Each PINx read
   is a single AVR instruction (1 clock cycle = 62.5 ns at 16 MHz).
   Total capture time: ~4 cycles = ~250 ns.

   IMPORTANT: The 74HC595 output-register propagation delay (tPHL/tPLH)
   is typically 10-25 ns, well within the time between the latch edge
   and the first port read (~125 ns including ISR entry overhead).
   ---------------------------------------------------------------------------
*/
ISR(INT4_vect)
{
    /* Snapshot all ports in rapid succession.
       Order: data low, data high, ECC low, ECC high.
       Minimal instructions between reads to ensure temporal coherence. */
    cap_data_lo = PINA;              /* PA0-PA7: H1 data [7:0]  */
    cap_data_hi = PINC & 0x7Fu;     /* PC0-PC6: H1 data [14:8] */
    cap_ecc_lo  = PINK;             /* PK0-PK7: ECC red [7:0]  */
    cap_ecc_hi  = PINF & 0x03u;    /* PF0-PF1: ECC red [9:8]  */
    cap_ready   = 1u;
}


/* ---------------------------------------------------------------------------
   spi_master_init
   ---------------------------------------------------------------------------
   Configures the ATmega2560 hardware SPI as master.

   Register: SPCR (SPI Control Register)
     Bit 7  SPIE = 0  (no SPI interrupt)
     Bit 6  SPE  = 1  (SPI enable)
     Bit 5  DORD = 0  (MSB first)
     Bit 4  MSTR = 1  (Master mode)
     Bit 3  CPOL = 0  (Clock idle low -- SPI Mode 0)
     Bit 2  CPHA = 0  (Sample on leading edge -- SPI Mode 0)
     Bit 1-0 SPR  = 00 (fosc/4 = 4 MHz at 16 MHz system clock)

   Register: SPSR (SPI Status Register)
     Bit 0  SPI2X = 0 (no double speed; 4 MHz is reliable for
                        inter-board wiring up to ~15 cm)

   Pin directions (Port B):
     PB0 (/SS, pin 53): OUTPUT -- driven as slave-select to ADuC841.
     PB1 (SCK,  pin 52): OUTPUT -- set automatically by SPI hardware.
     PB2 (MOSI, pin 51): OUTPUT -- set automatically by SPI hardware.
     PB3 (MISO, pin 50): INPUT  -- not used (slave has no return data).
   ---------------------------------------------------------------------------
*/
static void spi_master_init(void)
{
    /* Set SS, SCK, MOSI as outputs; MISO as input. */
    DDRB |= (1u << DDB0) | (1u << DDB1) | (1u << DDB2);
    DDRB &= ~(1u << DDB3);

    /* SS idle high (slave deselected). */
    PORTB |= (1u << PB0);

    /* Enable SPI, Master mode, Mode 0, fosc/4. */
    SPCR = (1u << SPE) | (1u << MSTR);
    SPSR &= ~(1u << SPI2X);
}


/* ---------------------------------------------------------------------------
   spi_send_byte
   ---------------------------------------------------------------------------
   Blocking single-byte SPI transmit.  Writes to SPDR and spins until
   the SPIF flag indicates transfer complete (~2 us at 4 MHz).
   ---------------------------------------------------------------------------
*/
static inline void spi_send_byte(uint8_t b)
{
    SPDR = b;
    while (!(SPSR & (1u << SPIF)))
        ;
    /* Reading SPSR (above) then SPDR clears SPIF per datasheet. */
    (void)SPDR;
}


/* ---------------------------------------------------------------------------
   spi_transmit_frame
   ---------------------------------------------------------------------------
   Sends one 4-byte SPI frame to the RX ADuC841.

   Protocol:
     1. Assert /SS low  (PB0 = 0)  -- ADuC841 SPI slave is selected,
        and its INT0 falling-edge ISR resets the byte counter.
     2. Clock out 4 bytes (data_lo, data_hi, ecc_lo, ecc_hi).
     3. Deassert /SS high (PB0 = 1) -- transfer complete.

   Total SPI time: 4 bytes * 8 bits * 250 ns/bit = 8 us.
   With SS assertion/deassertion overhead: ~9 us total.
   ---------------------------------------------------------------------------
*/
static void spi_transmit_frame(uint8_t d0, uint8_t d1,
                               uint8_t d2, uint8_t d3)
{
    /* Assert /SS low -- select slave, reset its byte counter. */
    PORTB &= ~(1u << PB0);

    /* Brief delay (~125 ns) to allow slave /SS recognition.
       Two NOPs at 16 MHz = 125 ns, sufficient for ADuC841. */
    __asm__ __volatile__("nop\n\t" "nop\n\t");

    spi_send_byte(d0);     /* Byte 0: H1 data [7:0]  */
    spi_send_byte(d1);     /* Byte 1: H1 data [14:8] */
    spi_send_byte(d2);     /* Byte 2: ECC red [7:0]  */
    spi_send_byte(d3);     /* Byte 3: ECC red [9:8]  */

    /* Deassert /SS high -- end of frame. */
    PORTB |= (1u << PB0);
}


/* ---------------------------------------------------------------------------
   configure_bus_inputs
   ---------------------------------------------------------------------------
   Sets all 25 bus-input pins as inputs with no pull-ups.
   The 74HC595 outputs are push-pull, so pull-ups are unnecessary.
   ---------------------------------------------------------------------------
*/
static void configure_bus_inputs(void)
{
    /* PORTA (PA0-PA7, pins 22-29): H1 data [7:0] -- all input, no pullup. */
    DDRA  = 0x00u;
    PORTA = 0x00u;

    /* PORTC (PC0-PC6, pins 37-31): H1 data [14:8] -- 7 bits input.
       PC7 (pin 30) is unused; set as input with no pullup for safety. */
    DDRC  = 0x00u;
    PORTC = 0x00u;

    /* PORTK (PK0-PK7, pins A8-A15): ECC red [7:0] -- all input. */
    DDRK  = 0x00u;
    PORTK = 0x00u;

    /* PORTF bits 0-1 (PF0=A0, PF1=A1): ECC red [9:8] -- input.
       Preserve other PORTF bits (analog reference, etc). */
    DDRF  &= ~0x03u;
    PORTF &= ~0x03u;
}


/* ---------------------------------------------------------------------------
   configure_rclk_interrupt
   ---------------------------------------------------------------------------
   Configures INT4 (pin 2, PE4) for rising-edge triggering.

   EICRB register controls INT4-INT7 sense:
     ISC41:ISC40 = 11 -> rising edge triggers INT4.

   EIMSK register enables INT4:
     Bit 4 (INT4) = 1 -> enable.
   ---------------------------------------------------------------------------
*/
static void configure_rclk_interrupt(void)
{
    /* PE4 (pin 2) as input with no pull-up.
       The RCLK line is actively driven by the TX 74HC595 latch. */
    DDRE  &= ~(1u << DDE4);
    PORTE &= ~(1u << PE4);

    /* INT4: rising edge. EICRB bits [1:0] = ISC41:ISC40. */
    EICRB = (EICRB & ~0x03u) | 0x03u;

    /* Enable INT4 in the external interrupt mask. */
    EIMSK |= (1u << INT4);
}


/* ==========================================================================
   ARDUINO ENTRY POINTS
   ========================================================================== */

void setup(void)
{
    cli();  /* Disable interrupts during configuration. */

    configure_bus_inputs();
    spi_master_init();
    configure_rclk_interrupt();

    /* Clear any pending state. */
    cap_ready = 0u;

    sei();  /* Enable global interrupts. */
}


void loop(void)
{
    /* Non-blocking: check if a new capture is available. */
    if (cap_ready)
    {
        /* Snapshot the volatile ISR data into local copies.
           Disable INT4 briefly to prevent a new capture from
           overwriting the buffer mid-read. */
        uint8_t d0, d1, d2, d3;

        cli();
        d0 = cap_data_lo;
        d1 = cap_data_hi;
        d2 = cap_ecc_lo;
        d3 = cap_ecc_hi;
        cap_ready = 0u;
        sei();

        /* Transmit the 4-byte frame to the RX ADuC841 via SPI. */
        spi_transmit_frame(d0, d1, d2, d3);
    }
}
