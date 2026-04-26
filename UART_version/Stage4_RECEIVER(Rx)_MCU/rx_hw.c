/*
   File: rx_hw.c

   ADuC841 UART (Mode 1) serves both paths on a single serial port:
     RxD (P3.0)  <--  USART1 TXD1 of the Arduino Mega (encoded frames)
     TxD (P3.1)  -->  USB-UART bridge to PuTTY          (decoded bytes)

   The UART controller is inherently full-duplex: receiving a frame on
   RxD and transmitting a decoded byte on TxD use disjoint hardware
   (RB and TB shift registers) and are mediated by the same ISR.

   FRAMING LOGIC:
     Incoming bytes are fed into a 3-state accumulator:
         UART_RX_WAIT_SOF     - hunt for 0xAA sync byte
         UART_RX_COLLECT      - capture the next 4 payload bytes
         UART_RX_WAIT_CRC     - verify CRC-8 over the 4 payload bytes
     Invalid frames are dropped and the accumulator returns to
     UART_RX_WAIT_SOF, so worst-case resync is one frame (~520 us).

   TX IS NOW INTERRUPT-DRIVEN:
     In the SPI revision the UART TX path was polled via rx_uart_pump().
     Enabling ES = 1 (to receive) means a pending TI flag will also
     fire the serial interrupt and cannot be left un-cleared, so TX is
     moved into the ISR.  The idle-flag tx_busy tracks whether SBUF
     currently holds an outbound byte; the first rx_send_uart_byte()
     primes SBUF directly, subsequent calls enqueue into the ring.
*/

#include "rx_system.h"


/*
   UART FRAME PROTOCOL CONSTANTS -- must match the Mega bridge
*/
#define UART_SOF            0xAAu // Start Of Frame
#define UART_PAYLOAD_LEN    4u    // 
#define UART_CRC_POLY       0x07u


/*
   UART FRAME RECEIVE STATE VARIABLES
*/

/* Staging registers: written from ISR when a complete CRC-valid frame
   has been assembled, read from main loop under EA=0 guard. */
volatile uint16_t rx_frame_data        = 0;
volatile uint16_t rx_frame_red         = 0;
volatile uint8_t  rx_frame_parity      = 0;
volatile uint8_t  rx_frame_ready       = 0;
volatile uint16_t rx_frame_crc_errors  = 0;

/* ISR-private state machine and byte accumulation buffer.  volatile
   solely to prevent the compiler from hoisting accesses out of the ISR
   (they are already single-threaded within the ISR itself). */
typedef enum
{
    UART_RX_WAIT_SOF  = 0,
    UART_RX_COLLECT   = 1,
    UART_RX_WAIT_CRC  = 2
} uart_rx_state_t;

static volatile uart_rx_state_t uart_rx_state    = UART_RX_WAIT_SOF;
static volatile uint8_t         uart_rx_count    = 0;
static uint8_t                  uart_rx_buf[UART_PAYLOAD_LEN];


/*
   UART TRANSMIT RING (interrupt-driven)
   ---
   Capacity bumped from 64 to 128 in the SPI revision to absorb bursts
   during PuTTY flow-control pauses.  Kept at 128 here.
*/
#define TX_BUF_SIZE 128u
static uint8_t xdata tx_buf[TX_BUF_SIZE];
static volatile uint8_t tx_head = 0;   /* producer: main loop */
static volatile uint8_t tx_tail = 0;   /* consumer: ISR       */

/* tx_busy = 1 when SBUF currently holds an outbound byte whose TI has
   not yet been serviced.  While tx_busy=0, the ISR will not re-fire
   on TI (we keep TI cleared) and the next rx_send_uart_byte() will
   prime SBUF directly. */
static volatile uint8_t tx_busy = 0;


/*
   CRC-8 HELPER (poly 0x07, init 0x00, no reflection)
   ---
   Must match the bridge's crc8().  Keil C51 generates ~8 cycles per
   inner iteration, so 32 iterations per frame = ~23 us at 11.0592 MHz
   core -- well under the 87 us UART byte time.
*/
/* NOTE: parameter is named `buf` rather than `data` because `data` is a
   Keil C51 reserved memory-space keyword (alongside xdata/idata/code/
   pdata/bdata) -- using it as an identifier yields C141 syntax errors. */
static uint8_t uart_crc8(uint8_t *buf, uint8_t len)
{
    uint8_t crc = 0u;
    uint8_t i, j;

    for (i = 0u; i < len; i++)
    {
        crc ^= buf[i];
        for (j = 0u; j < 8u; j++)
        {
            if (crc & 0x80u)
            {
                crc = (uint8_t)((crc << 1) ^ UART_CRC_POLY);
            }
            else
            {
                crc = (uint8_t)(crc << 1);
            }
        }
    }
    return crc;
}


/*
   UART RX BYTE HANDLER -- called from the ISR
   ---
   A single CRC mismatch aborts the current frame and returns the state
   machine to SOF hunt.  A stray 0xAA in the middle of a corrupted
   stream will therefore resync at most one frame later.
*/
static void uart_handle_rx_byte(uint8_t rx_byte)
{
    switch (uart_rx_state)
    {
        case UART_RX_WAIT_SOF:
            if (rx_byte == UART_SOF)
            {
                uart_rx_count = 0u;
                uart_rx_state = UART_RX_COLLECT;
            }
            break;

        case UART_RX_COLLECT:
            uart_rx_buf[uart_rx_count++] = rx_byte;
            if (uart_rx_count >= UART_PAYLOAD_LEN)
            {
                uart_rx_state = UART_RX_WAIT_CRC;
            }
            break;

        case UART_RX_WAIT_CRC:
        default:
            if (rx_byte == uart_crc8(uart_rx_buf, UART_PAYLOAD_LEN))
            {
                /* Frame is valid -- assemble the 26-bit word.  If the
                   main loop has not consumed the previous frame yet we
                   overwrite it; the newer data is always preferred. */
                rx_frame_data   = (uint16_t)uart_rx_buf[0]
                                | (((uint16_t)(uart_rx_buf[1] & 0x7Fu)) << 8);
                rx_frame_red    = (uint16_t)uart_rx_buf[2]
                                | (((uint16_t)(uart_rx_buf[3] & 0x03u)) << 8);
                rx_frame_parity = (uart_rx_buf[1] >> 7) & 0x01u;
                rx_frame_ready  = 1u;
            }
            else
            {
                rx_frame_crc_errors++;
#if RX_DEBUG
                /* 'C' = bridge-link CRC mismatch.  Nested call into
                   rx_send_uart_byte() from the UART ISR is safe: the
                   ES=0/ES=1 guards are idempotent while already inside
                   the vector-4 handler, and SBUF's RX and TX halves are
                   independent hardware. */
                rx_send_uart_byte('C');
#endif
            }
            uart_rx_state = UART_RX_WAIT_SOF;
            break;
    }
}


/*
   UART ISR (vector 4, address 0x23)
   ---
   Serves both RI (receive) and TI (transmit) on the single 8052 UART.
   Priority-wise RI is handled first so that a simultaneous RI+TI (rare
   at 115200, but possible when a new start bit arrives as the previous
   byte finishes shifting out) is drained promptly.

   TI handling: if there is buffered data, shove the next byte into
   SBUF and keep tx_busy = 1.  Otherwise clear tx_busy and leave TI = 0
   so the ISR does not re-fire until the next send primes SBUF again.
*/
void UART_ISR(void) interrupt 4
{
    uint8_t rx_byte;

    if (RI)
    {
        rx_byte = SBUF;
        RI = 0;
        uart_handle_rx_byte(rx_byte);
    }

    if (TI)
    {
        TI = 0;
        if (tx_tail != tx_head)
        {
            SBUF = tx_buf[tx_tail];
            tx_tail = (tx_tail + 1u) % TX_BUF_SIZE;
        }
        else
        {
            tx_busy = 0;
        }
    }
}


/*
   PERIPHERAL INITIALISATION
*/

/*
   RX_Timer3_Init -- Timer 3 as UART baud rate generator.

   Per the ADuC841 datasheet (Table 35, p.76): with the PLL at fCORE =
   11.0592 MHz (CD=0, PLL=1), T3CON = 0x82 and T3FD = 0x20 give an
   exact 115200 baud.  Verified by the datasheet formula
     actual_baud = (2 * fCORE) / (2^(DIV-1) * (T3FD + 64))
                 = (22118400) / (2 * 96) = 115200.
*/
void RX_Timer3_Init(void)
{
    T3FD  = 0x20u;   /* Fractional divider = 32 */
    T3CON = 0x82u;   /* T3BAUDEN=1, DIV=2 (binary 010) */
}


/*
   RX_UART_Init -- SCON Mode 1, full-duplex, interrupt-driven.

   SCON = 0x50:
     SM0=0, SM1=1   -> Mode 1 (8-bit async UART, variable baud)
     SM2=0          -> No multiprocessor framing
     REN=1          -> Receive enabled (NEW: was 0 in SPI version)
     TB8=RB8=TI=RI = 0

   ES=1 enables the serial interrupt; the ISR handles both RI and TI.
   TI starts at 0: the first rx_send_uart_byte() will prime SBUF.

   The MOV-as-byte style (rather than individual sbit writes to SM0/SM1)
   avoids the SMOD0 trap documented on the TX side -- when SMOD0=1 in
   PCON, SM0 aliases to a framing-error flag and sbit writes silently
   miss SCON.7.
*/
void RX_UART_Init(void)
{
    SCON    = 0x50u;
    tx_busy = 0;
    tx_head = 0;
    tx_tail = 0;
    uart_rx_state       = UART_RX_WAIT_SOF;
    uart_rx_count       = 0;
    rx_frame_ready      = 0;
    rx_frame_data       = 0;
    rx_frame_red        = 0;
    rx_frame_parity     = 0;
    rx_frame_crc_errors = 0;

    ES = 1;
    /* EA is set by the caller after all peripherals are initialised. */
}


/*
   NON-BLOCKING UART TX ENQUEUE
   ---
   If the transmitter is idle, write SBUF directly and mark tx_busy.
   If busy, push into the ring buffer; dropped on overflow.

   ES is toggled around the shared state update to prevent the ISR
   from racing on tx_busy / tx_head.
*/
void rx_send_uart_byte(uint8_t data_byte)
{
    uint8_t next_head;

    ES = 0;

    if (!tx_busy)
    {
        tx_busy = 1;
        SBUF    = data_byte;
        ES      = 1;
        return;
    }

    next_head = (tx_head + 1u) % TX_BUF_SIZE;
    if (next_head != tx_tail)
    {
        tx_buf[tx_head] = data_byte;
        tx_head         = next_head;
    }
    /* else: ring full, byte dropped */

    ES = 1;
}
