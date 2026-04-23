#include <SPI.h>

/* ============================================================
 * testspiiGuCciGaNg.ino
 *
 * Arduino Mega sketch that implements BOTH:
 *   1. host_sender.py's role -- splits each input byte into a
 *      high nibble and a low nibble (high first).
 *   2. tx_system's role      -- runs the H1-type differential bus
 *      encoder, the PESEC (3,2) SEC+DED ECC, and the overall
 *      even-parity bit that together produce the 26-bit physical
 *      bus word for every nibble.
 *
 * The 26-bit word is packed into a 4-byte SPI frame and shifted
 * to an ADuC841 running rx_system at SPI mode 1, CPHA=1.
 *
 * Frame byte layout (matches rx_hw.c SPI_Receive_ISR):
 *   byte 0 : data[7:0]
 *   byte 1 : (parity << 7) | data[14:8]
 *   byte 2 : red [7:0]
 *   byte 3 : red [9:8]         (high 6 bits = 0)
 *
 * MESSAGE : "GuCciGaNg" (9 characters -> 18 nibbles -> 18 frames).
 * A bus-reset frame ({0,0,0,0}) is sent before every message so
 * the rx side clears rx_previous_bus_state, the FSM, and the
 * error counters.  The Arduino's encoder state is reset in lock
 * step so both sides start each message from an all-zero bus.
 *
 * Repeats forever with a 2-second gap between consecutive
 * transmissions of the full message.
 *
 * Wiring (Arduino Mega -> ADuC841):
 *   Mega pin 53 (SS_PIN_SPI) -> ADuC P1.5 (/SS)
 *   Mega pin 10 (SS_PIN_INT) -> ADuC P3.2 (INT0 frame-sync)
 *   Mega MOSI (51)           -> ADuC MOSI/SDATA
 *   Mega SCK  (52)           -> ADuC SCLK
 *   Mega MISO (50)           -> ADuC MISO (P3.3)
 *   GND <-> GND (common ground required)
 * ============================================================ */

/* ----- Pin assignments --------------------------------------- */
const int SS_PIN_SPI = 53;
const int SS_PIN_INT = 10;

/* ----- Timing knobs ------------------------------------------
 * SPI_CLOCK_HZ is set to 125 kHz, which is the MINIMUM hardware
 * SPI rate achievable on an Arduino Mega (ATmega2560, F_CPU =
 * 16 MHz, SPI prescaler max = 128 -> 16 MHz / 128 = 125 kHz).
 * Requesting anything lower via SPISettings() is clamped to this
 * by the Arduino SPI library -- you cannot go below 125 kHz with
 * the hardware SPI peripheral.
 *
 * All surrounding delays (inter-byte, SS setup/hold, inter-frame)
 * have been widened to give the ADuC841's SPI/INT0 ISRs maximum
 * deadline margin at this slow rate.
 * ------------------------------------------------------------- */
const uint32_t SPI_CLOCK_HZ   = 125000UL;  /* 16 MHz / 128 -- min */
const uint16_t INTER_BYTE_US  = 1500;      /* 1.5 ms gap / byte    */
const uint16_t SS_SETUP_US    = 300;       /* /SS assert -> SCLK   */
const uint16_t SS_HOLD_US     = 300;       /* last bit -> /SS de   */
const uint16_t INTER_FRAME_MS = 60;        /* main-loop consume    */
const uint16_t INTER_CHAR_MS  = 150;       /* byte-boundary gap    */
const uint32_t INTER_MSG_MS   = 2000;

/* ----- H1 / PESEC compile-time constants --------------------- */
#define HAMMING_R         4
#define HAMMING_N         15
#define BUS_STATE_MASK    0x7FFF
#define PESEC_RED_MASK    0x03FF
#define MAX_PESEC_BLOCKS  5

/* ----- The message to transmit ------------------------------- */
const char MESSAGE[] = "AB";
const uint8_t MESSAGE_LEN = sizeof(MESSAGE) - 1;

/* ============================================================
 *                    TX-SYSTEM ENCODER STATE
 * ============================================================ */

/* H1 differential bus state (15 bits). */
static uint16_t current_bus_state = 0;

/* PESEC redundancy register (10 bits). */
static uint16_t pesec_redundancy_reg = 0;

/* Overall even-parity bit (1 bit). */
static uint8_t pesec_overall_parity = 0;

/* PESEC (3,2) matrices, built at startup. */
static uint8_t PESEC_MAT_D[20];
static uint8_t PESEC_MAT_A[40];
static uint8_t pesec_num_blocks   = 0;
static uint8_t pesec_bit_offsets[MAX_PESEC_BLOCKS];
static uint8_t pesec_col_offsets[MAX_PESEC_BLOCKS];
static uint8_t pesec_chunk_masks[MAX_PESEC_BLOCKS];
static uint8_t pesec_num_d_cols   = 0;
static uint8_t pesec_num_a_cols   = 0;

/* ============================================================
 *              PESEC MATRIX INITIALISATION (port)
 * ============================================================ */
static void initPesecMatrices(const uint8_t *m_sizes, uint8_t num_blocks) {
  uint8_t total_m = 0;
  uint8_t current_col_offset = 0;
  uint8_t max_syndrome;

  pesec_num_blocks = num_blocks;
  pesec_num_d_cols = 0;
  pesec_num_a_cols = 0;

  for (uint8_t b = 0; b < num_blocks; b++) {
    pesec_bit_offsets[b] = total_m;
    pesec_col_offsets[b] = current_col_offset;
    pesec_chunk_masks[b] = ((1 << m_sizes[b]) - 1) << total_m;

    for (uint8_t i = 1; i < (1 << m_sizes[b]); i++) {
      PESEC_MAT_D[pesec_num_d_cols++] = (i << total_m);
    }

    total_m += m_sizes[b];
    current_col_offset += ((1 << m_sizes[b]) - 1);
  }

  max_syndrome = (1 << total_m) - 1;

  for (uint8_t val = 1; val <= max_syndrome; val++) {
    uint8_t is_in_d = 0;
    for (uint8_t i = 0; i < pesec_num_d_cols; i++) {
      if (PESEC_MAT_D[i] == val) { is_in_d = 1; break; }
    }
    if (!is_in_d) {
      PESEC_MAT_A[pesec_num_a_cols++] = val;
    }
  }
}

/* ============================================================
 *                H1 SYNDROME + MINIMAL-WEIGHT
 * ============================================================ */
static uint8_t computeSyndromeFromBus(uint16_t bus_state) {
  uint8_t syndrome = 0;
  uint16_t temp = bus_state & BUS_STATE_MASK;
  for (uint8_t col_idx = 1; col_idx <= HAMMING_N; col_idx++) {
    if (temp & 0x0001u) syndrome ^= col_idx;
    temp >>= 1;
  }
  return syndrome;
}

static uint16_t findMinimalW(uint8_t s_target) {
  s_target &= 0x0F;
  if (s_target == 0) return 0;
  return ((uint16_t)1 << (s_target - 1));
}

/* ============================================================
 *                  PESEC HELPERS (port)
 * ============================================================ */
static uint8_t calcPesecSynd(uint16_t reg_val, const uint8_t *mat_ptr,
                             uint8_t cols) {
  uint8_t res = 0;
  for (uint8_t idx = 0; idx < cols; idx++) {
    if ((reg_val >> idx) & 1u) res ^= mat_ptr[idx];
  }
  return res;
}

static uint16_t solvePesecDelta(uint8_t synd_gap) {
  uint16_t delta_vec = 0;
  for (uint8_t b = 0; b < pesec_num_blocks; b++) {
    uint8_t chunk = synd_gap & pesec_chunk_masks[b];
    if (chunk) {
      uint8_t val = chunk >> pesec_bit_offsets[b];
      delta_vec |= ((uint16_t)1u << (pesec_col_offsets[b] + val - 1));
    }
  }
  return delta_vec;
}

static uint8_t parity16(uint16_t v) {
  uint8_t x = (uint8_t)(v >> 8) ^ (uint8_t)v;
  x ^= x >> 4;
  x ^= x >> 2;
  x ^= x >> 1;
  return x & 1u;
}

/* ============================================================
 *                 ECC() and process_nibble()
 * ============================================================ */
static void runEcc(void) {
  uint8_t synd_data = calcPesecSynd(current_bus_state,
                                    PESEC_MAT_A, pesec_num_a_cols);
  uint8_t synd_red  = calcPesecSynd(pesec_redundancy_reg,
                                    PESEC_MAT_D, pesec_num_d_cols);
  uint8_t synd_diff = synd_data ^ synd_red;

  uint16_t delta = solvePesecDelta(synd_diff);
  pesec_redundancy_reg ^= delta;
  pesec_redundancy_reg &= PESEC_RED_MASK;

  pesec_overall_parity = parity16(current_bus_state & BUS_STATE_MASK)
                       ^ parity16(pesec_redundancy_reg & PESEC_RED_MASK);
}

static void processNibble(uint8_t s_new) {
  s_new &= 0x0F;
  uint8_t s_old   = computeSyndromeFromBus(current_bus_state);
  uint8_t s_tgt   = s_new ^ s_old;
  uint16_t w      = findMinimalW(s_tgt);
  current_bus_state ^= w;
  current_bus_state &= BUS_STATE_MASK;
  runEcc();
}

/* ============================================================
 *                     SPI FRAME TRANSPORT
 * ============================================================ */
static void sendFrameBytes(const uint8_t *frame) {
  digitalWrite(SS_PIN_SPI, LOW);
  digitalWrite(SS_PIN_INT, LOW);
  delayMicroseconds(SS_SETUP_US);

  for (uint8_t i = 0; i < 4; i++) {
    SPI.transfer(frame[i]);
    delayMicroseconds(INTER_BYTE_US);
  }

  delayMicroseconds(SS_HOLD_US);
  digitalWrite(SS_PIN_SPI, HIGH);
  digitalWrite(SS_PIN_INT, HIGH);
}

static void packAndSendCurrentState(void) {
  uint8_t frame[4];
  uint16_t data = current_bus_state   & BUS_STATE_MASK;
  uint16_t red  = pesec_redundancy_reg & PESEC_RED_MASK;
  uint8_t  par  = pesec_overall_parity & 0x01u;

  frame[0] = (uint8_t)(data & 0xFF);
  frame[1] = (uint8_t)(((par & 0x01u) << 7) | ((data >> 8) & 0x7F));
  frame[2] = (uint8_t)(red & 0xFF);
  frame[3] = (uint8_t)((red >> 8) & 0x03);

  sendFrameBytes(frame);
}

/* ============================================================
 *               ENCODER RESET + BUS RESET FRAME
 * ============================================================ */
static void resetEncoderState(void) {
  current_bus_state    = 0;
  pesec_redundancy_reg = 0;
  pesec_overall_parity = 0;
}

static void sendBusResetFrame(void) {
  const uint8_t reset_frame[4] = {0x00, 0x00, 0x00, 0x00};
  sendFrameBytes(reset_frame);
  resetEncoderState();
}

/* ============================================================
 *                TX-HANDLER EQUIVALENT (per character)
 * ============================================================ */
static void txHandlerChar(uint8_t ch) {
  uint8_t high_nibble = (ch >> 4) & 0x0F;
  uint8_t low_nibble  = ch & 0x0F;

  processNibble(high_nibble);
  packAndSendCurrentState();
  delay(INTER_FRAME_MS);

  processNibble(low_nibble);
  packAndSendCurrentState();
  delay(INTER_FRAME_MS);
}

/* ============================================================
 *                 FULL-MESSAGE TRANSMISSION
 *
 * RESYNC STRATEGY (per-character reset -- same approach as
 * testspiiAAA.ino):
 *
 * The H1 differential encoder is STATEFUL.  The rx side tracks
 * rx_previous_bus_state and decodes every frame as a delta from
 * the previous state.  If a single SPI frame is dropped, shifted,
 * or noise-corrupted, the tx and rx states diverge and every
 * subsequent character decodes to garbage nibbles until both
 * sides are re-synchronised.
 *
 * To make every character independently recoverable, we send a
 * bus-reset frame ({0,0,0,0}) BEFORE EACH CHARACTER.  rx_main.c's
 * rx_is_bus_reset() detects this and calls rx_perform_full_reset()
 * which zeros rx_previous_bus_state, the FSM, and the error
 * counters.  We mirror the zero state in the Arduino encoder, so
 * both sides start every character from a known-clean state.
 *
 * Cost: 1 extra frame per character (-> 3 frames per char).
 * Benefit: a single bad frame now corrupts AT MOST one character
 * instead of the entire message tail.
 * ============================================================ */
static void sendFullMessage(void) {
  for (uint8_t i = 0; i < MESSAGE_LEN; i++) {
    /* Hard resync before every character. */
    sendBusResetFrame();
    delay(INTER_FRAME_MS);

    txHandlerChar((uint8_t)MESSAGE[i]);
    delay(INTER_CHAR_MS);
  }
}

/* ============================================================
 *                         SETUP / LOOP
 * ============================================================ */
void setup() {
  pinMode(SS_PIN_SPI, OUTPUT);
  pinMode(SS_PIN_INT, OUTPUT);
  digitalWrite(SS_PIN_SPI, HIGH);
  digitalWrite(SS_PIN_INT, HIGH);

  SPI.begin();
  SPI.beginTransaction(SPISettings(SPI_CLOCK_HZ, MSBFIRST, SPI_MODE1));

  /* PESEC (3,2) matrix build -- MUST match rx_main.c pesec_config. */
  const uint8_t pesec_config[2] = {3, 2};
  initPesecMatrices(pesec_config, 2);

  resetEncoderState();

  Serial.begin(9600);

  /* Allow the ADuC841 to finish boot and print RX-READY before we
     start pushing SPI frames at it. */
  delay(2000);

  Serial.println(F("GuCciGaNg SPI sender ready."));
}

void loop() {
  Serial.print(F("TX -> "));
  Serial.println(MESSAGE);

  sendFullMessage();

  Serial.println(F("Sleeping 2s..."));
  delay(INTER_MSG_MS);
}
