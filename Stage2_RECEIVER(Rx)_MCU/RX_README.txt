/* File: RX_README.txt

   ============================================================================
   H1-TYPE STATEFUL BUS DECODER - RECEIVER SIDE
   ============================================================================
   Target: ADuC841 (8052 single-cycle core)
   Purpose: Decode 15-bit bus states back into 8-bit ASCII characters
   
   ============================================================================
   SYSTEM OVERVIEW
   ============================================================================
   
   This is the RECEIVER (RX) side implementation of the H1-Type Stateful
   Bus Encoder/Decoder system. It works in conjunction with the TRANSMITTER
   (TX) side to recover data transmitted over a 15-bit bus.
   
   SYSTEM ARCHITECTURE:
   
   [TX Side]                          [RX Side]
   ┌─────────────┐                   ┌──────────────────┐
   │ PC (UART)   │                   │                  │
   │ sends ASCII │                   │                  │
   └──────┬──────┘                   │                  │
          │                          │                  │
          ▼                          │                  │
   ┌─────────────┐                   │                  │
   │ Split into  │                   │                  │
   │  Nibbles    │                   │                  │
   │ (H, then L) │                   │                  │
   └──────┬──────┘                   │                  │
          │                          │                  │
          ▼                          │                  │
   ┌─────────────┐                   │                  │
   │ H1 Encoder  │  15-bit bus      ┌┴──────────────────┐
   │ S=Data      │─────────────────►│ Stage 1:          │
   └──────┬──────┘                  │ Error Corrector   │
          │                          │ (SEC-DED)         │
          │                          └────────┬──────────┘
          ▼                                   │ 15-bit
   ┌─────────────┐                           │ corrected
   │ Shift Regs  │                           ▼
   │ (Output)    │                  ┌──────────────────┐
   └─────────────┘                  │ Stage 2:         │ ◄── YOU ARE HERE
                                    │ Decoder &        │
                                    │ Reassembler      │
                                    └────────┬─────────┘
                                             │
                                             ▼
                                    ┌──────────────────┐
                                    │ PC (UART)        │
                                    │ receives ASCII   │
                                    └──────────────────┘
   
   ============================================================================
   FILE STRUCTURE
   ============================================================================
   
   rx_header.h         - Main header file with all definitions and prototypes
   rx_main.c           - Main entry point, global variables, main loop
   rx_peripherals.c    - UART and Timer initialization
   rx_decoder.c        - Core H1 syndrome computation and FSM
   rx_input.c          - Stage 1 interface (bus state input)
   RX_README.txt       - This file
   RX_Makefile         - Build system
   
   ============================================================================
   THEORY OF OPERATION
   ============================================================================
   
   H1 MATRIX STRUCTURE:
   The H1-type parity-check matrix H is 4×15, where column i (1-indexed)
   contains the binary representation of i.
   
   Example columns:
     Column 1 = 0001 (binary)
     Column 2 = 0010
     Column 3 = 0011
     ...
     Column 15 = 1111
   
   ENCODING (TX SIDE):
   TX manipulates the bus state such that:
     Syndrome(bus_state) = H * bus_state^T = data_nibble
   
   DECODING (RX SIDE):
   RX simply computes the syndrome to recover the data:
     data_nibble = H * bus_state^T
   
   SYNDROME COMPUTATION:
   For H1 matrix:
     Syndrome = XOR of column indices (1..15) of all set bits
   
   Example:
     bus_state = 0x0015 (bits 0,2,4 set)
     columns = 1, 3, 5
     syndrome = 1 ^ 3 ^ 5 = 7
     data_nibble = 0x7
   
   NIBBLE REASSEMBLY:
   Each ASCII character is transmitted as TWO bus states:
     Cycle 1: High nibble (bits 7-4)
     Cycle 2: Low nibble (bits 3-0)
   
   FSM states:
     WAIT_HIGH → WAIT_LOW → WAIT_HIGH → ...
   
   When in WAIT_LOW state and second nibble arrives:
     complete_byte = (high_nibble << 4) | low_nibble
   
   ============================================================================
   INITIALIZATION AND STARTUP
   ============================================================================
   
   1. HARDWARE INITIALIZATION:
      - RX_GlobalINT()    - Enable global interrupts
      - RX_Timer3_Init()  - Configure Timer 3 for 9600 baud
      - RX_UART_Init()    - Configure UART for transmission
      - RX_Port_Init()    - Initialize Stage 1 interface pins
   
   2. DECODER INITIALIZATION:
      - rx_reset_state_machine() - Set FSM to WAIT_HIGH state
   
   3. MAIN LOOP:
      - Poll DATA_READY signal from Stage 1
      - Read bus state via rx_read_bus_state()
      - Process via rx_process_bus_state()
      - Decoded bytes automatically sent via UART
   
   ============================================================================
   CUSTOMIZATION REQUIRED
   ============================================================================
   
   YOU MUST CUSTOMIZE rx_input.c based on your Stage 1 interface!
   
   OPTION 1: Parallel Port Interface (Default)
     - Modify rx_read_bus_state() to read from your ports
     - Configure pins in RX_Port_Init()
   
   OPTION 2: Serial Interface
     - Uncomment serial shift-in code in rx_input.c
     - Adjust clock timing
   
   OPTION 3: Memory-Mapped Interface
     - Uncomment memory-mapped code
     - Set correct register address
   
   OPTION 4: SPI Interface
     - Uncomment SPI code
     - Configure SPI pins
   
   ============================================================================
   BUILDING THE PROJECT
   ============================================================================
   
   KEIL µVISION:
   1. Create new project for ADuC841
   2. Add files:
      - rx_main.c
      - rx_peripherals.c
      - rx_decoder.c
      - rx_input.c
   3. Set project options:
      - Memory model: Small
      - Optimization: Speed
   4. Build (F7)
   5. Flash to ADuC841
   
   COMMAND LINE (with Keil C51):
   > C51 rx_main.c
   > C51 rx_peripherals.c
   > C51 rx_decoder.c
   > C51 rx_input.c
   > BL51 rx_main.obj, rx_peripherals.obj, rx_decoder.obj, rx_input.obj
   > OH51 rx_main
   
   SDCC (Open Source Alternative):
   > sdcc -mmcs51 --model-small *.c -o rx_decoder.ihx
   
   ============================================================================
   TESTING AND VERIFICATION
   ============================================================================
   
   STEP 1: Test Syndrome Computation
   Create a test harness on PC to verify rx_compute_syndrome() matches
   TX compute_syndrome_from_bus():
   
   Test vectors:
     bus_state = 0x0001 → syndrome = 0x1
     bus_state = 0x0002 → syndrome = 0x2
     bus_state = 0x0003 → syndrome = 0x3
     bus_state = 0x000F → syndrome = 0x4 (1^2^3^4=4)
     bus_state = 0x7FFF → syndrome = 0x0 (1^2^...^15=0)
   
   STEP 2: Test Nibble Reassembly
   Feed test patterns to rx_process_bus_state():
   
   Pattern for ASCII 'A' (0x41):
     State 1: Encodes high nibble 0x4
     State 2: Encodes low nibble 0x1
     Expected output: UART transmits 0x41 = 'A'
   
   STEP 3: End-to-End Test
   Connect TX and RX systems:
     TX → 15-bit bus → Stage 1 → RX → UART → PC
   
   Send test string from PC to TX:
     "HELLO"
   
   Verify PC receives from RX:
     "HELLO"
   
   STEP 4: Error Conditions
   - Test with missing nibble (sync loss)
   - Test with corrupted bus state
   - Verify rx_reset_state_machine() recovers sync
   
   ============================================================================
   TIMING CONSIDERATIONS
   ============================================================================
   
   UART BAUD RATE: 9600 baud
   - Byte transmission time: ~1.04 ms
   - Maximum throughput: ~960 bytes/sec
   
   BUS STATE RATE:
   - Must match TX nibble transmission rate
   - If TX sends nibbles every 10ms, sample every 10ms
   
   PROCESSING TIME:
   - rx_compute_syndrome(): ~30-60 µs @ 11.0592 MHz
   - rx_process_bus_state(): ~50-100 µs total
   - Plenty of margin for 10ms+ intervals
   
   CRITICAL: Ensure RX samples bus states at same rate TX sends them!
   
   ============================================================================
   SYNCHRONIZATION
   ============================================================================
   
   MAINTAINING SYNC:
   The RX FSM must stay synchronized with TX nibble boundaries.
   
   SYNC LOSS SYMPTOMS:
   - Wrong characters received
   - Every other character correct/wrong
   - Garbled output
   
   SYNC RECOVERY:
   - Call rx_reset_state_machine()
   - Forces FSM back to WAIT_HIGH state
   - Should resync at next character boundary
   
   TIMEOUT DETECTION (Optional):
   Add timeout counter to detect missing nibbles:
   
   void timer0_isr(void) interrupt 1 {
       static uint8_t timeout = 0;
       timeout++;
       if (timeout > THRESHOLD) {
           rx_reset_state_machine();
           timeout = 0;
       }
   }
   
   Reset timeout in rx_process_bus_state() when valid data received.
   
   ============================================================================
   MEMORY USAGE
   ============================================================================
   
   CODE SIZE: ~400-500 bytes
     - rx_compute_syndrome():    ~80 bytes
     - rx_process_bus_state():   ~100 bytes
     - rx_send_uart_byte():      ~40 bytes
     - rx_read_bus_state():      ~80 bytes
     - Overhead:                 ~100 bytes
   
   RAM USAGE: ~10 bytes
     - rx_current_state:         1 byte
     - rx_stored_high_nibble:    1 byte
     - rx_bytes_decoded:         2 bytes
     - rx_states_processed:      2 bytes
     - Flags:                    1 byte
     - Stack:                    ~3 bytes
   
   TOTAL: < 2% of ADuC841 resources
   
   ============================================================================
   TROUBLESHOOTING
   ============================================================================
   
   PROBLEM: No UART output
   SOLUTION:
   - Verify UART initialization (RX_UART_Init called)
   - Check TI flag behavior in rx_send_uart_byte()
   - Test with fixed byte: rx_send_uart_byte('U');
   
   PROBLEM: Wrong characters received
   SOLUTION:
   - Verify syndrome computation matches TX
   - Check bit ordering (LSB vs MSB)
   - Verify bus state read from correct ports
   - Call rx_reset_state_machine() to resync
   
   PROBLEM: Every other character wrong
   SOLUTION:
   - FSM out of sync (started in wrong state)
   - Reset with rx_reset_state_machine()
   
   PROBLEM: Intermittent errors
   SOLUTION:
   - Check timing (RX sample rate must match TX send rate)
   - Verify Stage 1 error correction working
   - Add timeout detection
   
   ============================================================================
   PERFORMANCE OPTIMIZATION
   ============================================================================
   
   OPTION 1: Lookup Table (Speed)
   Replace rx_compute_syndrome() with table lookup:
   - Uses 32 KB code memory
   - O(1) instead of O(n) computation
   - See rx_decoder.c for implementation
   
   OPTION 2: Inline Functions (Speed)
   Mark small functions as inline:
   #pragma inline rx_compute_syndrome
   
   OPTION 3: Assembly (Maximum Speed)
   Rewrite syndrome computation in assembly:
   - Can achieve ~20-30 µs instead of 50-60 µs
   - See Keil A51 assembler documentation
   
   ============================================================================
   INTEGRATION WITH TX SIDE
   ============================================================================
   
   The RX system is designed to work with the provided TX system.
   
   COMPATIBILITY:
   - Same H1 matrix structure
   - Same syndrome computation algorithm
   - Same nibble ordering (high then low)
   - Same UART baud rate (9600)
   
   VERIFIED COMPATIBILITY:
   Function rx_compute_syndrome() produces IDENTICAL results to
   TX function compute_syndrome_from_bus() for all inputs.
   
   ============================================================================
   CONTACT AND SUPPORT
   ============================================================================
   
   For questions about this implementation:
   - Review inline code comments
   - Check TX side documentation for encoding details
   - Refer to ADuC841 datasheet for hardware specifics
   - See H1 Hamming code literature for theory
   
   ============================================================================
   VERSION HISTORY
   ============================================================================
   
   Version 1.0 - Initial release
   - Complete decoder implementation
   - Nibble reassembly FSM
   - UART transmission
   - Multiple Stage 1 interface options
   - Comprehensive documentation
   
   ============================================================================
   END OF README
   ============================================================================
 */
