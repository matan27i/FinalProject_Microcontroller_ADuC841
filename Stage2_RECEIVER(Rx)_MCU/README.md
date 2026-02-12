# Stage 2 Decoder & Reassembler for H1-Type Stateful Bus Encoder

## Overview

This module implements the receiver (RX) side Stage 2 decoder for an H1-type Hamming code based stateful bus communication system running on the ADuC841 microcontroller.

### System Architecture

```
[TX Side]                    [RX Side]
┌─────────────┐             ┌──────────────────┐
│   8-bit     │             │   Stage 1:       │
│   ASCII     │             │   Error          │
│   Data      │             │   Correction     │
└──────┬──────┘             │   (SEC-DED)      │
       │                    └────────┬─────────┘
       ▼                             │
┌─────────────┐                      │ 15-bit corrected
│ Split into  │                      │ bus_state
│  Nibbles    │                      ▼
│ (H, then L) │             ┌──────────────────┐
└──────┬──────┘             │   Stage 2:       │◄── YOU ARE HERE
       │                    │   Decoder &      │
       ▼                    │   Reassembler    │
┌─────────────┐             └────────┬─────────┘
│ H1 Encoder  │                      │
│ (S = Data)  │                      ▼
└──────┬──────┘             ┌──────────────────┐
       │                    │   UART Output    │
       ▼                    │   to PC          │
   15-bit bus              └──────────────────┘
```

## Theory of Operation

### H1-Type Encoding (TX Side)

The transmitter uses an H1-type Hamming matrix where:
- **Column i** = binary representation of **i** (1-indexed)
- For a 15-bit bus, columns 1-15 correspond to bit positions 0-14
- Data encoding: The TX manipulates the bus state such that `Syndrome = Data_Nibble`

**H1 Matrix Structure (4×15):**
```
        Col:  1    2    3    4    5    6    7  ...  15
             ───────────────────────────────────────────
      Row 0: 0001 0010 0011 0100 0101 0110 0111 ... 1111
          1: 
          2: 
          3: 
```

### Syndrome Computation (RX Side)

The syndrome S is computed as:
```
S = H · x^T
```

For H1 matrix, this simplifies to:
```
S = XOR of column indices (1-based) of all set bits
```

**Example:**
```
Bus State (15-bit): 0b001010010110000
Set bits at positions (0-indexed): 4, 5, 7, 10
Column indices (1-indexed):        5, 6, 8, 11

Syndrome = 5 ⊕ 6 ⊕ 8 ⊕ 11
         = 0101 ⊕ 0110 ⊕ 1000 ⊕ 1011
         = 1100
         = 0xC (12 decimal)

Therefore, Data Nibble = 0xC
```

### Nibble Reassembly

Each 8-bit ASCII character is transmitted as two bus states:
1. **Cycle 1:** High nibble (bits 7-4)
2. **Cycle 2:** Low nibble (bits 3-0)

The decoder maintains a simple 2-state FSM:
```
State: WAITING_HIGH_NIBBLE
  ↓ Receive bus state
  ↓ Extract syndrome → store as high_nibble
  ↓
State: WAITING_LOW_NIBBLE  
  ↓ Receive bus state
  ↓ Extract syndrome → store as low_nibble
  ↓ Combine: byte = (high_nibble << 4) | low_nibble
  ↓ Send via UART
  ↓
Return to: WAITING_HIGH_NIBBLE
```

## File Structure

```
rx_decoder.h       - Public API and function prototypes
rx_decoder.c       - Implementation of decoder logic
main.c             - Example integration code
README.md          - This file
```

## API Reference

### Core Functions

#### `uint8_t compute_syndrome(uint16_t bus_state)`
Computes the syndrome (data nibble) from a 15-bit bus state.

**Parameters:**
- `bus_state`: 15-bit corrected bus state (bits 0-14 valid)

**Returns:**
- 4-bit data nibble (0x0 - 0xF)

**Algorithm:**
```c
syndrome = 0;
for each bit position i (0 to 14):
    if bit i is set:
        syndrome ^= (i + 1);  // 1-based column index
return syndrome & 0x0F;
```

---

#### `void rx_process_state(uint16_t current_bus_state)`
Main decoder function. Processes incoming bus states and reassembles complete bytes.

**Parameters:**
- `current_bus_state`: 15-bit corrected bus state from Stage 1

**Behavior:**
- **First call:** Extracts high nibble, stores internally
- **Second call:** Extracts low nibble, combines with high nibble, sends via UART
- Automatically cycles between these two states

---

#### `void UART_Tx(char c)`
Transmits a byte via UART to the PC.

**Parameters:**
- `c`: Character/byte to transmit

**Prerequisites:**
- UART must be initialized before calling
- Uses standard 8051 registers: `SBUF`, `TI`

---

#### `void rx_decoder_init(void)`
Initializes the decoder state machine. Call once at startup.

## Usage Example

### Basic Integration

```c
#include "rx_decoder.h"

void main(void) {
    uint16_t corrected_bus_state;
    
    // Initialize UART (9600 baud, Mode 1)
    UART_Init();
    
    // Initialize decoder
    rx_decoder_init();
    
    while (1) {
        // Get corrected bus state from Stage 1
        if (new_data_available()) {
            corrected_bus_state = get_corrected_bus_state();
            
            // Decode and reassemble
            rx_process_state(corrected_bus_state);
        }
    }
}
```

### Complete Example with UART Initialization

```c
void UART_Init(void) {
    SCON = 0x50;    // Mode 1: 8-bit UART, REN=1
    TMOD |= 0x20;   // Timer 1 Mode 2 (auto-reload)
    TH1 = 0xFD;     // 9600 baud @ 11.0592 MHz
    TL1 = 0xFD;
    TR1 = 1;        // Start Timer 1
    TI = 0;         // Clear TX flag
}

void main(void) {
    UART_Init();
    rx_decoder_init();
    
    while (1) {
        // Process incoming bus states
        if (INT0) {  // Example: External interrupt
            rx_process_state(P1);  // Example: Read from port
        }
    }
}
```

## Building for ADuC841

### Keil µVision (C51 Compiler)

1. Create a new project for ADuC841
2. Add files: `rx_decoder.c`, `main.c`
3. Configure project options:
   - Target: ADuC841 or generic 8052
   - Crystal frequency: 11.0592 MHz (or your actual crystal)
4. Build and download to target

### Compilation Flags

```
--model-small      (Use small memory model)
--opt-code-speed   (Optimize for speed)
```

## Hardware Considerations

### ADuC841 Specifics

- **CPU Core:** 8052-compatible
- **Clock:** Typically 11.0592 MHz (for standard UART baud rates)
- **UART:** Standard 8051 UART on pins RXD (P3.0), TXD (P3.1)
- **Code Memory:** 62 KB Flash
- **RAM:** 2 KB internal SRAM

### UART Baud Rate Settings

For 11.0592 MHz crystal:

| Baud Rate | TH1 Value | Timer Mode |
|-----------|-----------|------------|
| 9600      | 0xFD      | Mode 2     |
| 19200     | 0xFE      | Mode 2     |
| 4800      | 0xFA      | Mode 2     |

Formula: `TH1 = 256 - (Crystal_Freq / (384 × Baud_Rate))`

## Testing & Validation

### Test Vector Example

Given TX sends ASCII 'A' (0x41 = 0100 0001):

**Cycle 1 (High Nibble = 0x4):**
```
TX encodes 0x4 into bus_state_1
RX receives bus_state_1
syndrome = compute_syndrome(bus_state_1) = 0x4
Stores high_nibble = 0x4
```

**Cycle 2 (Low Nibble = 0x1):**
```
TX encodes 0x1 into bus_state_2
RX receives bus_state_2
syndrome = compute_syndrome(bus_state_2) = 0x1
complete_byte = (0x4 << 4) | 0x1 = 0x41 = 'A'
UART_Tx('A')
```

### Debugging Tips

1. **Monitor State Transitions:**
   - Use `rx_get_state()` to check current FSM state
   - Expected pattern: 0 → 1 → 0 → 1...

2. **Verify Syndrome Computation:**
   - Add debug output in `compute_syndrome()`
   - Check that syndrome matches expected data nibble

3. **UART Verification:**
   - Use oscilloscope on TXD pin
   - Use serial terminal (9600 8N1) to receive data

## Memory Usage

**Code Size:** ~200-300 bytes (depending on optimization)

**RAM Usage:**
- Static variables: 2 bytes (state + high_nibble)
- Stack: Minimal (no recursion)

**Total:** < 1% of ADuC841 resources

## Performance

**Processing Time per Bus State:** ~50-100 µs @ 11.0592 MHz
- Syndrome computation: ~30-60 µs (15 iterations)
- State machine: ~10-20 µs
- UART transmission: Asynchronous (non-blocking)

**Maximum Throughput:**
- Limited by UART baud rate
- At 9600 baud: ~960 bytes/sec
- Bus state rate: ~1920 states/sec

## Error Handling

The decoder assumes Stage 1 provides **corrected** bus states. No additional error checking is performed in Stage 2. If Stage 1 detects uncorrectable errors (double-bit errors), it should handle them before passing to Stage 2.

**Robustness Features:**
- State machine has default case for safety
- Syndrome computation masks to 4 bits
- UART waits for TI flag (prevents buffer overflow)

## Future Enhancements

Potential improvements (not currently implemented):
- Timeout detection for missing nibbles
- CRC or checksum validation
- Flow control (RTS/CTS)
- Buffer for burst reception
- Error statistics logging

## References

1. **ADuC841 Datasheet:** Analog Devices MicroConverter documentation
2. **8051 Architecture:** Standard 8051/8052 reference manual  
3. **Hamming Codes:** Error-correcting code theory (H1 matrix)
4. **Keil C51:** Compiler and toolchain documentation

## License

This code is provided as-is for educational and development purposes.

## Author & Support

For questions or issues with this decoder implementation, refer to:
- ADuC841 datasheet for hardware specifics
- C51 compiler manual for language features
- Project documentation for system-level architecture
