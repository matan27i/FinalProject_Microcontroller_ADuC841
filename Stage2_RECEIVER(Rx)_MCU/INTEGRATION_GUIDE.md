/**
 * INTEGRATION_GUIDE.md
 * 
 * Quick Start Guide for Integrating RX Decoder into ADuC841 System
 */

# Integration Guide for Stage 2 RX Decoder

## Quick Start (5 Minutes)

### Step 1: Add Files to Your Project
```
Your_Project/
├── main.c              (your existing main)
├── rx_decoder.c        (add this)
├── rx_decoder.h        (add this)
└── Makefile or .uvproj
```

### Step 2: Initialize in Your main()
```c
#include "rx_decoder.h"

void main(void) {
    // ... existing initialization ...
    
    // Initialize UART
    SCON = 0x50;        // Mode 1, REN=1
    TMOD |= 0x20;       // Timer 1 Mode 2
    TH1 = 0xFD;         // 9600 baud @ 11.0592MHz
    TR1 = 1;            // Start timer
    TI = 0;             // Clear flags
    
    // Initialize decoder
    rx_decoder_init();
    
    // ... rest of your code ...
}
```

### Step 3: Call rx_process_state() When Data Arrives
```c
// Option A: Polling
while (1) {
    if (new_data_flag) {
        uint16_t bus_state = get_corrected_bus_state();
        rx_process_state(bus_state);
        new_data_flag = 0;
    }
}

// Option B: Interrupt-driven
void ext0_isr(void) interrupt 0 {
    rx_process_state(P1);  // Example: read from port
}
```

That's it! The decoder will automatically:
- Extract data nibbles
- Reassemble bytes
- Send via UART

---

## Common Integration Patterns

### Pattern 1: Direct Port Reading
Best for: Hardware that presents bus state on GPIO port

```c
#include <reg52.h>
#include "rx_decoder.h"

// P1 = bus state bits 0-7
// P2.0-P2.6 = bus state bits 8-14

uint16_t read_bus_state(void) {
    uint16_t state = P1;                    // Lower 8 bits
    state |= ((uint16_t)(P2 & 0x7F) << 8);  // Upper 7 bits
    return state;
}

void main(void) {
    UART_Init();
    rx_decoder_init();
    
    while (1) {
        if (data_ready_pin) {
            uint16_t bus = read_bus_state();
            rx_process_state(bus);
        }
    }
}
```

### Pattern 2: Shift Register Interface
Best for: Serial-to-parallel conversion (e.g., 74HC595)

```c
#include "rx_decoder.h"

sbit LATCH = P2^0;
sbit CLOCK = P2^1;
sbit DATA  = P2^2;

uint16_t shift_in_15bits(void) {
    uint16_t value = 0;
    uint8_t i;
    
    LATCH = 0;  // Load parallel data
    LATCH = 1;
    
    for (i = 0; i < 15; i++) {
        value >>= 1;
        if (DATA) value |= 0x4000;
        CLOCK = 1;
        CLOCK = 0;
    }
    
    return value;
}

void main(void) {
    UART_Init();
    rx_decoder_init();
    
    while (1) {
        uint16_t bus = shift_in_15bits();
        rx_process_state(bus);
        delay_ms(10);  // Adjust to match TX rate
    }
}
```

### Pattern 3: Interrupt-Driven with FIFO Buffer
Best for: High-speed or bursty data

```c
#include "rx_decoder.h"

#define FIFO_SIZE 16
uint16_t fifo_buffer[FIFO_SIZE];
uint8_t fifo_head = 0;
uint8_t fifo_tail = 0;

// ISR pushes to FIFO
void ext0_isr(void) interrupt 0 {
    uint16_t bus_state = read_hardware_register();
    fifo_buffer[fifo_head] = bus_state;
    fifo_head = (fifo_head + 1) % FIFO_SIZE;
}

// Main loop processes FIFO
void main(void) {
    UART_Init();
    rx_decoder_init();
    
    EX0 = 1;  // Enable INT0
    EA = 1;   // Global interrupt enable
    
    while (1) {
        if (fifo_tail != fifo_head) {
            rx_process_state(fifo_buffer[fifo_tail]);
            fifo_tail = (fifo_tail + 1) % FIFO_SIZE;
        }
    }
}
```

### Pattern 4: SPI Slave Receiver
Best for: SPI-based error corrector module

```c
#include <reg52.h>
#include "rx_decoder.h"

sbit SS   = P1^4;
sbit MOSI = P1^5;
sbit MISO = P1^6;
sbit SCK  = P1^7;

uint16_t spi_receive_15bits(void) {
    uint16_t data = 0;
    uint8_t i;
    
    while (SS);  // Wait for chip select
    
    for (i = 0; i < 15; i++) {
        while (!SCK);  // Wait for clock high
        data = (data << 1) | MOSI;
        while (SCK);   // Wait for clock low
    }
    
    return data;
}

void main(void) {
    UART_Init();
    rx_decoder_init();
    
    while (1) {
        uint16_t bus = spi_receive_15bits();
        rx_process_state(bus);
    }
}
```

---

## Timing Considerations

### TX Rate Matching
The decoder must process bus states at the TX transmission rate:

```
TX sends: [High Nibble] --delay--> [Low Nibble] --delay--> [Next High]
          └─────────────────────────────┘
          One complete ASCII character
```

**Example:** If TX sends nibbles every 10ms:
```c
void main(void) {
    Timer0_Init_10ms();  // Configure Timer 0
    
    while (1) {
        if (timer0_flag) {
            timer0_flag = 0;
            uint16_t bus = read_bus_state();
            rx_process_state(bus);
        }
    }
}
```

### UART Bandwidth
Ensure UART baud rate can handle the incoming data rate:

```
Data rate = (TX nibble rate / 2) bytes/sec
Required baud = Data rate × 10 bits/byte

Example:
- TX sends 100 nibbles/sec
- Data rate = 50 bytes/sec  
- Required baud ≥ 500 baud
- Use 9600 baud (safe margin)
```

---

## Debugging Techniques

### 1. Monitor Syndrome Values
```c
void rx_process_state(uint16_t current_bus_state) {
    uint8_t nibble = compute_syndrome(current_bus_state);
    
    // Debug: Output to spare UART or LED display
    debug_output(nibble);  
    
    // Normal processing continues...
}
```

### 2. State Machine Tracking
```c
// Add to rx_decoder.c
uint8_t get_rx_state(void) {
    return (uint8_t)current_state;
}

// In main.c
if (get_rx_state() != expected_state) {
    error_handler();
}
```

### 3. Loopback Testing
```c
// Connect TX directly to RX for testing
void loopback_test(void) {
    uint8_t test_char = 'A';
    
    // Manually encode and send
    uint8_t high = (test_char >> 4) & 0x0F;
    uint8_t low = test_char & 0x0F;
    
    // Simulate TX encoding (you need TX encode function)
    uint16_t bus1 = encode_nibble(high);
    uint16_t bus2 = encode_nibble(low);
    
    // Decode
    rx_process_state(bus1);
    rx_process_state(bus2);
    
    // Check UART output = 'A'
}
```

### 4. Oscilloscope Checkpoints
Monitor these signals:
- **Bus State Input:** Verify 15-bit pattern matches TX
- **UART TXD:** Check proper framing (start bit, 8 data, stop bit)
- **Data Ready:** Ensure proper timing/synchronization

---

## Common Issues & Solutions

### Issue 1: Wrong Characters Received
**Symptoms:** Garbled output, incorrect bytes
**Causes:**
- Bit order mismatch (LSB vs MSB first)
- Endianness confusion
- Missing bits in bus state

**Solutions:**
```c
// Verify bit ordering
uint16_t bus = 0x0001;  // Should give nibble = 1
uint8_t result = compute_syndrome(bus);
// If result != 1, check bit indexing
```

### Issue 2: Every Other Character Wrong
**Symptoms:** Pattern like "A?B?C?"
**Cause:** State machine out of sync

**Solution:**
```c
// Add sync reset mechanism
void rx_reset_sync(void) {
    rx_decoder_init();  // Reset to high nibble state
}

// Call on timeout or special sync signal
```

### Issue 3: UART Not Transmitting
**Symptoms:** No output on terminal
**Checks:**
```c
// 1. Verify UART init
SCON = 0x50;  // Check mode and REN bit

// 2. Check baud rate calculation
// At 11.0592MHz for 9600 baud:
TH1 = 256 - (11059200 / (384 * 9600)) = 0xFD

// 3. Verify Timer 1 running
if (!TR1) TR1 = 1;

// 4. Test with known character
UART_Tx('U');  // Should see 0x55 = 01010101 (good for scope)
```

### Issue 4: Lost Synchronization
**Symptoms:** Works initially, then fails
**Cause:** Missing nibble (noise, timing)

**Solution:**
```c
// Add timeout detection
static uint8_t last_nibble_time = 0;

void timer0_isr(void) interrupt 1 {
    last_nibble_time++;
    if (last_nibble_time > TIMEOUT_THRESHOLD) {
        rx_decoder_init();  // Reset on timeout
        last_nibble_time = 0;
    }
}

void rx_process_state(uint16_t bus) {
    last_nibble_time = 0;  // Reset timer on valid data
    // ... normal processing ...
}
```

---

## Performance Optimization

### 1. Fast Syndrome Computation (Lookup Table)
For maximum speed, use a 32KB lookup table:

```c
// Pre-compute all syndromes (do this once at compile time)
const uint8_t syndrome_lut[32768] = { /* ... */ };

uint8_t compute_syndrome_fast(uint16_t bus_state) {
    return syndrome_lut[bus_state & 0x7FFF];
}
```

### 2. Inline Functions
```c
// In rx_decoder.h
#pragma inline
static uint8_t compute_syndrome(uint16_t bus_state) {
    // Implementation here
}
```

### 3. Direct Register Access
```c
// Replace function call with macro for ISR
#define PROCESS_BUS(state) do { \
    uint8_t nib = compute_syndrome(state); \
    if (rx_state) { \
        SBUF = (high_nib << 4) | nib; \
        rx_state = 0; \
    } else { \
        high_nib = nib; \
        rx_state = 1; \
    } \
} while(0)
```

---

## Testing Checklist

- [ ] Syndrome computation verified (use test_decoder.c)
- [ ] UART baud rate matches PC terminal
- [ ] UART initialization before decoder init
- [ ] Correct bit ordering (LSB/MSB)
- [ ] Bus state timing matches TX rate
- [ ] All 15 bits captured correctly
- [ ] State machine resets properly
- [ ] Loopback test passes
- [ ] Full ASCII range (0x00-0xFF) tested
- [ ] Error handling for sync loss

---

## Further Reading

- **ADuC841 Datasheet:** Sections on UART, GPIO, Timers
- **8051 Assembly:** For ultra-fast syndrome computation
- **Hamming Codes:** Understanding H1 matrix properties
- **Test File:** test_decoder.c for validation

---

## Support

If you encounter issues:
1. Run test_decoder.c to verify syndrome logic
2. Check UART with oscilloscope (0x55 pattern)
3. Verify bus state capture (all 15 bits)
4. Add debug outputs to track state machine
5. Use loopback test to isolate RX vs TX issues
