#!/usr/bin/env python3
"""
File: host_sender.py
=============================================================================
H1-Type Bus Encoder Host Script
Sends raw characters to ADuC841 via serial port.
=============================================================================

INPUT FORMAT:
-------------
This script sends raw 8-bit characters directly to the MCU.
The MCU firmware splits each character into:
  - High nibble (bits 7..4) -> transmitted first
  - Low nibble (bits 3..0)  -> transmitted second

Each nibble becomes a 4-bit syndrome S_new for the H1 encoder.

USAGE:
------
    python host_sender.py

Enter text when prompted. Press Enter to send.
The script adds '\\n' as a batch terminator after your input.

CONFIGURATION:
--------------
Edit the PORT and BAUDRATE variables below to match your setup.
"""

import serial
import time

# =============================================================================
# CONFIGURATION (User-Editable)
# =============================================================================
PORT = 'COM5'       # Serial port (Windows: 'COM5', Linux: '/dev/ttyUSB0')
BAUDRATE = 9600     # Must match MCU configuration

# =============================================================================
# FUNCTIONS
# =============================================================================

def send_string_to_mcu(text, port=PORT, baudrate=BAUDRATE):
    """
    Send a string of characters to the MCU.
    
    Each character is sent as a raw byte. The MCU firmware will:
    1. Split each byte into high nibble and low nibble
    2. Process high nibble first, then low nibble
    3. Each nibble triggers a full H1 encode cycle with shift register output
    
    Args:
        text: String to send (will be encoded to UTF-8 bytes)
        port: Serial port name
        baudrate: Baud rate (should be 9600)
    """
    try:
        print(f"Connecting to {port} at {baudrate} baud...")
        ser = serial.Serial(port, baudrate, timeout=1)
        
        # Wait for connection stabilization
        time.sleep(2)
        
        # Send each character
        print(f"Sending: '{text}'")
        for i, char in enumerate(text):
            byte_val = ord(char)
            high_nibble = (byte_val >> 4) & 0x0F
            low_nibble = byte_val & 0x0F
            
            print(f"  Char '{char}' (0x{byte_val:02X}): "
                  f"High=0x{high_nibble:X}, Low=0x{low_nibble:X}")
            
            # Send the raw byte
            ser.write(bytes([byte_val]))
            
            # Small delay between characters for MCU processing
            time.sleep(0.01)
        
        # Send terminator
        ser.write(b'\\n')
        print("Sent terminator (newline)")
        
        ser.close()
        print("Done.")
        
    except serial.SerialException as e:
        print(f"Serial port error: {e}")

def interactive_mode():
    """
    Interactive mode: repeatedly prompt user for input and send to MCU.
    """
    print("="*60)
    print("H1-Type Bus Encoder Host Script")
    print("="*60)
    print(f"Port: {PORT}, Baudrate: {BAUDRATE}")
    print("Enter text to send. Press Ctrl+C to exit.\\n")
    
    try:
        while True:
            user_input = input("Enter text: ")
            if user_input:
                send_string_to_mcu(user_input)
                print()
    except KeyboardInterrupt:
        print("\\nExiting.")

def demonstrate_nibble_split():
    """
    Demonstration of nibble splitting for common test characters.
    """
    print("\\n" + "="*60)
    print("Nibble Split Demonstration")
    print("="*60)
    
    test_chars = "ABCD01239!\\n"
    
    print(f"{'Char':<6} {'ASCII':<8} {'Binary':<12} {'High':<6} {'Low':<6}")
    print("-"*40)
    
    for char in test_chars:
        byte_val = ord(char)
        high = (byte_val >> 4) & 0x0F
        low = byte_val & 0x0F
        
        display_char = repr(char) if char in '\\n\\r\\t' else f"'{char}'"
        print(f"{display_char:<6} 0x{byte_val:02X}    {byte_val:08b}    "
              f"0x{high:X}    0x{low:X}")
    
    print()

# =============================================================================
# MAIN
# =============================================================================

if __name__ == "__main__":
    # Show nibble split demonstration
    demonstrate_nibble_split()
    
    # Enter interactive mode
    interactive_mode()
