#!/usr/bin/env python3
import serial
import time

PORT = 'COM5'
BAUDRATE = 9600

# Global serial connection
_ser = None


def get_serial():
    global _ser
    if _ser is None or not _ser.is_open:
        print(f"Opening {PORT}...")
        _ser = serial.Serial(PORT, BAUDRATE, timeout=1)
        time.sleep(2)  # Wait for port to stabilize
        _ser.reset_input_buffer()
        _ser.reset_output_buffer()
        print("Port ready.")
    return _ser


def send_string_to_mcu(text):
    ser = get_serial()
    print(f"Sending: '{text}'")
    for char in text:
        byte_val = ord(char)
        print(f"  '{char}' (0x{byte_val:02X})")
        ser.write(bytes([byte_val]))
        time.sleep(0.01)
    # Don't close - keep port open


def interactive_mode():
    print("=" * 60)
    print("H1-Type Bus Encoder Host Script")
    print("=" * 60)
    get_serial()  # Open port once at startup
    print("Enter text to send. Press Ctrl+C to exit.\n")

    try:
        while True:
            user_input = input("Enter text: ")
            if user_input:
                send_string_to_mcu(user_input)
                print()
    except KeyboardInterrupt:
        if _ser:
            _ser.close()
        print("\nExiting.")

if __name__ == "__main__":
    interactive_mode()