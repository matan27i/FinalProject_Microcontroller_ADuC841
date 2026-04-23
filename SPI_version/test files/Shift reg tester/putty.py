import serial
import time
import sys


SERIAL_PORT = 'COM4'
BAUD_RATE = 9600

def main():
    try:

        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        print(f"--- Connected to {SERIAL_PORT} ---")
        print("Commands:")
        print(" [A] + Enter -> Shift Bit (Send 'A')")
        print(" [C] + Enter -> Clear Register (Send 'C')")
        print(" [AUTO]      -> Auto fill (Send 'A' 8 times)")
        print(" [Q] + Enter -> Quit")
        print("---------------------------------")

        while True:

            user_input = input("Cmd >> ").upper().strip()

            if user_input == 'A':
                ser.write(b'A') 
                print(" -> Sent 'A' (Shift)")
            
            elif user_input == 'C':
                ser.write(b'C')
                print(" -> Sent 'C' (Clear)")
                
            elif user_input == 'AUTO':
                print(" -> Filling register...")
                for _ in range(8):
                    ser.write(b'A')
                    time.sleep(0.2) 
                print(" -> Done.")

            elif user_input == 'Q':
                print("Exiting...")
                break
            
            else:
                print("Unknown command. Use A, C, or Q.")

        ser.close()

    except serial.SerialException as e:
        print(f"\nError: Could not open port {SERIAL_PORT}.")
        print("Check if the device is connected and the Port name is correct.")
        print(f"System Error: {e}")

if __name__ == "__main__":
    main()