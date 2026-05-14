import serial

# State descriptions mapped from the C code patterns array
state_descriptions = {
    0: "P2.0=0  P2.1=1  P2.2=0 (Hex: 0x02)",
    1: "P2.0=1  P2.1=0  P2.2=1 (Hex: 0x05)",
    2: "P2.0=1  P2.1=1  P2.2=1 (Hex: 0x07)",
    3: "P2.0=0  P2.1=0  P2.2=0 (Hex: 0x00)"
}


def main():
    # Configure the port and baud rate (match the 9600 from C code)
    port_name = 'COM4'  # Change to your actual COM port

    try:
        ser = serial.Serial(port_name, 9600)
        print(f"Opened {port_name} at 9600 baud.")
    except serial.SerialException as e:
        print(f"Error opening port: {e}")
        return

    # Updated instructions for the user
    print("Enter a state number (0, 1, 2, or 3) and press Enter, or 'q' to quit.")
    print("State 3 = All OFF | State 2 = All ON")

    while True:
        user_input = input(">> ").strip().lower()

        if user_input == 'q':
            break

        # Check if the user entered a valid state number
        elif user_input in ['0', '1', '2', '3']:
            current_state = int(user_input)

            # Convert state to character and encode to bytes (e.g., '3' -> b'3')
            char_to_send = str(current_state).encode()
            ser.write(char_to_send)

            # Print the current hardware state
            print(f"--> Entered State '{current_state}': {state_descriptions[current_state]}")

        else:
            print("Invalid input. Type a number between 0 and 3, or 'q' to quit.")

    # Close the port when done
    ser.close()
    print("Port closed.")


if __name__ == "__main__":
    main()