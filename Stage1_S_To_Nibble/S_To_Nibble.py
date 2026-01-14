import serial
import time


def convert_bits_to_hex_symbols(bits):
    hex_symbols = []
    for i in range(0, len(bits), 4):
        chunk = bits[i: i + 4]
        while len(chunk) < 4:
            chunk.append(0)
        val = 0
        for bit in chunk:
            val = (val << 1) | bit
        hex_symbols.append(f'{val:X}')
    return hex_symbols


def send_hex_to_mcu(hex_list, port='COM5', baudrate=9600):
    try:
        print(f"Connecting to {port}...")
        ser = serial.Serial(port, baudrate, timeout=1)
        time.sleep(2)
        message_string = "".join(hex_list)
        print(f"Sending string: {message_string}")
        ser.write(message_string.encode('utf-8'))
        print("Done sending.")
        ser.close()
    except serial.SerialException as e:
        print(f"Error opening serial port: {e}")


def get_user_input_as_syndromes():
    user_text = input("Please enter text (press Enter to send): ")
    user_text += '\n'
    all_syndromes = []

    print(f"\nProcessing input: '{user_text.strip()}' + Enter")

    for char in user_text:
        ascii_val = ord(char)
        # 2. המרה לבינארי
        bin_str = f'{ascii_val:08b}'
        # 3. המרה לרשימה של מספרים
        bits = [int(b) for b in bin_str]
        all_syndromes.append(bits)

    return all_syndromes


# --- Main ---
if __name__ == "__main__":
    # 1. קבלת הקלט מהמשתמש (רשימה של רשימות ביטים)
    syndromes_list = get_user_input_as_syndromes()
    # 2. מעבר על כל "סינדרום" (תו) ושליחתו
    for p, raw_syndrome in enumerate(syndromes_list):
        print(f"--- Processing Character {p + 1} ---")
        print(f"Bits: {raw_syndrome}")
        # המרה להקס
        hex_result = convert_bits_to_hex_symbols(raw_syndrome)
        print(f"Converted to Hex: {hex_result}")
        # שליחה למיקרו-בקר
        #  שולח תו אחרי תו
        send_hex_to_mcu(hex_result, port='COM5', baudrate=9600)
        # השהייה קטנה בין תווים
        time.sleep(0.1)