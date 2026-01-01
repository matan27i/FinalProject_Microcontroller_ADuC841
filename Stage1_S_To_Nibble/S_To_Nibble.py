import serial
import time


# --- הפונקציה שלך (ללא שינוי) ---
def convert_bits_to_hex_symbols(bits):
    hex_symbols = []
    for i in range(0, len(bits), 4):
        chunk = bits[i: i + 4]

        # השלמת אפסים אם חסר
        while len(chunk) < 4:
            chunk.append(0)

        val = 0
        for bit in chunk:
            val = (val << 1) | bit

        hex_symbols.append(f'{val:X}')

    return hex_symbols


# --- פונקציה לשליחה ל-UART ---
def send_hex_to_mcu(hex_list, port='COM5', baudrate=9600):
    try:
        # 1. פתיחת הפורט
        print(f"Connecting to {port}...")
        ser = serial.Serial(port, baudrate, timeout=1)
        time.sleep(2)  # המתנה קצרה ליציבות החיבור (חשוב בבקרים ישנים)

        # 2. הכנת המחרוזת לשליחה
        # הפונקציה שלך מחזירה רשימה ['F', '6', 'F']
        # נחבר אותה למחרוזת אחת: "F6F"
        message_string = "".join(hex_list)

        print(f"Sending string: {message_string}")

        # 3. שליחה בפועל
        # encode הופך את המחרוזת לבתים (Bytes) כי זה מה שהפורט מבין
        ser.write(message_string.encode('utf-8'))

        # אופציונלי: הוספת תו ירידת שורה אם הבקר שלך מצפה לזה בסוף
        # ser.write(b'\n') 

        print("Done sending.")
        ser.close()

    except serial.SerialException as e:
        print(f"Error opening serial port: {e}")
        print("Hint: Make sure PuTTY is CLOSED before running this!")


# --- Main ---
if __name__ == "__main__":
    # 1. הנתונים הגולמיים (הביטים שלך)
    raw_syndrome = [0,1,1,0]

    # 2. המרה להקס (הלוגיקה שלך)
    hex_result = convert_bits_to_hex_symbols(raw_syndrome)
    print(f"Converted to Hex: {hex_result}")

    # 3. שליחה למיקרו-בקר
    # שנה את 'COM3' לפורט האמיתי שלך!
    send_hex_to_mcu(hex_result, port='COM5', baudrate=9600)
