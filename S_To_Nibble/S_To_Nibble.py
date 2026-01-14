def convert_bits_to_hex_symbols(bits):
    #bits = list(syndrome_bits)
    hex_symbols = []

    for i in range(0, len(bits), 4):
        chunk = bits[i : i + 4]

        # --- התיקון כאן ---
        # מוסיפים אפסים בסוף (Right Padding / Left Justify)
        # זה מבטיח שהביט הראשון ברשימה יישאר ב-MSB וייקרא ראשון ע"י ה-C
        while len(chunk) < 4:
            chunk.append(0)

        val = 0
        for bit in chunk:
            val = (val << 1) | bit

        hex_symbols.append(f'{val:X}')

    return hex_symbols
# --- בדיקה ---
if __name__ == "__main__":
    raw_syndrome = [1, 1, 1, 1, 0, 1, 1,0,1,1,1,1]
    result = convert_bits_to_hex_symbols(raw_syndrome)

    print(f"Original Bits: {raw_syndrome}")
    print(f"Hex Symbols:   {result}")
