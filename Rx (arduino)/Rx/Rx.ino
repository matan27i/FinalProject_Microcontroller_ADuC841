/*
 * 74HC595 Latch-Triggered Reader
 * Reads data ONLY when SR_LATCH goes HIGH
 */

// --- הגדרת פינים ---
const int PIN_LATCH_TRIGGER = 2; // פין הפסיקה (חייב להיות 2 או 3 ב-Uno/Nano)

// פיני המידע החדשים (הוזזו כדי לפנות את פין 2)
// סדר: Q0, Q1, Q2 ... Q14
const int dataPins[] = {
  3, 4, 5, 6, 7, 8, 9, 10,   // Bits 0-7 (Register 1)
  11, 12, 13, A0, A1, A2, A3 // Bits 8-14 (Register 2)
};

const int TOTAL_BITS = 15;

// משתנה נדיף (Volatile) לתקשורת בין הפסיקה ללולאה הראשית
volatile bool dataReady = false;

void setup() {
  Serial.begin(9600); // קצב מהיר
  Serial.println("--- Waiting for Latch Trigger... ---");

  // הגדרת פין הטריגר ככניסה
  pinMode(PIN_LATCH_TRIGGER, INPUT);

  // הגדרת פיני המידע
  for (int i = 0; i < TOTAL_BITS; i++) {
    pinMode(dataPins[i], INPUT);
  }

  // --- הגדרת הפסיקה (The Magic) ---
  // ברגע שפין 2 מזהה עלייה (RISING) מ-0 ל-1, הפונקציה onLatchRise תופעל
  attachInterrupt(digitalPinToInterrupt(PIN_LATCH_TRIGGER), onLatchRise, RISING);
}

// פונקציית הפסיקה (ISR) - רצה כשהלאץ' עולה
void onLatchRise() {
  dataReady = true; // הרמת דגל: "יש מידע חדש!"
}

void loop() {
  // בדיקה אם הדגל הורם בפסיקה
  if (dataReady) {
    // השהייה קטנטנה לוודא שהנתונים התייצבו (לא חובה, לבטיחות)
    delayMicroseconds(10); 
    
    unsigned int currentVal = 0;

    // קריאת הפינים
    for (int i = 0; i < TOTAL_BITS; i++) {
      if (digitalRead(dataPins[i]) == HIGH) {
        currentVal |= (1 << i);
      }
    }

    // הדפסה
    printFormatted(currentVal);

    // הורדת הדגל - מחכים ללאץ' הבא
    dataReady = false; 
  }
}

void printFormatted(unsigned int val) {
  Serial.print("Update Detected -> Binary: ");
  
  // הדפסה בינארית יפה (MSB משמאל)
  for (int i = TOTAL_BITS - 1; i >= 0; i--) {
    Serial.print((val >> i) & 1);
    if (i == 8) Serial.print(" "); // רווח בין האוגרים
  }
  
  Serial.print(" | Hex: 0x");
  Serial.println(val, HEX);
}