#include <SPI.h>

const int SYNC_PIN = 10; 
const int SS_PIN = 53;   

// מסגרות האיפוס
uint8_t frame_reset[4]  = {0x00, 0x00, 0x00, 0x00};

// המסגרות המדויקות לאות 'A'
uint8_t frame_A_high[4] = {0x08, 0x80, 0x88, 0x00};
uint8_t frame_A_low[4]  = {0x18, 0x80, 0x01, 0x00};

void sendSpiFrame(uint8_t* frame) {
  SPI.beginTransaction(SPISettings(50000, MSBFIRST, SPI_MODE1));
  
  digitalWrite(SYNC_PIN, LOW);
  delayMicroseconds(10); 
  digitalWrite(SS_PIN, LOW);
  delayMicroseconds(20); 
  
  for (int i = 0; i < 4; i++) {
    SPI.transfer(frame[i]);
  }
  
  digitalWrite(SS_PIN, HIGH);
  digitalWrite(SYNC_PIN, HIGH);
  SPI.endTransaction();
  
  delay(50); 
}

void setup() {
  // 1. ייצוב קשיח ומהיר של קווי הנתונים והשעון מיד עם עליית הארדואינו
  pinMode(52, OUTPUT); // SCK
  pinMode(51, OUTPUT); // MOSI
  digitalWrite(52, LOW); 
  digitalWrite(51, LOW);

  pinMode(SYNC_PIN, OUTPUT);
  pinMode(SS_PIN, OUTPUT);
  digitalWrite(SYNC_PIN, HIGH);
  digitalWrite(SS_PIN, HIGH);
  
  SPI.begin();
  
  // 2. השהייה קטנה להתייצבות מתחים אחרי הריסט
  delay(500); 

  // 3. שלב א': שולחים 4 מסגרות איפוס ליישור ודאי של מכונת המצבים של ה-841
  // זה נועד לתקן רעשים שאולי נכנסו בזמן שהאצבע שלך לחצה על הריסט
  for(int i = 0; i < 4; i++) {
    sendSpiFrame(frame_reset);
  }
  
  delay(150); 

  // 4. שלב ב': שליחת האות 'A'
  sendSpiFrame(frame_A_high);
  sendSpiFrame(frame_A_low);
}

void loop() {
  // התוכנית עוצרת כאן. 
  // האות A תישלח שוב אך ורק בפעם הבאה שתלחץ על כפתור ה-Reset.
}