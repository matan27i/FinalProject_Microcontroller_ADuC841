#include <SPI.h>

const int SYNC_PIN = 10; 
const int SS_PIN = 53;   

uint8_t frame_reset[4]  = {0x00, 0x00, 0x00, 0x00};
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
  // 1. ייצוב קשיח של קווי הנתונים והשעון מיד בהדלקה כדי למנוע רעשים אלקטרומגנטיים
  pinMode(52, OUTPUT); // SCK
  pinMode(51, OUTPUT); // MOSI
  digitalWrite(52, LOW); // ב-SPI MODE 1 השעון במנוחה צריך להיות נמוך
  digitalWrite(51, LOW);

  pinMode(SYNC_PIN, OUTPUT);
  pinMode(SS_PIN, OUTPUT);
  digitalWrite(SYNC_PIN, HIGH);
  digitalWrite(SS_PIN, HIGH);
  
  SPI.begin();
}

void loop() {
  // המתנה של שנייה 1 (1000 מילישניות)
  // ביחד עם 450 מילישניות של שידורים, המחזור יקח כ-1.45 שניות.
  delay(1000); 

  // שלב א': שליחת 4 מסגרות איפוס ליישור ודאי
  for(int i = 0; i < 4; i++) {
    sendSpiFrame(frame_reset);
  }
  
  delay(150); 

  // שלב ב': שליחת האות 'A'
  sendSpiFrame(frame_A_high);
  sendSpiFrame(frame_A_low);
}