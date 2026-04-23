/* File: mega_simulator.ino 
 * Target: Arduino Mega 2560
 */

#define PKT_START 0xAA
#define PKT_END   0x55

void setup() {
  Serial.begin(115200); // Debug
  Serial1.begin(9600);  // ADuC841 Connection (Pin 18 TX)
  Serial.println("Mega Simulator Ready - Sending 'A' periodically.");
}

void send_encoded_packet(uint16_t data, uint16_t red, uint8_t parity) {
  uint8_t payload[4];
  uint8_t crc = 0;

  payload[0] = data & 0xFF;
  payload[1] = ((data >> 8) & 0x7F) | (parity << 7);
  payload[2] = red & 0xFF;
  payload[3] = (red >> 8) & 0x03;

  Serial1.write(PKT_START);
  for(int i=0; i<4; i++) {
    Serial1.write(payload[i]);
    crc ^= payload[i];
  }
  Serial1.write(crc);
  Serial1.write(PKT_END);
}

void loop() {
  // Simulating 'A' (0x41) -> High Nibble 4, Low Nibble 1
  // These values represent the physical bus state after H1 encoding
  
  // High Nibble (0x4) -> Bit 3 of bus flips
  send_encoded_packet(0x0008, 0x007C, 1); 
  delay(10);
  
  // Low Nibble (0x1) -> Bit 0 of bus flips
  send_encoded_packet(0x0009, 0x001F, 0); 
  
  delay(2000);
}