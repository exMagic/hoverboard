#include <SPI.h>

const int CS_PIN = 10;

void setup() {
  Serial.begin(9600);
  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH); // idle state

  SPI.begin();
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1)); // MT6816 prefers MODE1
}

void loop() {
  uint16_t angle = readMT6816();
  float degrees = (angle * 360.0) / 16384.0;

  Serial.print("Raw: ");
  Serial.print(angle);
  Serial.print(" -> Angle (°): ");
  Serial.println(degrees, 2);

  delay(300);
}

uint16_t readMT6816() {
  uint8_t highByte, lowByte;
  uint16_t value = 0;

  digitalWrite(CS_PIN, LOW);
  delayMicroseconds(1); // short pulse

  highByte = SPI.transfer(0x00);
  lowByte  = SPI.transfer(0x00);

  digitalWrite(CS_PIN, HIGH);
  delayMicroseconds(1); // ensure CS returns HIGH

  value = ((highByte << 8) | lowByte) & 0x3FFF;
  return value;
}
