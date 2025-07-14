/* --- MT6816 4-Wire SPI on ESP32 --- */
#include <SPI.h>

// Pinmap
constexpr uint8_t PIN_SCK  = 18;
constexpr uint8_t PIN_MOSI = 23;
constexpr uint8_t PIN_MISO = 19;
constexpr uint8_t PIN_CSN  = 5;

// Opcjonalnie podnieś SCK na 16 MHz przy stabilnym okablowaniu
constexpr uint32_t FREQ_HZ = 8000000; // 8 MHz

// Rejestry
constexpr uint8_t REG_ANGLE_MSB = 0x03;
constexpr uint8_t REG_ANGLE_LSB = 0x04;

SPIClass spi(VSPI);

void setup() {
  Serial.begin(115200);
  pinMode(PIN_CSN, OUTPUT);
  digitalWrite(PIN_CSN, HIGH);

  spi.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CSN);
  spi.beginTransaction(SPISettings(FREQ_HZ, MSBFIRST, SPI_MODE3));
}

uint16_t readAngle() {
  uint8_t msb, lsb;

  digitalWrite(PIN_CSN, LOW);
  spi.transfer(REG_ANGLE_MSB | 0x80);   // bit7=1 = Read
  msb = spi.transfer(0x00);             // dummy
  digitalWrite(PIN_CSN, HIGH);

  digitalWrite(PIN_CSN, LOW);
  spi.transfer(REG_ANGLE_LSB | 0x80);
  lsb = spi.transfer(0x00);
  digitalWrite(PIN_CSN, HIGH);

  return ((msb << 8) | lsb) & 0x3FFF;   // 14 bit
}

void loop() {
  float deg = readAngle() * 360.0 / 16383.0;
  Serial.printf("Kąt: %.2f°\n", deg);
  delay(50);
}
