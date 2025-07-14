#include <Arduino.h>
#include <SPI.h>

constexpr uint8_t PIN_SCK   = PB7;
constexpr uint8_t PIN_MOSI  = PB5;
constexpr uint8_t PIN_MISO  = PB10;
constexpr uint8_t PIN_CS    = PB6;
constexpr uint8_t LED_PIN   = PB2;

SPISettings settings(1000000, MSBFIRST, SPI_MODE0);

uint32_t lastHeartbeat = 0;
uint32_t lastSPITest   = 0;
bool     ledState      = false;

void spiTest() {
  uint8_t patterns[4] = {0xAA, 0x55, 0x0F, 0xF0};
  SPI.beginTransaction(settings);
  digitalWrite(PIN_CS, LOW);
  for (auto b : patterns) {
    SPI.transfer(b);
    delayMicroseconds(10);
  }
  digitalWrite(PIN_CS, HIGH);
  SPI.endTransaction();
}

void setup() {
  pinMode(PIN_CS, OUTPUT);
  digitalWrite(PIN_CS, HIGH);
  SPI.setMOSI(PIN_MOSI);
  SPI.setSCLK(PIN_SCK);
  SPI.setMISO(PIN_MISO);        // <— dokładnie tej linii brakowało
  SPI.begin();
  pinMode(LED_PIN, OUTPUT);
}

void loop() {
  uint32_t now = millis();

  // heartbeat co 1s
  if (now - lastHeartbeat >= 1000) {
    lastHeartbeat = now;
    ledState = !ledState;
    digitalWrite(LED_PIN, ledState);
  }

  // test SPI co 5s
  if (now - lastSPITest >= 5000) {
    lastSPITest = now;
    spiTest();
  }

  // inne nieblokujące zadania…
}
