#include <SPI.h>

volatile bool dataReady = false;
volatile uint8_t receivedByte = 0;

void setup() {
  pinMode(MISO, OUTPUT);      // pin 12 jako MISO
  
  pinMode(10, INPUT);  // SS pin jako INPUT_PULLUP
  SPI.begin();                // hardware SPI jako slave
  SPCR |= _BV(SPIE);          // włącz przerwanie SPI
  Serial.begin(115200);
  Serial.println("start");
}

ISR(SPI_STC_vect) {
  receivedByte = SPDR;  // odczytaj bajt
  dataReady = true;
}

void loop() {
  // Serial.print("SS= ");
  // Serial.println(digitalRead(10));
  // delay(500);
  if (dataReady) {
    Serial.print("Odebrano: 0x");
    if (receivedByte < 0x10) Serial.print('0');
    Serial.println(receivedByte, HEX);
    dataReady = false;
  }
}
