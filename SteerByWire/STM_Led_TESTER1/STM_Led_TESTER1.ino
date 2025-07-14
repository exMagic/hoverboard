/* ===================================================================
   TEST PINÓW GPIO Z 4 DIODAMI LED + BUZZER
   Płyta: STM32F103RCT6 (hoverboard mainboard)
   Diody:
     LED_SCK  → PB7
     LED_MOSI → PB5
     LED_MISO → PB10
     LED_CSN  → PB6
   Buzzer  → PA4
   =================================================================== */

#include <Arduino.h>

// definicje pinów
constexpr uint8_t PIN_SCK_LED  = PB7;
constexpr uint8_t PIN_MOSI_LED = PB5;
constexpr uint8_t PIN_MISO_LED = PB10;
constexpr uint8_t PIN_CSN_LED  = PB6;

constexpr uint8_t BUZZER_PIN   = PA4;

// funkcja beep
void beep(uint16_t frequency, uint16_t duration_ms) {
  if (frequency == 0) {
    digitalWrite(BUZZER_PIN, LOW);
    delay(duration_ms);
    return;
  }
  uint32_t period_us = 1000000UL / frequency;
  uint32_t half = period_us / 2;
  uint32_t cycles = (duration_ms * 1000UL) / period_us;
  for (uint32_t i = 0; i < cycles; i++) {
    digitalWrite(BUZZER_PIN, HIGH);
    delayMicroseconds(half);
    digitalWrite(BUZZER_PIN, LOW);
    delayMicroseconds(half);
  }
}

void setup() {
  // konfiguracja pinów jako wyjścia
  pinMode(PIN_SCK_LED,  OUTPUT);
  pinMode(PIN_MOSI_LED, OUTPUT);
  pinMode(PIN_MISO_LED, OUTPUT);
  pinMode(PIN_CSN_LED,  OUTPUT);
  pinMode(BUZZER_PIN,   OUTPUT);
  // wszystkie diody wyłączone
  digitalWrite(PIN_SCK_LED,  LOW);
  digitalWrite(PIN_MOSI_LED, LOW);
  digitalWrite(PIN_MISO_LED, LOW);
  digitalWrite(PIN_CSN_LED,  LOW);
  // startowy 3 beep
  for (int i = 0; i < 3; i++) {
    beep(2000, 150);
    delay(100);
  }
  delay(2000);
}

void loop() {
  // 1 beep -> zapal 2 diody SCK
  beep(2000, 150);
  digitalWrite(PIN_SCK_LED, HIGH);
  delay(100);
  digitalWrite(PIN_SCK_LED, LOW);
  delay(100);
  digitalWrite(PIN_SCK_LED, HIGH);
  delay(100);
  digitalWrite(PIN_SCK_LED, LOW);
  delay(2000);

  // beep -> zapal 3 diody MOSI

  beep(1800, 150);
  digitalWrite(PIN_MOSI_LED, HIGH);
  delay(100);
  digitalWrite(PIN_MOSI_LED, LOW);
  delay(100);
  digitalWrite(PIN_MOSI_LED, HIGH);
  delay(100);
  digitalWrite(PIN_MOSI_LED, LOW);
  delay(2000);


  //

  beep(1600, 150);
  digitalWrite(PIN_MISO_LED, HIGH);
  delay(100);
  digitalWrite(PIN_MISO_LED, LOW);
  delay(100);
  digitalWrite(PIN_MISO_LED, HIGH);
  delay(100);
  digitalWrite(PIN_MISO_LED, LOW);
  delay(2000);

    //

  beep(1400, 150);
  digitalWrite(PIN_CSN_LED, HIGH);
  delay(100);
  digitalWrite(PIN_CSN_LED, LOW);
  delay(100);
  digitalWrite(PIN_CSN_LED, HIGH);
  delay(100);
  digitalWrite(PIN_CSN_LED, LOW);
  delay(2000);

  

  // powtórz cały cykl
}
