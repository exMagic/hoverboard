/* ===================================================================
   STM_SPI_Start1 - Prosty Test Software SPI na STM32F103RCT6
   Płyta   : STM32F103RCT6 (hoverboard mainboard)
   Urządzenie : MT6816 enkoder LUB microSD card adapter
   Piny    : SCK = PB7, MOSI = PB5, MISO = PB10, CS = PB6
   
   UWAGA: Piny PB5, PB6, PB7 mają pull-up 2.2kΩ do 3.3V
   
   Diagnostyka:
   - LED PB2: Miganie wskazuje status
   - Buzzer PA4: 5 beepów = błąd, 3 beepy = sukces
   =================================================================== */

#include <Arduino.h>

// === DEFINICJE PINÓW ===
constexpr uint8_t PIN_SCK  = PB7;   // Serial Clock
constexpr uint8_t PIN_MOSI = PB5;   // Master Out Slave In
constexpr uint8_t PIN_MISO = PB10;  // Master In Slave Out
constexpr uint8_t PIN_CS   = PB6;   // Chip Select

constexpr uint8_t LED_PIN     = PB2;  // LED diagnostyczna
constexpr uint8_t BUZZER_PIN  = PA4;  // Buzzer

// === USTAWIENIA TESTOWE ===
//#define TEST_MT6816     // Odkomentuj dla testu MT6816
#define TEST_MICROSD    // Odkomentuj dla testu microSD

// === FUNKCJE BUZZER I LED ===
void beep(uint16_t frequency, uint16_t duration_ms) {
  uint32_t period_us = 1000000UL / frequency;
  uint32_t half_period = period_us / 2;
  uint32_t cycles = (duration_ms * 1000UL) / period_us;
  
  for (uint32_t i = 0; i < cycles; i++) {
    digitalWrite(BUZZER_PIN, HIGH);
    delayMicroseconds(half_period);
    digitalWrite(BUZZER_PIN, LOW);
    delayMicroseconds(half_period);
  }
}

void signal_error() {
  // 5 beepów = błąd
  for (int i = 0; i < 5; i++) {
    digitalWrite(LED_PIN, HIGH);
    beep(500, 200);
    digitalWrite(LED_PIN, LOW);
    delay(200);
  }
}

void signal_success() {
  // 3 beepy = sukces
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_PIN, HIGH);
    beep(2000, 300);
    digitalWrite(LED_PIN, LOW);
    delay(200);
  }
}

// === SOFTWARE SPI FUNKCJE ===
void spi_init() {
  pinMode(PIN_CS, OUTPUT);
  pinMode(PIN_SCK, OUTPUT);
  pinMode(PIN_MOSI, OUTPUT);
  pinMode(PIN_MISO, INPUT);  // Bez pull-down - pull-up już jest na płycie
  
  // Ustawienia początkowe dla SPI Mode 3 (CPOL=1, CPHA=1)
  digitalWrite(PIN_CS, HIGH);   // CS nieaktywny
  digitalWrite(PIN_SCK, HIGH);  // SCK idle HIGH
  digitalWrite(PIN_MOSI, LOW);  // MOSI idle LOW
  
  delay(10); // Stabilizacja
}

uint8_t spi_transfer(uint8_t data) {
  uint8_t received = 0;
  
  for (int i = 7; i >= 0; i--) {
    // Ustaw dane na MOSI
    digitalWrite(PIN_MOSI, (data >> i) & 1);
    
    // Falling edge - dane się ustawiają
    digitalWrite(PIN_SCK, LOW);
    delayMicroseconds(5);
    
    // Rising edge - dane się próbkują
    digitalWrite(PIN_SCK, HIGH);
    delayMicroseconds(2);
    
    // Odczytaj bit z MISO
    if (digitalRead(PIN_MISO)) {
      received |= (1 << i);
    }
    
    delayMicroseconds(3);
  }
  
  return received;
}

// === TESTY URZĄDZEŃ ===
#ifdef TEST_MT6816
bool test_mt6816() {
  // Test odczytu rejestrów MT6816
  digitalWrite(PIN_CS, LOW);
  delayMicroseconds(5);
  
  // Komenda odczytu rejestru 0x03 (MSB kąta)
  spi_transfer(0x80 | (0x03 << 1));  // Read command + address
  uint8_t reg03 = spi_transfer(0x00);  // Dummy byte do odczytu
  
  digitalWrite(PIN_CS, HIGH);
  delayMicroseconds(20);
  
  // Test odczytu rejestru 0x04 (LSB kąta)
  digitalWrite(PIN_CS, LOW);
  delayMicroseconds(5);
  
  spi_transfer(0x80 | (0x04 << 1));  // Read command + address
  uint8_t reg04 = spi_transfer(0x00);  // Dummy byte do odczytu
  
  digitalWrite(PIN_CS, HIGH);
  delayMicroseconds(20);
  
  // Sprawdź czy otrzymane dane są sensowne
  // MT6816 powinien zwracać dane różne od 0x00 i 0xFF
  if ((reg03 == 0x00 && reg04 == 0x00) || 
      (reg03 == 0xFF && reg04 == 0xFF)) {
    return false;  // Brak komunikacji
  }
  
  return true;  // Komunikacja działa
}
#endif

#ifdef TEST_MICROSD
bool test_microsd() {
  // Test podstawowej komunikacji z microSD
  
  // Wyślij kilka dummy clock cycles (CS HIGH)
  digitalWrite(PIN_CS, HIGH);
  for (int i = 0; i < 10; i++) {
    spi_transfer(0xFF);
  }
  
  // CMD0 - Reset SD card
  digitalWrite(PIN_CS, LOW);
  delayMicroseconds(5);
  
  spi_transfer(0x40);  // CMD0 (01000000)
  spi_transfer(0x00);  // Argument 0
  spi_transfer(0x00);
  spi_transfer(0x00);
  spi_transfer(0x00);
  spi_transfer(0x95);  // CRC for CMD0
  
  // Czekaj na odpowiedź
  uint8_t response = 0xFF;
  for (int i = 0; i < 8; i++) {
    response = spi_transfer(0xFF);
    if (response != 0xFF) break;
  }
  
  digitalWrite(PIN_CS, HIGH);
  spi_transfer(0xFF);  // Dummy clock
  
  // Odpowiedź powinna być 0x01 (idle state)
  return (response == 0x01);
}
#endif

// === TEST PODSTAWOWY SPI ===
bool test_spi_loopback() {
  // Test podstawowy - sprawdź czy piny działają
  
  // Test 1: Sprawdź czy CS działa
  digitalWrite(PIN_CS, HIGH);
  delay(1);
  if (!digitalRead(PIN_CS)) return false;
  
  digitalWrite(PIN_CS, LOW);
  delay(1);
  if (digitalRead(PIN_CS)) return false;
  
  // Test 2: Sprawdź czy SCK działa
  digitalWrite(PIN_SCK, HIGH);
  delay(1);
  if (!digitalRead(PIN_SCK)) return false;
  
  digitalWrite(PIN_SCK, LOW);
  delay(1);
  if (digitalRead(PIN_SCK)) return false;
  
  // Test 3: Sprawdź czy MOSI działa
  digitalWrite(PIN_MOSI, HIGH);
  delay(1);
  if (!digitalRead(PIN_MOSI)) return false;
  
  digitalWrite(PIN_MOSI, LOW);
  delay(1);
  if (digitalRead(PIN_MOSI)) return false;
  
  return true;
}

// === SETUP ===
void setup() {
  // Inicjalizacja pinów
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  
  // Sygnał startowy - krótkie mignięcie
  digitalWrite(LED_PIN, HIGH);
  beep(1500, 200);
  digitalWrite(LED_PIN, LOW);
  delay(500);
  
  // Inicjalizacja SPI
  spi_init();
  
  delay(1000);  // Pauza na stabilizację
}

// === MAIN LOOP ===
void loop() {
  bool test_passed = false;
  
  // Test 1: Podstawowy test pinów
  if (!test_spi_loopback()) {
    signal_error();
    delay(3000);
    return;
  }
  
  // Test 2: Test urządzenia
  #ifdef TEST_MT6816
    test_passed = test_mt6816();
  #endif
  
  #ifdef TEST_MICROSD
    test_passed = test_microsd();
  #endif
  
  // Brak zdefiniowanego testu - domyślnie sukces
  #if !defined(TEST_MT6816) && !defined(TEST_MICROSD)
    test_passed = true;
  #endif
  
  // Sygnalizacja wyniku
  if (test_passed) {
    signal_success();
    delay(5000);  // Długa pauza po sukcesie
  } else {
    signal_error();
    delay(3000);  // Krótsza pauza po błędzie
  }
}
