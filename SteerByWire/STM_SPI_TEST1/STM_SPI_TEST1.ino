/* ===================================================================
   ROZSZERZONA DIAGNOSTYKA MT6816 – SZCZEGÓŁOWE WZORCE BEEP
   Płyta   :  STM32F103RCT6 (hoverboard mainboard)
   Enkoder :  MT6816-STD
   Piny    :  SCK  = PB7, CS = PB6, MOSI = PB5, MISO = PB10
   Diagnostyka: Buzzer PA4 + LED PB2
   =================================================================== */

#include <SPI.h>

// === DEFINICJE PINÓW ===
constexpr uint8_t PIN_SCK  = PB7;   
constexpr uint8_t PIN_MOSI = PB5;   
constexpr uint8_t PIN_MISO = PB10;  
constexpr uint8_t PIN_CSN  = PB6;   

constexpr uint8_t LED_PIN     = PB2;
constexpr uint8_t BUZZER_PIN  = PA4;

// === MT6816 REJESTRY ===
constexpr uint8_t REG_ANGLE_MSB = 0x03;
constexpr uint8_t REG_ANGLE_LSB = 0x04;
constexpr uint32_t FREQ_HZ      = 1000000UL;   // 1 MHz

SPISettings mt6816_settings(FREQ_HZ, MSBFIRST, SPI_MODE3);

// === CZĘSTOTLIWOŚCI BEEP DLA RÓŻNYCH STANÓW ===
#define BEEP_STARTUP      2000  // Ton startowy
#define BEEP_SPI_OK       1800  // SPI działa
#define BEEP_NO_COMM      500   // Brak komunikacji
#define BEEP_MAGNET_OK    1500  // Magnet wykryty
#define BEEP_NO_MAGNET    300   // Brak magnesu
#define BEEP_ANGLE_GOOD   2200  // Kąt się zmienia
#define BEEP_ANGLE_STATIC 800   // Kąt statyczny
#define BEEP_ERROR        200   // Ogólny błąd
#define BEEP_TEST         1000  // Test sprzętu

// === STRUKTURA STANU ===
struct DiagnosticState {
  bool spi_responding;
  bool magnet_detected;
  bool angle_changing;
  bool pins_connected;
  uint16_t raw_angle;
  float angle_degrees;
  uint8_t reg03;
  uint8_t reg04;
  uint8_t error_count;
  uint32_t last_angle_change;
};

DiagnosticState diag_state = {false, false, false, false, 0, 0.0, 0, 0, 0, 0};

// === FUNKCJE BEEP Z RÓŻNYMI WZORCAMI ===
void beep_tone(uint16_t frequency, uint16_t duration_ms) {
  if (frequency == 0) {
    digitalWrite(BUZZER_PIN, LOW);
    delay(duration_ms);
    return;
  }
  
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

void led_blink(uint8_t count, uint16_t on_time = 100, uint16_t off_time = 100) {
  for (uint8_t i = 0; i < count; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(on_time);
    digitalWrite(LED_PIN, LOW);
    delay(off_time);
  }
}

// === WZORCE DIAGNOSTYCZNE ===
void diagnostic_startup() {
  // Sekwencja startowa: 3 rosnące tony + LED
  led_blink(1, 500);
  beep_tone(BEEP_STARTUP, 200);
  delay(100);
  beep_tone(BEEP_STARTUP + 300, 200);
  delay(100);
  beep_tone(BEEP_STARTUP + 600, 300);
  delay(500);
}

void diagnostic_pin_test() {
  // Test połączeń pinów - 5 krótkich beepów z LED
  led_blink(5, 50, 150);
  for (int i = 0; i < 5; i++) {
    beep_tone(BEEP_TEST, 100);
    delay(150);
  }
}

void diagnostic_spi_no_response() {
  // Brak odpowiedzi SPI - 7 bardzo niskich tonów
  led_blink(7, 100, 200);
  for (int i = 0; i < 7; i++) {
    beep_tone(BEEP_NO_COMM, 200);
    delay(300);
  }
}

void diagnostic_spi_working() {
  // SPI działa - 2 wysokie tony
  led_blink(2, 200);
  beep_tone(BEEP_SPI_OK, 250);
  delay(200);
  beep_tone(BEEP_SPI_OK, 250);
}

void diagnostic_no_magnet() {
  // Brak magnesu - 4 długie niskie tony
  led_blink(4, 400, 300);
  for (int i = 0; i < 4; i++) {
    beep_tone(BEEP_NO_MAGNET, 600);
    delay(400);
  }
}

void diagnostic_magnet_detected() {
  // Magnet wykryty - 3 średnie tony
  led_blink(3, 250);
  for (int i = 0; i < 3; i++) {
    beep_tone(BEEP_MAGNET_OK, 300);
    delay(200);
  }
}

void diagnostic_angle_changing() {
  // Kąt się zmienia - melodia rosnąca + ton reprezentujący kąt
  led_blink(1, 1000);
  beep_tone(BEEP_ANGLE_GOOD, 200);
  delay(100);
  beep_tone(BEEP_ANGLE_GOOD + 200, 200);
  delay(100);
  
  // Ton reprezentujący kąt (200-3000 Hz)
  uint16_t angle_freq = 200 + (uint16_t)(diag_state.angle_degrees * 8);
  if (angle_freq > 3000) angle_freq = 3000;
  beep_tone(angle_freq, 400);
}

void diagnostic_angle_static() {
  // Kąt statyczny - 1 długi średni ton
  led_blink(1, 800);
  beep_tone(BEEP_ANGLE_STATIC, 1000);
}

void diagnostic_multiple_errors() {
  // Wiele błędów - alarm wzorowy
  for (int i = 0; i < 3; i++) {
    led_blink(10, 50, 50);
    beep_tone(BEEP_ERROR, 100);
    delay(100);
    beep_tone(BEEP_ERROR + 100, 100);
    delay(100);
  }
}

void diagnostic_raw_data_output() {
  // Wyświetl surowe dane przez serie beepów
  // REG03 jako liczba krótkich beepów (górne 4 bity)
  uint8_t upper_nibble = (diag_state.reg03 >> 4) & 0x0F;
  for (int i = 0; i < upper_nibble; i++) {
    beep_tone(1500, 100);
    delay(150);
  }
  delay(500);
  
  // REG04 jako liczba długich beepów (dolne 4 bity)
  uint8_t lower_nibble = diag_state.reg04 & 0x0F;
  for (int i = 0; i < lower_nibble; i++) {
    beep_tone(800, 300);
    delay(200);
  }
}

// === FUNKCJE TESTOWANIA SPRZĘTU ===
bool test_pin_connectivity() {
  // Test czy piny odpowiadają
  pinMode(PIN_CSN, OUTPUT);
  pinMode(PIN_SCK, OUTPUT);
  pinMode(PIN_MOSI, OUTPUT);
  pinMode(PIN_MISO, INPUT);
  
  // Test CS - czy możemy kontrolować pin
  digitalWrite(PIN_CSN, HIGH);
  delay(1);
  bool cs_high = digitalRead(PIN_CSN);
  digitalWrite(PIN_CSN, LOW);
  delay(1);
  bool cs_low = !digitalRead(PIN_CSN);
  
  // Test SCK
  digitalWrite(PIN_SCK, HIGH);
  delay(1);
  bool sck_high = digitalRead(PIN_SCK);
  digitalWrite(PIN_SCK, LOW);
  delay(1);
  bool sck_low = !digitalRead(PIN_SCK);
  
  return (cs_high && cs_low && sck_high && sck_low);
}

uint16_t read_angle_with_retry() {
  uint8_t msb, lsb;
  uint8_t retry_count = 0;
  
  do {
    SPI.beginTransaction(mt6816_settings);
    
    // Odczyt MSB
    digitalWrite(PIN_CSN, LOW);
    delayMicroseconds(5);
    SPI.transfer(REG_ANGLE_MSB | 0x80);
    msb = SPI.transfer(0x00);
    digitalWrite(PIN_CSN, HIGH);
    delayMicroseconds(10);
    
    // Odczyt LSB
    digitalWrite(PIN_CSN, LOW);
    delayMicroseconds(5);
    SPI.transfer(REG_ANGLE_LSB | 0x80);
    lsb = SPI.transfer(0x00);
    digitalWrite(PIN_CSN, HIGH);
    
    SPI.endTransaction();
    
    diag_state.reg03 = msb;
    diag_state.reg04 = lsb;
    
    retry_count++;
    
    // Sprawdź czy dane są sensowne
    if ((msb != 0x00 && msb != 0xFF) || (lsb != 0x00 && lsb != 0xFF)) {
      return ((uint16_t(msb) << 8) | lsb) & 0x3FFF;
    }
    
    delay(10); // Krótka pauza przed ponowną próbą
    
  } while (retry_count < 3);
  
  return 0; // Niepowodzenie
}

void comprehensive_diagnostics() {
  static uint32_t last_diagnostic = 0;
  static uint16_t previous_angle = 0;
  static bool first_reading = true;
  
  if (millis() - last_diagnostic < 3000) return; // Diagnostyka co 3 sekundy
  last_diagnostic = millis();
  
  // === FAZA 1: TEST POŁĄCZEŃ PINÓW ===
  diag_state.pins_connected = test_pin_connectivity();
  if (!diag_state.pins_connected) {
    diagnostic_multiple_errors();
    return;
  }
  
  // === FAZA 2: TEST KOMUNIKACJI SPI ===
  uint16_t raw_data = read_angle_with_retry();
  diag_state.raw_angle = raw_data;
  
  if (raw_data == 0) {
    diag_state.spi_responding = false;
    diag_state.error_count++;
    diagnostic_spi_no_response();
    return;
  } else {
    diag_state.spi_responding = true;
    diag_state.error_count = 0;
    diagnostic_spi_working();
    delay(500);
  }
  
  // === FAZA 3: TEST DETEKCJI MAGNESU ===
  diag_state.magnet_detected = !(diag_state.reg04 & 0x02); // No_Mag_Warning bit
  
  if (!diag_state.magnet_detected) {
    diagnostic_no_magnet();
    return;
  } else {
    diagnostic_magnet_detected();
    delay(500);
  }
  
  // === FAZA 4: TEST ZMIAN KĄTA ===
  diag_state.angle_degrees = (diag_state.raw_angle * 360.0f) / 16383.0f;
  
  if (!first_reading) {
    uint16_t angle_diff = abs((int)diag_state.raw_angle - (int)previous_angle);
    if (angle_diff > 50) { // Tolerancja ~1.1°
      diag_state.angle_changing = true;
      diag_state.last_angle_change = millis();
    }
  } else {
    first_reading = false;
  }
  
  previous_angle = diag_state.raw_angle;
  
  // Sprawdź czy były zmiany w ostatnich 10 sekundach
  bool recent_change = (millis() - diag_state.last_angle_change) < 10000;
  
  if (diag_state.angle_changing && recent_change) {
    diagnostic_angle_changing();
  } else {
    diagnostic_angle_static();
  }
  
  delay(1000);
  
  // === FAZA 5: SUROWE DANE (OPCJONALNE) ===
  diagnostic_raw_data_output();
}

// === SETUP ===
void setup() {
  pinMode(PIN_CSN, OUTPUT);
  digitalWrite(PIN_CSN, HIGH);
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  
  // Sekwencja startowa
  diagnostic_startup();
  
  // Test połączeń pinów
  diagnostic_pin_test();
  
  beep_tone(BEEP_TEST, 200);  // np. 1000 Hz przez 200 ms

  // Konfiguracja SPI
  SPI.setMOSI(PIN_MOSI);
  SPI.setMISO(PIN_MISO);
  SPI.setSCLK(PIN_SCK);
  SPI.begin();
  
  delay(1000);
}

// === MAIN LOOP ===
void loop() {
  // Główna diagnostyka co 3 sekundy
  comprehensive_diagnostics();
  
  // Krótkie mignięcie LED co sekundę = system żyje
  static uint32_t last_heartbeat = 0;
  if (millis() - last_heartbeat > 1000) {
    last_heartbeat = millis();
    led_blink(1, 150);
    
  }
  
  delay(100);
}
