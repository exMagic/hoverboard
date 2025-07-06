// === TEST POJEDYNCZEGO MT6816 Z SOFTWARE SPI ===
// Płyta: STM32F103RCT6 Hoverboard  
// Enkoder: MT6816 podłączony do pinów Hall sensors
// Diagnostyka: Buzzer na PA4

// === DEFINICJE PINÓW ===
#define SCK_PIN PB7     // Serial Clock (Hall sensor line)
#define CSN_PIN PB6     // Chip Select (Hall sensor line)  
#define MISO_PIN PB10   // Master In Slave Out (sideboard pin)
#define BUZZER_PIN PA4  // Buzzer/Speaker
#define LED_PIN PC13    // LED na płycie głównej

// === DEFINICJE CZĘSTOTLIWOŚCI BUZZER ===
#define BEEP_SUCCESS 2000    // Wysoka częstotliwość - sukces
#define BEEP_WARNING 1000    // Średnia częstotliwość - ostrzeżenie  
#define BEEP_ERROR 500       // Niska częstotliwość - błąd
#define BEEP_DATA 1500       // Częstotliwość dla danych

// === STRUKTURA STANU MT6816 ===
struct MT6816_Status {
  bool communication_ok;
  bool magnet_detected;
  bool angle_changing;
  uint16_t raw_angle;
  float angle_degrees;
  uint8_t reg03;
  uint8_t reg04;
};

// === FUNKCJE BUZZER ===
void beep(uint16_t frequency, uint16_t duration_ms) {
  if (frequency == 0) {
    digitalWrite(BUZZER_PIN, LOW);
    delay(duration_ms);
    return;
  }
  
  uint32_t period_us = 1000000 / frequency;
  uint32_t half_period = period_us / 2;
  uint32_t cycles = (duration_ms * 1000UL) / period_us;
  
  for (uint32_t i = 0; i < cycles; i++) {
    digitalWrite(BUZZER_PIN, HIGH);
    delayMicroseconds(half_period);
    digitalWrite(BUZZER_PIN, LOW);
    delayMicroseconds(half_period);
  }
}

// === WZORCE DIAGNOSTYCZNE BUZZER ===
void beep_startup() {
  // Sygnał startowy - długi ton średni
  beep(BEEP_DATA, 800);
  delay(200);
}

void beep_no_communication() {
  // 5 krótkich wysokich beepów = brak komunikacji SPI
  for (int i = 0; i < 5; i++) {
    beep(BEEP_SUCCESS, 100);
    delay(350);
  }
}

void beep_no_magnet() {
  // 3 długie niskie beepy = brak magnesu (No_Mag_Warning)
  for (int i = 0; i < 3; i++) {
    beep(BEEP_ERROR, 400);
    delay(200);
  }
}

void beep_sensor_working() {
  // 2 średnie beepy = enkoder działa poprawnie
  for (int i = 0; i < 2; i++) {
    beep(BEEP_WARNING, 250);
    delay(150);
  }
}

void beep_angle_data(float angle_degrees) {
  // Odegraj kąt jako zmienną częstotliwość (200-3000 Hz)
  uint16_t freq = 200 + (uint16_t)(angle_degrees * 8);
  if (freq > 3000) freq = 3000;
  beep(freq, 300);
}

void beep_constant_reading() {
  // 1 długi średni beep = stałe odczyty (brak ruchu)
  beep(BEEP_WARNING, 600);
}

// === SOFTWARE SPI DLA MT6816 ===
void spi_init() {
  pinMode(CSN_PIN, OUTPUT);
  pinMode(SCK_PIN, OUTPUT);
  pinMode(MISO_PIN, INPUT_PULLDOWN); // Pull-down przeciwko zewnętrznym pull-up
  
  // SPI Mode 3: CPOL=1, CPHA=1 (idle HIGH)
  digitalWrite(CSN_PIN, HIGH);  // CSN idle HIGH
  digitalWrite(SCK_PIN, HIGH);  // SCK idle HIGH
  
  delay(50); // Stabilizacja MT6816
}

uint8_t spi_read_register(uint8_t reg_addr) {
  digitalWrite(CSN_PIN, LOW);  // Start transmisji
  delayMicroseconds(10);       // Wolniejsze SPI dla pull-up
  
  // Komenda odczytu MT6816: 0x80 | (reg_addr << 1)
  uint8_t command = 0x80 | (reg_addr << 1);
  
  // Wyślij komendę (8 bitów)
  for (int i = 7; i >= 0; i--) {
    digitalWrite(SCK_PIN, LOW);
    delayMicroseconds(10);
    digitalWrite(SCK_PIN, HIGH);
    delayMicroseconds(10);
  }
  
  // Odbierz dane (8 bitów)
  uint8_t data = 0;
  for (int i = 7; i >= 0; i--) {
    digitalWrite(SCK_PIN, LOW);
    delayMicroseconds(10);
    digitalWrite(SCK_PIN, HIGH);
    delayMicroseconds(5);
    
    if (digitalRead(MISO_PIN)) {
      data |= (1 << i);
    }
    delayMicroseconds(5);
  }
  
  digitalWrite(CSN_PIN, HIGH);  // Koniec transmisji
  delayMicroseconds(20);
  
  return data;
}

// === FUNKCJE DIAGNOSTYCZNE ===
MT6816_Status diagnose_mt6816() {
  MT6816_Status status = {false, false, false, 0, 0.0, 0, 0};
  
  // Odczytaj rejestry pozycji MT6816 (0x03 i 0x04)
  status.reg03 = spi_read_register(0x03);
  delay(10);
  status.reg04 = spi_read_register(0x04);
  
  // Test komunikacji - sprawdź czy otrzymujemy dane inne niż 0x00/0xFF
  if ((status.reg03 != 0x00 && status.reg03 != 0xFF) || 
      (status.reg04 != 0x00 && status.reg04 != 0xFF)) {
    status.communication_ok = true;
  }
  
  // Test detekcji magnesu - sprawdź No_Mag_Warning (bit 1 w reg04)
  status.magnet_detected = !(status.reg04 & 0x02);
  
  // Oblicz kąt z 14-bitowych danych
  if (status.communication_ok) {
    status.raw_angle = ((uint16_t)status.reg03 << 8) | status.reg04;
    status.raw_angle &= 0x3FFF; // Maska 14-bit (0-16383)
    status.angle_degrees = (status.raw_angle * 360.0) / 16384.0;
  }
  
  return status;
}

bool test_angle_changes() {
  static uint16_t previous_angle = 0;
  static bool first_reading = true;
  
  MT6816_Status status = diagnose_mt6816();
  
  if (!status.communication_ok) return false;
  
  if (first_reading) {
    previous_angle = status.raw_angle;
    first_reading = false;
    return false;
  }
  
  // Sprawdź zmianę kąta (tolerancja na szum: 20 LSB ≈ 0.44°)
  uint16_t diff = abs((int)status.raw_angle - (int)previous_angle);
  if (diff > 20) {
    previous_angle = status.raw_angle;
    return true;
  }
  
  return false;
}

// === GŁÓWNE FUNKCJE ===
void setup() {
  // Inicjalizacja pinów
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  
  // Sygnał startowy
  digitalWrite(LED_PIN, HIGH);
  beep_startup();
  digitalWrite(LED_PIN, LOW);
  
  // Inicjalizacja Software SPI
  spi_init();
  
  delay(1000); // Pauza przed pierwszym testem
}

void loop() {
  static int test_cycle = 0;
  test_cycle++;
  
  // Migaj LED co cykl testowy
  digitalWrite(LED_PIN, test_cycle % 2);
  
  // === GŁÓWNY TEST DIAGNOSTYCZNY ===
  
  // Test 1: Podstawowa komunikacja SPI
  MT6816_Status status = diagnose_mt6816();
  
  if (!status.communication_ok) {
    // BŁĄD: Brak komunikacji SPI
    beep_no_communication();
    delay(3000);
    return;
  }
  
  // Test 2: Detekcja magnesu
  if (!status.magnet_detected) {
    // BŁĄD: No_Mag_Warning = 1 (brak magnesu)
    beep_no_magnet();
    delay(3000);
    return;
  }
  
  // Test 3: Detekcja ruchu (3 sekundy obserwacji)
  bool angle_changed = false;
  for (int i = 0; i < 30; i++) { // 30 x 100ms = 3s
    if (test_angle_changes()) {
      angle_changed = true;
      break;
    }
    delay(100);
  }
  
  // Test 4: Wyniki diagnostyki
  if (angle_changed) {
    // SUKCES: Enkoder działa i wykrywa ruch
    beep_sensor_working();
    delay(500);
    
    // Bonus: Odegraj aktualny kąt jako częstotliwość
    beep_angle_data(status.angle_degrees);
    
  } else {
    // OSTRZEŻENIE: Enkoder działa ale brak ruchu
    beep_constant_reading();
  }
  
  delay(4000); // Pauza między cyklami testowymi
}
