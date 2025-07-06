// === TEST MT6816 Z ARDUINO UNO ===
// Diagnostyka przez wbudowaną LED (pin 13)
// Software SPI dla MT6816

// === DEFINICJE PINÓW ===
#define SCK_PIN 7      // Serial Clock - dowolny pin digital
#define CSN_PIN 6      // Chip Select - dowolny pin digital  
#define MISO_PIN 5     // Master In Slave Out - dowolny pin digital
#define LED_PIN LED_BUILTIN // Wbudowana LED Arduino Uno (pin 13)

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

// === WZORCE DIAGNOSTYCZNE LED ===
void led_startup() {
  // Sygnał startowy - długo świeci (2 sekundy)
  digitalWrite(LED_PIN, HIGH);
  delay(2000);
  digitalWrite(LED_PIN, LOW);
  delay(500);
}

void led_no_communication() {
  // 5 krótkich błyśnięć = brak komunikacji SPI
  for (int i = 0; i < 5; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(200);
    digitalWrite(LED_PIN, LOW);
    delay(400);
  }
}

void led_no_magnet() {
  // 3 długie błyśnięcia = brak magnesu (No_Mag_Warning)
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(800);
    digitalWrite(LED_PIN, LOW);
    delay(300);
  }
}

void led_sensor_working() {
  // 2 średnie błyśnięcia = enkoder działa poprawnie
  for (int i = 0; i < 2; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(400);
    digitalWrite(LED_PIN, LOW);
    delay(200);
  }
}

void led_angle_data(float angle_degrees) {
  // Reprezentacja kąta przez częstotliwość migania
  // 0-90° = 50ms błyski, 90-180° = 100ms błyski, 
  // 180-270° = 150ms błyski, 270-360° = 200ms błyski
  uint16_t blink_duration;
  
  if (angle_degrees < 90) {
    blink_duration = 50;
  } else if (angle_degrees < 180) {
    blink_duration = 100;
  } else if (angle_degrees < 270) {
    blink_duration = 150;
  } else {
    blink_duration = 200;
  }
  
  // Pokaż kąt przez 10 błyśnięć
  for (int i = 0; i < 10; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(blink_duration);
    digitalWrite(LED_PIN, LOW);
    delay(blink_duration);
  }
}

void led_constant_reading() {
  // 1 bardzo długie błyśnięcie = stałe odczyty (brak ruchu)
  digitalWrite(LED_PIN, HIGH);
  delay(1500);
  digitalWrite(LED_PIN, LOW);
}

// === SOFTWARE SPI DLA MT6816 ===
void spi_init() {
  pinMode(CSN_PIN, OUTPUT);
  pinMode(SCK_PIN, OUTPUT);
  pinMode(MISO_PIN, INPUT); // Arduino Uno nie ma problemów z pull-up
  
  // SPI Mode 3: CPOL=1, CPHA=1 (idle HIGH)
  digitalWrite(CSN_PIN, HIGH);  // CSN idle HIGH
  digitalWrite(SCK_PIN, HIGH);  // SCK idle HIGH
  
  delay(100); // Stabilizacja MT6816 po zasileniu
}

uint8_t spi_read_register(uint8_t reg_addr) {
  digitalWrite(CSN_PIN, LOW);  // Start transmisji
  delayMicroseconds(5);        // Krótkie opóźnienie
  
  // Komenda odczytu MT6816: 0x80 | (reg_addr << 1)
  uint8_t command = 0x80 | (reg_addr << 1);
  
  // Wyślij komendę (8 bitów)
  for (int i = 7; i >= 0; i--) {
    digitalWrite(SCK_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(SCK_PIN, HIGH);
    delayMicroseconds(2);
  }
  
  // Odbierz dane (8 bitów)
  uint8_t data = 0;
  for (int i = 7; i >= 0; i--) {
    digitalWrite(SCK_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(SCK_PIN, HIGH);
    delayMicroseconds(1);
    
    if (digitalRead(MISO_PIN)) {
      data |= (1 << i);
    }
    delayMicroseconds(1);
  }
  
  digitalWrite(CSN_PIN, HIGH);  // Koniec transmisji
  delayMicroseconds(10);
  
  return data;
}

// === FUNKCJE DIAGNOSTYCZNE ===
MT6816_Status diagnose_mt6816() {
  MT6816_Status status = {false, false, false, 0, 0.0, 0, 0};
  
  // Odczytaj rejestry pozycji MT6816 (0x03 i 0x04)
  status.reg03 = spi_read_register(0x03);
  delay(5);
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

// === MAIN FUNCTIONS ===
void setup() {
  // Inicjalizacja LED
  pinMode(LED_PIN, OUTPUT);
  
  // Sygnał startowy
  led_startup();
  
  // Inicjalizacja Software SPI
  spi_init();
  
  // Opcjonalnie: Serial dla debug jeśli masz podłączony PC
  Serial.begin(115200);
  Serial.println("MT6816 Test Started on Arduino Uno");
  
  delay(1000); // Pauza przed pierwszym testem
}

void loop() {
  static int test_cycle = 0;
  test_cycle++;
  
  // === GŁÓWNY TEST DIAGNOSTYCZNY ===
  
  // Test 1: Podstawowa komunikacja SPI
  MT6816_Status status = diagnose_mt6816();
  
  // Debug output (jeśli Serial Monitor jest dostępny)
  if (Serial) {
    Serial.print("Cycle ");
    Serial.print(test_cycle);
    Serial.print(": REG03=0x");
    Serial.print(status.reg03, HEX);
    Serial.print(", REG04=0x");
    Serial.print(status.reg04, HEX);
    Serial.print(", Comm=");
    Serial.print(status.communication_ok ? "OK" : "FAIL");
    Serial.print(", Magnet=");
    Serial.print(status.magnet_detected ? "OK" : "FAIL");
    Serial.print(", Angle=");
    Serial.println(status.angle_degrees);
  }
  
  if (!status.communication_ok) {
    // BŁĄD: Brak komunikacji SPI
    led_no_communication();
    delay(3000);
    return;
  }
  
  // Test 2: Detekcja magnesu
  if (!status.magnet_detected) {
    // BŁĄD: No_Mag_Warning = 1 (brak magnesu)
    led_no_magnet();
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
    led_sensor_working();
    delay(500);
    
    // Bonus: Pokaż aktualny kąt przez wzorzec migania
    led_angle_data(status.angle_degrees);
    
  } else {
    // OSTRZEŻENIE: Enkoder działa ale brak ruchu
    led_constant_reading();
  }
  
  delay(2000); // Pauza między cyklami testowymi
}
