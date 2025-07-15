#include <Arduino.h>

// === DEFINICJE PINÓW ===
constexpr uint8_t PIN_SCK  = PB7;   // Serial Clock
constexpr uint8_t PIN_MOSI = PB5;   // Master Out Slave In
constexpr uint8_t PIN_MISO = PB10;  // Master In Slave Out
constexpr uint8_t PIN_CSN  = PB6;   // Chip Select (active low)

constexpr uint8_t LED_PIN     = PB2;  // LED do sygnalizacji
constexpr uint8_t BUZZER_PIN  = PA4;  // Buzzer do sygnalizacji audio

// === MT6816 KOMENDY ===
constexpr uint16_t MT6816_READ_ANGLE = 0x83FF;  // Komenda odczytu kąta
constexpr uint8_t MT6816_NOP = 0x00;            // No Operation

// === ZMIENNE GLOBALNE ===
uint16_t lastAngle = 0;
uint16_t angleBuffer[5] = {0};  // Bufor dla filtracji
uint8_t bufferIndex = 0;
bool communicationOK = false;
unsigned long lastReadTime = 0;
unsigned long lastDiagTime = 0;
const unsigned long READ_INTERVAL = 100;   // Odczyt co 100ms
const unsigned long DIAG_INTERVAL = 2000;  // Diagnostyka co 2s

// === SYGNALIZACJA ===
void signalSuccess() {
  // 3 krótkie błyśnięcia LED + 3 krótkie piki buzzera
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_PIN, HIGH);
    tone(BUZZER_PIN, 1000, 100);
    delay(100);
    digitalWrite(LED_PIN, LOW);
    delay(100);
  }
}

void signalError() {
  // Długie błyśnięcie LED + długi pik buzzera
  digitalWrite(LED_PIN, HIGH);
  tone(BUZZER_PIN, 500, 1000);
  delay(1000);
  digitalWrite(LED_PIN, LOW);
}

void signalHeartbeat() {
  // Pojedyncze krótkie błyśnięcie - komunikacja OK
  digitalWrite(LED_PIN, HIGH);
  delay(50);
  digitalWrite(LED_PIN, LOW);
}

// === FUNKCJE SPI ===
uint16_t spiTransfer16(uint16_t data) {
  uint16_t receivedData = 0;
  
  // Aktywuj CSN (active low)
  digitalWrite(PIN_CSN, LOW);
  delayMicroseconds(10); // Setup time
  
  // Przesyłaj 16 bitów (MSB first)
  for (int i = 15; i >= 0; i--) {
    // Ustaw bit danych na MOSI
    digitalWrite(PIN_MOSI, (data & (1 << i)) ? HIGH : LOW);
    delayMicroseconds(10);
    
    // Wygeneruj zbocze narastające zegara
    digitalWrite(PIN_SCK, HIGH);
    delayMicroseconds(10);
    
    // Odczytaj bit z MISO na zboczu narastającym
    if (digitalRead(PIN_MISO)) {
      receivedData |= (1 << i);
    }
    
    // Zbocze opadające zegara
    digitalWrite(PIN_SCK, LOW);
    delayMicroseconds(10);
  }
  
  // Dezaktywuj CSN
  delayMicroseconds(10);
  digitalWrite(PIN_CSN, HIGH);
  delayMicroseconds(50); // Hold time
  
  return receivedData;
}

// === FUNKCJE MT6816 ===
uint16_t readMT6816Angle() {
  // Wyślij komendę odczytu kąta
  uint16_t response = spiTransfer16(MT6816_READ_ANGLE);
  
  // MT6816 potrzebuje drugiej transakcji dla otrzymania danych
  delayMicroseconds(100);
  response = spiTransfer16(MT6816_NOP);
  
  // Sprawdź poprawność danych (MT6816 ma 14-bitową rozdzielczość)
  // Bity 15-14 powinny być 0, bity 13-0 to dane kąta
  if ((response & 0xC000) == 0) {
    return response & 0x3FFF; // Zwróć tylko 14 bitów danych
  }
  
  return 0xFFFF; // Błąd komunikacji
}

bool validateAngleData(uint16_t angle) {
  // Sprawdź czy kąt jest w prawidłowym zakresie (0-16383 dla 14-bit)
  if (angle == 0xFFFF || angle > 16383) {
    return false;
  }
  
  // Sprawdź stabilność - kąt nie powinien skakać o więcej niż 1000 w jednym odczycie
  // (chyba że przeszedł przez 0/16383)
  if (lastAngle != 0) {
    uint16_t diff = (angle > lastAngle) ? (angle - lastAngle) : (lastAngle - angle);
    if (diff > 1000 && diff < 15000) { // 15000 to próg dla przejścia przez 0
      return false;
    }
  }
  
  return true;
}

void updateAngleBuffer(uint16_t angle) {
  angleBuffer[bufferIndex] = angle;
  bufferIndex = (bufferIndex + 1) % 5;
}

uint16_t getFilteredAngle() {
  // Prosta filtracja - mediana z 5 próbek
  uint16_t sorted[5];
  for (int i = 0; i < 5; i++) {
    sorted[i] = angleBuffer[i];
  }
  
  // Sortowanie bąbelkowe
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4 - i; j++) {
      if (sorted[j] > sorted[j + 1]) {
        uint16_t temp = sorted[j];
        sorted[j] = sorted[j + 1];
        sorted[j + 1] = temp;
      }
    }
  }
  
  return sorted[2]; // Mediana
}

// === SETUP ===
void setup() {
  // Konfiguracja pinów SPI
  pinMode(PIN_SCK, OUTPUT);
  pinMode(PIN_MOSI, OUTPUT);
  pinMode(PIN_MISO, INPUT_PULLDOWN);  // Pull-down z powodu zewnętrznych pull-up
  pinMode(PIN_CSN, OUTPUT);
  
  // Konfiguracja pinów sygnalizacji
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  
  // Inicjalizacja stanów pinów SPI
  digitalWrite(PIN_SCK, LOW);    // Clock idle low
  digitalWrite(PIN_MOSI, LOW);   // MOSI idle low
  digitalWrite(PIN_CSN, HIGH);   // CSN idle high (inactive)
  
  // Inicjalizacja sygnalizacji
  digitalWrite(LED_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);
  
  // Sygnał startu - 2 długie piki
  for (int i = 0; i < 2; i++) {
    digitalWrite(LED_PIN, HIGH);
    tone(BUZZER_PIN, 800, 200);
    delay(300);
    digitalWrite(LED_PIN, LOW);
    delay(200);
  }
  
  delay(1000); // Stabilizacja MT6816
}

// === MAIN LOOP ===
void loop() {
  unsigned long currentTime = millis();
  
  // === ODCZYT KĄTA ===
  if (currentTime - lastReadTime >= READ_INTERVAL) {
    lastReadTime = currentTime;
    
    // Odczytaj kąt z MT6816
    uint16_t rawAngle = readMT6816Angle();
    
    if (validateAngleData(rawAngle)) {
      // Dane poprawne
      updateAngleBuffer(rawAngle);
      uint16_t filteredAngle = getFilteredAngle();
      lastAngle = filteredAngle;
      communicationOK = true;
      
      // Sygnalizuj poprawną komunikację (miganie co 10 odczytów)
      static uint8_t readCounter = 0;
      readCounter++;
      if (readCounter >= 10) {
        signalHeartbeat();
        readCounter = 0;
      }
      
    } else {
      // Błąd komunikacji
      communicationOK = false;
      
      // Sygnalizuj błąd co 5 nieudanych prób
      static uint8_t errorCounter = 0;
      errorCounter++;
      if (errorCounter >= 5) {
        signalError();
        errorCounter = 0;
      }
    }
  }
  
  // === DIAGNOSTYKA ===
  if (currentTime - lastDiagTime >= DIAG_INTERVAL) {
    lastDiagTime = currentTime;
    
    if (communicationOK) {
      signalSuccess(); // Komunikacja działa poprawnie
      
      // Dodatkowa sygnalizacja kąta przez buzzer
      // Wysokość tonu proporcjonalna do kąta
      uint16_t toneFreq = 200 + (lastAngle * 800 / 16383); // 200-1000 Hz
      tone(BUZZER_PIN, toneFreq, 100);
      
    } else {
      // Brak komunikacji - seria błędów
      for (int i = 0; i < 5; i++) {
        digitalWrite(LED_PIN, HIGH);
        tone(BUZZER_PIN, 200, 100);
        delay(100);
        digitalWrite(LED_PIN, LOW);
        delay(100);
      }
    }
  }
  
  delay(10); // Krótka pauza głównej pętli
}
