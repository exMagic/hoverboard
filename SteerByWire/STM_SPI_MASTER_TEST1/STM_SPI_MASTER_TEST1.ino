#include <Arduino.h>

// Definicja pinów dla software SPI
constexpr uint8_t PIN_SCK   = PB7;   // Serial Clock
constexpr uint8_t PIN_MOSI  = PB5;   // Master Out Slave In
constexpr uint8_t PIN_MISO  = PB10;  // Master In Slave Out  
constexpr uint8_t PIN_CS    = PB6;   // Chip Select
constexpr uint8_t LED_PIN   = PB2;   // LED do sygnalizacji

// Zmienne globalne
uint8_t testData[] = {0xAA, 0x55, 0x12, 0x34, 0xFF, 0x00};
uint8_t dataIndex = 0;
unsigned long lastSendTime = 0;
const unsigned long SEND_INTERVAL = 1000; // Wysyłanie co 1 sekundę

void setup() {
  // Konfiguracja pinów - wyłącz wewnętrzne pull-up
  pinMode(PIN_SCK, OUTPUT);
  pinMode(PIN_MOSI, OUTPUT);
  pinMode(PIN_MISO, INPUT_PULLDOWN);  // Pull-down zamiast floating
  pinMode(PIN_CS, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  
  // Inicjalizacja stanów pinów
  digitalWrite(PIN_SCK, LOW);    // Clock idle low
  digitalWrite(PIN_MOSI, LOW);   // MOSI idle low
  digitalWrite(PIN_CS, HIGH);    // CS idle high (inactive)
  digitalWrite(LED_PIN, LOW);    // LED off
  
  delay(1000);
}

void loop() {
  // Sprawdź czy czas na wysłanie danych
  if (millis() - lastSendTime >= SEND_INTERVAL) {
    
    // Zapal LED podczas transmisji
    digitalWrite(LED_PIN, HIGH);
    
    // Wyślij dane przez SPI
    uint8_t dataToSend = testData[dataIndex];
    uint8_t receivedData = spiTransfer(dataToSend);
    
    // Przejdź do następnego elementu tablicy
    dataIndex = (dataIndex + 1) % (sizeof(testData) / sizeof(testData[0]));
    
    // Zaktualizuj czas ostatniej transmisji
    lastSendTime = millis();
    
    // Zgaś LED
    digitalWrite(LED_PIN, LOW);
    
    delay(100); // Krótka pauza między transmisją a kolejną iteracją
  }
}

// Funkcja do przesyłania jednego bajtu przez software SPI
uint8_t spiTransfer(uint8_t data) {
  uint8_t receivedData = 0;
  
  // Aktywuj CS (active low)
  digitalWrite(PIN_CS, LOW);
  delayMicroseconds(50); // Dłuższa pauza dla stabilizacji z pull-up
  
  // Przesyłaj bit po bicie (MSB first)
  for (int i = 7; i >= 0; i--) {
    // Ustaw bit danych na MOSI
    digitalWrite(PIN_MOSI, (data & (1 << i)) ? HIGH : LOW);
    delayMicroseconds(50); // Jeszcze wolniejszy timing
    
    // Wygeneruj zbocze narastające zegara
    digitalWrite(PIN_SCK, HIGH);
    delayMicroseconds(50); // Dłuższy czas HIGH
    
    // Odczytaj bit z MISO na zboczu narastającym
    if (digitalRead(PIN_MISO)) {
      receivedData |= (1 << i);
    }
    
    // Zbocze opadające zegara
    digitalWrite(PIN_SCK, LOW);
    delayMicroseconds(50); // Dłuższy czas LOW
  }
  
  // Dezaktywuj CS
  delayMicroseconds(50);
  digitalWrite(PIN_CS, HIGH);
  
  return receivedData;
}
