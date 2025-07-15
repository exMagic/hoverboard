#include <Arduino.h>
// Arduino UNO SPI Slave Test
// Współpracuje z STM32 jako SPI Master

// Piny SPI dla Arduino UNO (hardware SPI):
// Pin 10 - SS/CS (Slave Select)
// Pin 11 - MOSI (Master Out Slave In)  
// Pin 12 - MISO (Master In Slave Out)
// Pin 13 - SCK (Serial Clock)

volatile bool dataReady = false;
volatile uint8_t receivedByte = 0;
volatile uint8_t responseData = 0x42; // Domyślna odpowiedź do wysłania
uint8_t responseTable[] = {0x42, 0x84, 0xC6, 0xFF, 0x00, 0xA5}; // Tabela odpowiedzi
uint8_t responseIndex = 0;
unsigned long lastReceiveTime = 0;
uint16_t packetCounter = 0;

void setup() {
  // Inicjalizacja komunikacji szeregowej
  Serial.begin(115200);
  Serial.println("Arduino UNO SPI Slave Test - Starting...");
  
  // Konfiguracja pinów SPI
  pinMode(MISO, OUTPUT);      // Pin 12 - Master In Slave Out
  pinMode(SS, INPUT_PULLUP);  // Pin 10 - Slave Select z pull-up
  pinMode(MOSI, INPUT);       // Pin 11 - Master Out Slave In  
  pinMode(SCK, INPUT);        // Pin 13 - Serial Clock
  
  // Wbudowana dioda LED do sygnalizacji
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  
  // Inicjalizacja hardware SPI jako slave
  SPCR |= _BV(SPE);   // Włącz SPI
  SPCR |= _BV(SPIE);  // Włącz przerwanie SPI
  
  // Wstaw pierwszą odpowiedź do rejestru
  SPDR = responseTable[responseIndex];
  
  Serial.println("SPI Slave initialized:");
  Serial.println("SS: Pin 10, MOSI: Pin 11, MISO: Pin 12, SCK: Pin 13");
  Serial.println("Ready to receive data from STM32 Master...");
  Serial.println("Format: [Packet#] Received: 0x?? | Sent: 0x??");
  Serial.println("---");
}

// Przerwanie SPI - wykonuje się gdy master zakończy transmisję
ISR(SPI_STC_vect) {
  // Odczytaj odebrany bajt
  receivedByte = SPDR;
  
  // Przygotuj następną odpowiedź do rejestru
  responseIndex = (responseIndex + 1) % (sizeof(responseTable) / sizeof(responseTable[0]));
  SPDR = responseTable[responseIndex];
  
  // Ustaw flagę gotowości danych
  dataReady = true;
  
  // Migaj LED podczas odbioru
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
}

void loop() {
  // Sprawdź czy odebrano nowe dane
  if (dataReady) {
    // Zaktualizuj czas ostatniego odbioru
    lastReceiveTime = millis();
    packetCounter++;
    
    // Wyświetl informacje o odebranym pakiecie
    Serial.print("[");
    Serial.print(packetCounter);
    Serial.print("] Received: 0x");
    if (receivedByte < 0x10) Serial.print("0");
    Serial.print(receivedByte, HEX);
    Serial.print(" | Sent: 0x");
    if (responseTable[(responseIndex == 0) ? (sizeof(responseTable)/sizeof(responseTable[0])-1) : responseIndex-1] < 0x10) Serial.print("0");
    Serial.print(responseTable[(responseIndex == 0) ? (sizeof(responseTable)/sizeof(responseTable[0])-1) : responseIndex-1], HEX);
    Serial.print(" | SS: ");
    Serial.println(digitalRead(SS) ? "HIGH" : "LOW");
    
    // Wyczyść flagę
    dataReady = false;
  }
  
  // Sprawdź timeout (brak komunikacji przez 5 sekund)
  if (millis() - lastReceiveTime > 5000 && lastReceiveTime > 0) {
    Serial.println("Communication timeout - no data from master for 5 seconds");
    digitalWrite(LED_BUILTIN, LOW); // Zgaś LED przy timeout
    lastReceiveTime = millis(); // Reset timeout
  }
  
  // Sprawdź stan SS pin co 2 sekundy (diagnostyka)
  static unsigned long lastDiagTime = 0;
  if (millis() - lastDiagTime > 2000) {
    lastDiagTime = millis();
    
    // Tylko jeśli nie było ostatniej komunikacji przez więcej niż 3 sekundy
    if (millis() - lastReceiveTime > 3000 || lastReceiveTime == 0) {
      Serial.print("Diagnostic - SS pin: ");
      Serial.print(digitalRead(SS) ? "HIGH" : "LOW");
      Serial.print(" | Packets received: ");
      Serial.println(packetCounter);
    }
  }
}
