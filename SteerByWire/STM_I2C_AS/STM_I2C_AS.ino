#include <Wire.h>

// Konfiguracja AS5600
#define AS5600_I2C_ADDRESS 0x36  // Adres I2C AS5600
#define AS5600_ANGLE_REG 0x0E    // Rejestr kąta (MSB)
#define AS5600_STATUS_REG 0x0B   // Rejestr statusu
#define AS5600_MAGNITUDE_REG 0x1B // Rejestr magnetu

// Piny
#define BUZZER_PIN PA4
#define I2C_SDA_PIN PB11
#define I2C_SCL_PIN PB10

// Utworzenie instancji I2C2 dla pinów PB10/PB11
TwoWire Wire2(I2C_SDA_PIN, I2C_SCL_PIN);

// Zmienne globalne
bool as5600_connected = false;
uint16_t angle_value = 0;
uint8_t status_value = 0;

void setup() {
  // Konfiguracja buzzera
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  
  // Sygnał startowy - 3 krótkie sygnały
  for(int i = 0; i < 3; i++) {
    tone(BUZZER_PIN, 1000, 200);
    delay(300);
  }
  
  // Inicjalizacja I2C na pinach PB10/PB11
  Wire2.begin();
  Wire2.setClock(100000); // 100kHz
  
  delay(100);
  
  // Test połączenia z AS5600
  if(testAS5600Connection()) {
    as5600_connected = true;
    // Sygnał sukcesu - długi sygnał
    tone(BUZZER_PIN, 1500, 1000);
    delay(1200);
  } else {
    // Sygnał błędu - 5 krótkich sygnałów
    for(int i = 0; i < 5; i++) {
      tone(BUZZER_PIN, 500, 100);
      delay(200);
    }
    delay(1000);
  }
}

void loop() {
  if(as5600_connected) {
    // Odczyt wartości z AS5600
    angle_value = readAS5600Angle();
    status_value = readAS5600Status();
    
    // Sprawdzenie czy magnes jest prawidłowo umieszczony
    if(status_value & 0x20) { // Magnet detected
      // Mapowanie wartości kąta (0-4095) na częstotliwość (200-2000Hz)
      int frequency = map(angle_value, 0, 4095, 200, 2000);
      
      // Ciągłe generowanie tonu o zmiennej częstotliwości
      tone(BUZZER_PIN, frequency);
      
    } else {
      // Brak magnetu - generuj sygnał ostrzegawczy (szybkie pulsowanie)
      static unsigned long lastPulse = 0;
      static bool pulseState = false;
      
      if(millis() - lastPulse > 100) { // Pulsowanie co 100ms
        if(pulseState) {
          tone(BUZZER_PIN, 300);
        } else {
          noTone(BUZZER_PIN);
        }
        pulseState = !pulseState;
        lastPulse = millis();
      }
    }
    
  } else {
    // Brak połączenia z AS5600 - sygnał błędu (bardzo powolne pulsowanie)
    static unsigned long lastErrorPulse = 0;
    static bool errorState = false;
    
    if(millis() - lastErrorPulse > 500) { // Pulsowanie co 500ms
      if(errorState) {
        tone(BUZZER_PIN, 200);
      } else {
        noTone(BUZZER_PIN);
      }
      errorState = !errorState;
      lastErrorPulse = millis();
    }
    
    // Próba ponownego połączenia co 2 sekundy
    static unsigned long lastReconnectAttempt = 0;
    if(millis() - lastReconnectAttempt > 2000) {
      if(testAS5600Connection()) {
        as5600_connected = true;
        // Krótki sygnał potwierdzenia reconnect
        tone(BUZZER_PIN, 1200);
        delay(300);
        noTone(BUZZER_PIN);
      }
      lastReconnectAttempt = millis();
    }
  }
  
  // Krótka pauza dla stabilności
  delay(10);
}


// Funkcja testująca połączenie z AS5600
bool testAS5600Connection() {
  Wire2.beginTransmission(AS5600_I2C_ADDRESS);
  Wire2.write(AS5600_STATUS_REG);
  
  if(Wire2.endTransmission() == 0) {
    Wire2.requestFrom(AS5600_I2C_ADDRESS, 1);
    if(Wire2.available()) {
      Wire2.read(); // Odczyt statusu
      return true;
    }
  }
  return false;
}

// Funkcja odczytu kąta z AS5600
uint16_t readAS5600Angle() {
  uint16_t angle = 0;
  
  Wire2.beginTransmission(AS5600_I2C_ADDRESS);
  Wire2.write(AS5600_ANGLE_REG);
  
  if(Wire2.endTransmission() == 0) {
    Wire2.requestFrom(AS5600_I2C_ADDRESS, 2);
    
    if(Wire2.available() >= 2) {
      uint8_t msb = Wire2.read();
      uint8_t lsb = Wire2.read();
      angle = ((msb << 8) | lsb) & 0x0FFF; // 12-bit wartość
    }
  }
  
  return angle;
}

// Funkcja odczytu statusu z AS5600
uint8_t readAS5600Status() {
  uint8_t status = 0;
  
  Wire2.beginTransmission(AS5600_I2C_ADDRESS);
  Wire2.write(AS5600_STATUS_REG);
  
  if(Wire2.endTransmission() == 0) {
    Wire2.requestFrom(AS5600_I2C_ADDRESS, 1);
    
    if(Wire2.available()) {
      status = Wire2.read();
    }
  }
  
  return status;
}

// Funkcja odczytu surowego kąta (RAW ANGLE)
uint16_t readAS5600RawAngle() {
  uint16_t rawAngle = 0;
  
  Wire2.beginTransmission(AS5600_I2C_ADDRESS);
  Wire2.write(0x0C); // RAW ANGLE register
  
  if(Wire2.endTransmission() == 0) {
    Wire2.requestFrom(AS5600_I2C_ADDRESS, 2);
    
    if(Wire2.available() >= 2) {
      uint8_t msb = Wire2.read();
      uint8_t lsb = Wire2.read();
      rawAngle = ((msb << 8) | lsb) & 0x0FFF;
    }
  }
  
  return rawAngle;
}

// Funkcja odczytu siły pola magnetycznego
uint16_t readAS5600Magnitude() {
  uint16_t magnitude = 0;
  
  Wire2.beginTransmission(AS5600_I2C_ADDRESS);
  Wire2.write(AS5600_MAGNITUDE_REG);
  
  if(Wire2.endTransmission() == 0) {
    Wire2.requestFrom(AS5600_I2C_ADDRESS, 2);
    
    if(Wire2.available() >= 2) {
      uint8_t msb = Wire2.read();
      uint8_t lsb = Wire2.read();
      magnitude = ((msb << 8) | lsb) & 0x0FFF;
    }
  }
  
  return magnitude;
}

// Funkcja sprawdzająca czy magnes jest prawidłowo umieszczony
bool isMagnetDetected() {
  uint8_t status = readAS5600Status();
  return (status & 0x20) != 0; // Bit 5 - magnet detected
}

// Funkcja sprawdzająca czy magnes jest za słaby
bool isMagnetTooWeak() {
  uint8_t status = readAS5600Status();
  return (status & 0x10) != 0; // Bit 4 - magnet too weak
}

// Funkcja sprawdzająca czy magnes jest za silny
bool isMagnetTooStrong() {
  uint8_t status = readAS5600Status();
  return (status & 0x08) != 0; // Bit 3 - magnet too strong
}
