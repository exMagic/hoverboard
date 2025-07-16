#include <Wire.h>

// ---------- Definicje ----------

// AS5600
#define AS5600_I2C_ADDRESS   0x36
#define AS5600_ANGLE_REG     0x0E
#define AS5600_STATUS_REG    0x0B

// TCA9548A
#define TCA_ADDR             0x70  // Domyślny adres multipleksera

// Piny
#define BUZZER_PIN           PA4
#define I2C_SDA_PIN          PB11
#define I2C_SCL_PIN          PB10

// Utworzenie instancji I2C2 na PB11/PB10
TwoWire Wire2(I2C_SDA_PIN, I2C_SCL_PIN);

// ---------- Funkcje pomocnicze ----------

// Wybór kanału na TCA9548A (0–7)
void tcaSelect(uint8_t channel) {
  if (channel > 7) return;
  Wire2.beginTransmission(TCA_ADDR);
  Wire2.write(1 << channel);
  Wire2.endTransmission();
}

// Test połączenia z AS5600 na bieżącym kanale
bool testAS5600() {
  Wire2.beginTransmission(AS5600_I2C_ADDRESS);
  Wire2.write(AS5600_STATUS_REG);
  if (Wire2.endTransmission() != 0) return false;
  Wire2.requestFrom(AS5600_I2C_ADDRESS, 1);
  return Wire2.available() == 1;
}

// Odczyt kąta (12-bit)
uint16_t readAS5600Angle() {
  Wire2.beginTransmission(AS5600_I2C_ADDRESS);
  Wire2.write(AS5600_ANGLE_REG);
  Wire2.endTransmission();
  Wire2.requestFrom(AS5600_I2C_ADDRESS, 2);
  if (Wire2.available() < 2) return 0;
  return ((Wire2.read() << 8) | Wire2.read()) & 0x0FFF;
}

// ---------- Setup ----------

void setup() {
  // Buzzer
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  // Inicjalizacja I2C2
  Wire2.begin();
  Wire2.setClock(100000);

  // Sygnał startowy - prosty beep
  for (uint8_t i = 0; i < 3; i++) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(100);
    digitalWrite(BUZZER_PIN, LOW);
    delay(200);
  }
}

// ---------- Główna pętla ----------

void loop() {
  uint16_t angle0 = 0, angle7 = 0;
  bool ok0, ok7;

  // AS5600 #1 (kanał 0)
  tcaSelect(0);
  ok0 = testAS5600();
  if (ok0) angle0 = readAS5600Angle();

  // AS5600 #2 (kanał 7)
  tcaSelect(7);
  ok7 = testAS5600();
  if (ok7) angle7 = readAS5600Angle();

  // Generowanie tonu
  if (ok0 && ok7) {
    // Obie wartości poprawne: krótkie naprzemienne tony
    tone(BUZZER_PIN, map(angle0, 0, 4095, 200, 2000), 30);
    delay(3);
    tone(BUZZER_PIN, map(angle7, 0, 4095, 200, 2000), 30);
  } else if (ok0) {
    // Tylko enkoder 0
    tone(BUZZER_PIN, map(angle0, 0, 4095, 200, 2000));
  } else if (ok7) {
    // Tylko enkoder 7
    tone(BUZZER_PIN, map(angle7, 0, 4095, 200, 2000));
  } else {
    // Brak połączenia z oboma - sygnał błędu
    static uint32_t last = 0;
    static bool state = false;
    if (millis() - last > 500) {
      digitalWrite(BUZZER_PIN, state ? HIGH : LOW);
      state = !state;
      last = millis();
    }
  }
}
