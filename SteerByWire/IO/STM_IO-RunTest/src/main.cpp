#include <Wire.h>
#include <SimpleFOC.h>

// I2C / multiplekser TCA9548A
#define I2C_SDA_PIN PB11
#define I2C_SCL_PIN PB10
#define TCA_ADDR    0x70
#define AS5600_ADDR 0x36

// buzzer diagnostyczny
#define BUZZER_PIN PA4

// PWM 6-kanałowy dla silnika 1 (kanał 0) left
#define PHA1_HI PA8
#define PHA1_LO PB13
#define PHB1_HI PA9
#define PHB1_LO PB14
#define PHC1_HI PA10
#define PHC1_LO PB15

// PWM 6-kanałowy dla silnika 2 (kanał 7) right
#define PHA2_HI PC6
#define PHA2_LO PA7
#define PHB2_HI PC7
#define PHB2_LO PB0
#define PHC2_HI PC8
#define PHC2_LO PB1

const int POLE_PAIRS = 15;  // par biegunów silnika hoverboard

// instancja I2C dla multipleksera
TwoWire Wire2(I2C_SDA_PIN, I2C_SCL_PIN);

// wybór kanału TCA9548A
void tcaSelect(uint8_t ch) {
  if(ch>7) return;
  Wire2.beginTransmission(TCA_ADDR);
  Wire2.write(1<<ch);
  Wire2.endTransmission();
}

// obiekty dla silnika 1
BLDCDriver6PWM driver1 = BLDCDriver6PWM(PHA1_HI, PHA1_LO, PHB1_HI, PHB1_LO, PHC1_HI, PHC1_LO);
BLDCMotor    motor1  = BLDCMotor(POLE_PAIRS);
// AS5600 ma rozdzielczość 12-bit, MSB = 0x0E, 4 bity w MSB
MagneticSensorI2C sensor1 = MagneticSensorI2C(AS5600_ADDR, 12, 0x0E, 4);

// obiekty dla silnika 2
BLDCDriver6PWM driver2 = BLDCDriver6PWM(PHA2_HI, PHA2_LO, PHB2_HI, PHB2_LO, PHC2_HI, PHC2_LO);
BLDCMotor    motor2  = BLDCMotor(POLE_PAIRS);
MagneticSensorI2C sensor2 = MagneticSensorI2C(AS5600_ADDR, 12, 0x0E, 4);

// funkjce diagnostyczne buzzera
void buzzStart() { for(int i=0;i<3;i++){ tone(BUZZER_PIN,1000,150); delay(200);} }
void buzzOK()    { tone(BUZZER_PIN,1500,500); delay(600); noTone(BUZZER_PIN); }
void buzzErr()   { for(int i=0;i<4;i++){ tone(BUZZER_PIN,400,100); delay(150);} delay(200); }

void setup() {
  // buzzer
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  
  // BEZPIECZNY START - wszystkie piny PWM na LOW i INPUT na początku
  
  // Ustawienie wszystkich pinów PWM jako INPUT (high impedance) - BEZPIECZNE
  pinMode(PHA1_HI, INPUT);
  pinMode(PHA1_LO, INPUT);
  pinMode(PHB1_HI, INPUT);
  pinMode(PHB1_LO, INPUT);
  pinMode(PHC1_HI, INPUT);
  pinMode(PHC1_LO, INPUT);
  
  pinMode(PHA2_HI, INPUT);
  pinMode(PHA2_LO, INPUT);
  pinMode(PHB2_HI, INPUT);
  pinMode(PHB2_LO, INPUT);
  pinMode(PHC2_HI, INPUT);
  pinMode(PHC2_LO, INPUT);
  
  // Sygnał że system startuje bezpiecznie
  tone(BUZZER_PIN, 500, 200);
  delay(300);
  tone(BUZZER_PIN, 800, 200);
  delay(300);
  tone(BUZZER_PIN, 1200, 500);
  delay(600);
  
  // Test I2C i sensorów
  Wire2.begin();
  Wire2.setClock(100000);
  
  tcaSelect(0);
  sensor1.init(&Wire2);
  delay(100);
  float a1 = sensor1.getAngle();
  
  tcaSelect(7);
  sensor2.init(&Wire2);
  delay(100);
  float a2 = sensor2.getAngle();
  
  // Raport sensorów
  if(!isnan(a1) && !isnan(a2)) {
    // Oba sensory OK
    tone(BUZZER_PIN, 1500, 200);
    delay(250);
    tone(BUZZER_PIN, 1500, 200);
    delay(250);
  } else {
    // Problem z sensorami - ZATRZYMAJ TEST
    tone(BUZZER_PIN, 300, 200);
    delay(250);
    tone(BUZZER_PIN, 300, 200);
    delay(250);
    tone(BUZZER_PIN, 300, 200);
    delay(2000);
    return; // NIE testuj silników jeśli sensory nie działają
  }
  
  // OSTROŻNY TEST SILNIKÓW - POZIOM 1: Pojedyncze impulsy
  delay(2000); // długa pauza przed testem
  
  tone(BUZZER_PIN, 600, 300); // sygnał rozpoczęcia testów silników
  delay(500);
  
  // Test 1: Bardzo krótkie impulsy - silnik 1
  tone(BUZZER_PIN, 800, 100);
  delay(200);
  
  // Ustawienie jednego pinu jako OUTPUT na bardzo krótko
  pinMode(PHA1_HI, OUTPUT);
  digitalWrite(PHA1_HI, LOW); // upewnij się że jest LOW
  delay(100);
  
  // Bardzo krótki impuls (1ms)
  digitalWrite(PHA1_HI, HIGH);
  delay(1);
  digitalWrite(PHA1_HI, LOW);
  delay(500);
  
  // Drugi pin
  pinMode(PHB1_HI, OUTPUT);
  digitalWrite(PHB1_HI, LOW);
  delay(100);
  
  digitalWrite(PHB1_HI, HIGH);
  delay(1);
  digitalWrite(PHB1_HI, LOW);
  delay(500);
  
  // Trzeci pin
  pinMode(PHC1_HI, OUTPUT);
  digitalWrite(PHC1_HI, LOW);
  delay(100);
  
  digitalWrite(PHC1_HI, HIGH);
  delay(1);
  digitalWrite(PHC1_HI, LOW);
  delay(500);
  
  // Powrót do INPUT (bezpieczne)
  pinMode(PHA1_HI, INPUT);
  pinMode(PHB1_HI, INPUT);
  pinMode(PHC1_HI, INPUT);
  
  delay(1000);
  
  // Test 2: Silnik 2 - te same krótkie impulsy
  tone(BUZZER_PIN, 1200, 100);
  delay(200);
  
  pinMode(PHA2_HI, OUTPUT);
  digitalWrite(PHA2_HI, LOW);
  delay(100);
  
  digitalWrite(PHA2_HI, HIGH);
  delay(1);
  digitalWrite(PHA2_HI, LOW);
  delay(500);
  
  pinMode(PHB2_HI, OUTPUT);
  digitalWrite(PHB2_HI, LOW);
  delay(100);
  
  digitalWrite(PHB2_HI, HIGH);
  delay(1);
  digitalWrite(PHB2_HI, LOW);
  delay(500);
  
  pinMode(PHC2_HI, OUTPUT);
  digitalWrite(PHC2_HI, LOW);
  delay(100);
  
  digitalWrite(PHC2_HI, HIGH);
  delay(1);
  digitalWrite(PHC2_HI, LOW);
  delay(500);
  
  // Powrót do INPUT (bezpieczne)
  pinMode(PHA2_HI, INPUT);
  pinMode(PHB2_HI, INPUT);
  pinMode(PHC2_HI, INPUT);
  
  // Test 3: Jeśli przeżyliśmy krótkie impulsy, wypróbuj dłuższe (10ms)
  delay(1000);
  tone(BUZZER_PIN, 900, 200);
  delay(300);
  
  // Silnik 1 - dłuższe impulsy
  pinMode(PHA1_HI, OUTPUT);
  digitalWrite(PHA1_HI, LOW);
  delay(100);
  
  digitalWrite(PHA1_HI, HIGH);
  delay(10); // 10ms
  digitalWrite(PHA1_HI, LOW);
  delay(1000);
  
  // Powrót do bezpiecznego stanu
  pinMode(PHA1_HI, INPUT);
  
  // Końcowy sygnał - test zakończony
  tone(BUZZER_PIN, 2000, 1000);
  delay(1100);
  
  // Dodatkowy sygnał sukcesu
  tone(BUZZER_PIN, 1500, 200);
  delay(250);
  tone(BUZZER_PIN, 1800, 200);
  delay(250);
  tone(BUZZER_PIN, 2000, 500);
  delay(600);
}

void loop() {
  // pusta - test zakończony w setup()
}
