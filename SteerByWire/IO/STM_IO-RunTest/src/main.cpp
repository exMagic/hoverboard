#include <Wire.h>
#include <SimpleFOC.h>

// I2C / multiplekser TCA9548A
#define I2C_SDA_PIN PB11
#define I2C_SCL_PIN PB10
#define TCA_ADDR    0x70
#define AS5600_ADDR 0x36

// buzzer diagnostyczny
#define BUZZER_PIN PA4

// PWM 6-kanałowy dla silnika 1 (kanał 0) RIGHT = PA8 pins (POPRAWIONE!)
#define PHA1_HI PA8   
#define PHA1_LO PB13
#define PHB1_HI PA9
#define PHB1_LO PB14
#define PHC1_HI PA10
#define PHC1_LO PB15

// PWM 6-kanałowy dla silnika 2 (kanał 7) LEFT = PC6 pins (POPRAWIONE!)
#define PHA2_HI PC6   
#define PHA2_LO PA7
#define PHB2_HI PC7
#define PHB2_LO PB0
#define PHC2_HI PC8
#define PHC2_LO PB1

const int POLE_PAIRS = 15;  // par biegunów silnika hoverboard

// PARAMETRY HOVERBOARD (na podstawie analizy main.c)
const float BATTERY_VOLTAGE = 36.0;    // Rzeczywiste napięcie baterii hoverboard
const float VOLTAGE_LIMIT = 12.0;      // Bezpieczny limit (1/3 zasilania)
const float CURRENT_LIMIT = 10.0;      // Bezpieczny limit prądu [A]
const float VELOCITY_LIMIT = 20.0;     // Limit prędkości [rad/s]

// Globalne flagi bezpieczeństwa (jak w oryginalnym firmware)
volatile bool enable_motors = false;
volatile bool sensors_ok = false;
volatile uint32_t main_loop_counter = 0;

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
void buzzMotorsEnabled() { tone(BUZZER_PIN,1000,100); delay(150); tone(BUZZER_PIN,1500,100); delay(150); }

void setup() {
  // buzzer
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  
  // Sygnał startu systemu - OPEN LOOP MODE
  buzzStart();
  delay(1000);
  
  // ####### POMIŃ SENSORY - OPEN LOOP TEST #######
  // Dla open loop nie potrzebujemy sensorów!
  sensors_ok = true; // Wymuszamy OK żeby przejść dalej
  
  tone(BUZZER_PIN, 1200, 300); // Sygnał: OPEN LOOP MODE
  delay(400);
  tone(BUZZER_PIN, 800, 300);  // Sygnał: BEZ SENSORÓW
  delay(400);
  
  // ####### KONFIGURACJA STEROWNIKÓW (poprawne parametry) #######
  
  // Driver 1 - POPRAWNE NAPIĘCIA!
  // driver1.voltage_power_supply = BATTERY_VOLTAGE;  // 36V
  // driver1.init();
  
  // Driver 2 - POPRAWNE NAPIĘCIA!
  driver2.voltage_power_supply = BATTERY_VOLTAGE;  // 36V
  driver2.init();
  
  // ####### KONFIGURACJA SILNIKÓW - OPEN LOOP #######
  
  // Motor 1 - WYŁĄCZONY DLA TESTU! (PRAWY SILNIK)
  // motor1.linkDriver(&driver1);
  // motor1.voltage_limit = 3.0;   
  // motor1.velocity_limit = 5.0;  
  // motor1.controller = MotionControlType::velocity_openloop;
  // motor1.init();
  
  // Motor 2 - TYLKO ON AKTYWNY! (LEWY SILNIK - test)
  motor2.linkDriver(&driver2);
  motor2.voltage_limit = 3.0;   
  motor2.velocity_limit = 5.0;
  motor2.controller = MotionControlType::velocity_openloop;
  motor2.init();
  
  delay(1000);
  
  // ####### BEZ initFOC() - to potrzebuje sensorów! #######
  // motor1.initFOC(); - POMINIĘTE
  // motor2.initFOC(); - POMINIĘTE
  
  // Sygnał gotowości systemu
  buzzOK();
  
  // ####### BEZPIECZNE WŁĄCZANIE SILNIKÓW #######
  delay(2000); // Pauza przed włączeniem
  
  enable_motors = true;
  buzzMotorsEnabled(); // Sygnał włączenia silników
  delay(500);
}

void loop() {
  main_loop_counter++;
  
  // OPEN LOOP - nie sprawdzamy sensorów
  if(!enable_motors) {
    delay(100);
    return;
  }
  
  // ####### OPEN LOOP - BEZ AKTUALIZACJI SENSORÓW #######
  // NIE UŻYWAMY: motor1.loopFOC(); - to potrzebuje sensorów
  // NIE UŻYWAMY: tcaSelect() - nie potrzebujemy sensorów
  
  // ####### OPEN LOOP TEST RUCHU SILNIKÓW #######
  // Co 3 sekundy (3000 ms) wykonaj bardzo delikatny test
  
  static unsigned long lastTestTime = 0;
  static int testPhase = 0;
  
  if(millis() - lastTestTime > 3000) { // Częściej niż 5s
    lastTestTime = millis();
    
    switch(testPhase) {
      case 0:
        // Test 1: TYLKO MOTOR2 (PC6 pins) - powinien być LEWY
        tone(BUZZER_PIN, 1200, 200); // Wysoki sygnał dla motor2
        delay(300);
        
        motor2.move(1.0); // TYLKO motor2!
        delay(500);       
        motor2.move(0);   
        
        break;
        
      case 1:
        // Test 2: PONOWNIE MOTOR2 - potwierdzenie
        tone(BUZZER_PIN, 1300, 200);
        delay(300);
        
        motor2.move(-1.0); // Przeciwny kierunek
        delay(500);       
        motor2.move(0);   
        
        break;
        
      case 2:
        // Test 3: JESZCZE RAZ MOTOR2 - maksymalne potwierdzenie
        tone(BUZZER_PIN, 1400, 200);
        delay(300);
        
        motor2.move(0.5); // Wolniej
        delay(700);       // Jeszcze dłużej
        motor2.move(0);
        
        break;
        
      case 3:
        // Test 4: Pauza - motor1 jest wyłączony
        tone(BUZZER_PIN, 500, 500); // Długi ton = koniec testów
        delay(1000);
        
        break;
        
      case 4:
        // Test 5: Reset - długa pauza
        tone(BUZZER_PIN, 200, 1000); // Bardzo długi niski ton
        delay(3000); // Bardzo długa pauza
        testPhase = -1; 
        break;
    }
    
    testPhase++;
  }
  
  // OPEN LOOP - szybsza pętla
  delay(5); // 200Hz loop
}
