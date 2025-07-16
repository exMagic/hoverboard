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
  buzzStart();

  // init I2C + multiplekser
  Wire2.begin();
  Wire2.setClock(100000);

  // --- Silnik 1 (kanał 0) ---
  tcaSelect(0);
  sensor1.init(&Wire2);
  motor1.linkSensor(&sensor1);
  driver1.voltage_power_supply = 12;
  driver1.dead_zone           = 0.02f;       // 2% dead-time[16]
  driver1.pwm_frequency       = 20000;
  driver1.init();
  motor1.linkDriver(&driver1);
  motor1.controller = MotionControlType::torque;
  motor1.init();
  motor1.initFOC();

  // --- Silnik 2 (kanał 7) ---
  tcaSelect(7);
  sensor2.init(&Wire2);
  motor2.linkSensor(&sensor2);
  driver2.voltage_power_supply = 24;
  driver2.dead_zone           = 0.02f;
  driver2.pwm_frequency       = 20000;
  driver2.init();
  motor2.linkDriver(&driver2);
  motor2.controller = MotionControlType::torque;
  motor2.init();
  motor2.initFOC();

  // diagnostyka inicjalizacji
  tcaSelect(0); float a1 = sensor1.getAngle();
  tcaSelect(7); float a2 = sensor2.getAngle();
  if(!isnan(a1) && !isnan(a2)) buzzOK();
  else buzzErr();
}

void loop() {
  static unsigned long lastD=0;

  // FOC read & update
  tcaSelect(0); motor1.loopFOC();
  tcaSelect(7); motor2.loopFOC();

  // wirtualny link haptic
  float gain = 5.0;
  float ang1 = motor1.shaft_angle;
  float ang2 = motor2.shaft_angle;
  tcaSelect(0); motor2.move( gain*(ang2 - ang1) );
  tcaSelect(7); motor1.move( gain*(ang1 - ang2) );

  // diagnostyka co 2s: krótki tonalny potwierdzenie lub ostrzeżenie
  if(millis()-lastD>2000) {
    lastD = millis();
    tcaSelect(0); float d1 = sensor1.getAngle();
    tcaSelect(7); float d2 = sensor2.getAngle();
    if(!isnan(d1) && !isnan(d2)) tone(BUZZER_PIN,1200,100);
    else tone(BUZZER_PIN,300,200);
  }
}
