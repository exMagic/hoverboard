#include <Wire.h>

// Podstawowe stałe matematyczne
#define _PI 3.14159265359f
#define _2PI 6.28318530718f
#define _SQRT3 1.73205080757f

// I2C / multiplekser TCA9548A
#define I2C_SDA_PIN PB11
#define I2C_SCL_PIN PB10
#define TCA_ADDR    0x70
#define AS5600_ADDR 0x36

// buzzer diagnostyczny
#define BUZZER_PIN PA4

// PWM 6-kanałowy dla silnika 1 (kanał 0)
#define PHA1_HI PA8
#define PHA1_LO PA7
#define PHB1_HI PA9
#define PHB1_LO PB0
#define PHC1_HI PA10
#define PHC1_LO PB1

// PWM 6-kanałowy dla silnika 2 (kanał 7)
#define PHA2_HI PB6
#define PHA2_LO PB7
#define PHB2_HI PB8
#define PHB2_LO PB9
#define PHC2_HI PC6
#define PHC2_LO PC7

const int POLE_PAIRS = 11;

// Minimalna struktura sensora AS5600
struct SimpleSensor {
  uint8_t address;
  TwoWire* wire;
  float offset;
  
  void init(TwoWire* w, uint8_t addr) {
    wire = w;
    address = addr;
    offset = 0;
  }
  
  float readAngle() {
    wire->beginTransmission(address);
    wire->write(0x0E); // Raw angle register
    if (wire->endTransmission() != 0) return NAN;
    
    wire->requestFrom(address, (uint8_t)2);
    if (wire->available() < 2) return NAN;
    
    uint16_t raw = (wire->read() << 8) | wire->read();
    return ((float)(raw & 0x0FFF) / 4096.0f) * _2PI - offset;
  }
};

// Minimalna struktura drivera 6PWM
struct SimpleDriver {
  int pin_ah, pin_al, pin_bh, pin_bl, pin_ch, pin_cl;
  float supply_voltage;
  
  void init(int ah, int al, int bh, int bl, int ch, int cl, float voltage) {
    pin_ah = ah; pin_al = al;
    pin_bh = bh; pin_bl = bl; 
    pin_ch = ch; pin_cl = cl;
    supply_voltage = voltage;
    
    pinMode(pin_ah, OUTPUT); pinMode(pin_al, OUTPUT);
    pinMode(pin_bh, OUTPUT); pinMode(pin_bl, OUTPUT);
    pinMode(pin_ch, OUTPUT); pinMode(pin_cl, OUTPUT);
    
    // Ustaw częstotliwość PWM na 20kHz
    analogWriteFrequency(20000);
  }
  
  void setPWM(float ua, float ub, float uc) {
    // Ograniczenie napięć
    ua = constrain(ua, -supply_voltage/2, supply_voltage/2);
    ub = constrain(ub, -supply_voltage/2, supply_voltage/2);
    uc = constrain(uc, -supply_voltage/2, supply_voltage/2);
    
    // Konwersja na duty cycle (0-255)
    int duty_a = (int)((ua / supply_voltage + 0.5f) * 255);
    int duty_b = (int)((ub / supply_voltage + 0.5f) * 255);
    int duty_c = (int)((uc / supply_voltage + 0.5f) * 255);
    
    duty_a = constrain(duty_a, 0, 255);
    duty_b = constrain(duty_b, 0, 255);
    duty_c = constrain(duty_c, 0, 255);
    
    // Sterowanie 6PWM z dead-time
    analogWrite(pin_ah, duty_a);
    analogWrite(pin_al, 255 - duty_a);
    analogWrite(pin_bh, duty_b);
    analogWrite(pin_bl, 255 - duty_b);
    analogWrite(pin_ch, duty_c);
    analogWrite(pin_cl, 255 - duty_c);
  }
};

// Minimalna struktura motora BLDC
struct SimpleMotor {
  SimpleSensor* sensor;
  SimpleDriver* driver;
  float zero_electric_angle;
  float shaft_angle;
  float target_voltage;
  int pole_pairs;
  
  void init(SimpleSensor* s, SimpleDriver* d, int pp) {
    sensor = s;
    driver = d;
    pole_pairs = pp;
    zero_electric_angle = 0;
    shaft_angle = 0;
    target_voltage = 0;
  }
  
  void calibrate() {
    // Proste znalezienie electrical zero
    driver->setPWM(3.0f, 0, 0);
    delay(500);
    zero_electric_angle = sensor->readAngle();
    driver->setPWM(0, 0, 0);
    delay(100);
  }
  
  void updateFOC() {
    float angle = sensor->readAngle();
    if (isnan(angle)) return;
    
    shaft_angle = angle;
    float electrical_angle = (angle - zero_electric_angle) * pole_pairs;
    
    // Clarke + Park inverse transform (uproszczone FOC)
    float cos_ea = cos(electrical_angle);
    float sin_ea = sin(electrical_angle);
    
    float Uq = target_voltage;  // q-axis voltage (torque)
    float Ud = 0;               // d-axis voltage (flux)
    
    // Inverse Park transform
    float Ualpha = cos_ea * Ud - sin_ea * Uq;
    float Ubeta = sin_ea * Ud + cos_ea * Uq;
    
    // Inverse Clarke transform (3-phase)
    float Ua = Ualpha;
    float Ub = -0.5f * Ualpha + _SQRT3 * 0.5f * Ubeta;
    float Uc = -0.5f * Ualpha - _SQRT3 * 0.5f * Ubeta;
    
    driver->setPWM(Ua, Ub, Uc);
  }
  
  void setTorque(float voltage) {
    target_voltage = constrain(voltage, -6.0f, 6.0f);
  }
};

// Globalne obiekty
TwoWire Wire2(I2C_SDA_PIN, I2C_SCL_PIN);

SimpleSensor sensor1, sensor2;
SimpleDriver driver1, driver2;
SimpleMotor motor1, motor2;

// TCA9548A multiplexer control
void selectI2C(uint8_t channel) {
  if (channel > 7) return;
  Wire2.beginTransmission(TCA_ADDR);
  Wire2.write(1 << channel);
  Wire2.endTransmission();
}

// Buzzer functions
void buzzStart() { 
  for(int i=0; i<3; i++) { 
    tone(BUZZER_PIN, 3000, 150); 
    delay(200); 
  } 
}

void buzzOK() { 
  tone(BUZZER_PIN, 1500, 500); 
  delay(600); 
  noTone(BUZZER_PIN); 
}

void buzzError() { 
  for(int i=0; i<4; i++) { 
    tone(BUZZER_PIN, 400, 100); 
    delay(150); 
  } 
  delay(200); 
}

void setup() {
  // Buzzer init
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  buzzStart();
  
  // I2C init
  Wire2.begin();
  Wire2.setClock(100000);
  
  // Motor 1 setup (channel 0)
  selectI2C(0);
  sensor1.init(&Wire2, AS5600_ADDR);
  driver1.init(PHA1_HI, PHA1_LO, PHB1_HI, PHB1_LO, PHC1_HI, PHC1_LO, 24.0f);
  motor1.init(&sensor1, &driver1, POLE_PAIRS);
  motor1.calibrate();
  
  // Motor 2 setup (channel 7)
  selectI2C(7);
  sensor2.init(&Wire2, AS5600_ADDR);
  driver2.init(PHA2_HI, PHA2_LO, PHB2_HI, PHB2_LO, PHC2_HI, PHC2_LO, 24.0f);
  motor2.init(&sensor2, &driver2, POLE_PAIRS);
  motor2.calibrate();
  
  // Test sensors
  selectI2C(0); float a1 = sensor1.readAngle();
  selectI2C(7); float a2 = sensor2.readAngle();
  
  if (!isnan(a1) && !isnan(a2)) {
    buzzOK();
  } else {
    buzzError();
  }
}

void loop() {
  static unsigned long lastDiag = 0;
  
  // Update FOC for both motors
  selectI2C(0); motor1.updateFOC();
  selectI2C(7); motor2.updateFOC();
  
  // Haptic feedback coupling
  float haptic_gain = 2.0f;
  float angle_diff = motor2.shaft_angle - motor1.shaft_angle;
  
  motor1.setTorque(haptic_gain * angle_diff);
  motor2.setTorque(-haptic_gain * angle_diff);
  
  // Diagnostics every 2 seconds
  if (millis() - lastDiag > 2000) {
    lastDiag = millis();
    
    selectI2C(0); float d1 = sensor1.readAngle();
    selectI2C(7); float d2 = sensor2.readAngle();
    
    if (!isnan(d1) && !isnan(d2)) {
      tone(BUZZER_PIN, 800, 100);
    } else {
      tone(BUZZER_PIN, 300, 200);
    }
  }
}
