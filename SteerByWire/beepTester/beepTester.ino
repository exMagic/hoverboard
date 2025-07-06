// === DEFINICJE PINÓW ===
#define CSN_PIN PB6    
#define SCK_PIN PB7    
#define MISO_PIN PB10   
#define BUZZER_PIN PA4  
#define LED_PIN PB2    

// === DEFINICJE CZĘSTOTLIWOŚCI BUZZER ===
#define BEEP_HIGH 2000    
#define BEEP_MID 1000     
#define BEEP_LOW 500      

// === DEFINICJE STRUKTUR - MUSZĄ BYĆ NA POCZĄTKU! ===
struct MT6816_Status {
  bool communication_ok;
  bool magnet_detected;
  bool angle_changing;
  uint16_t raw_angle;
  float angle_degrees;
  uint8_t reg03;
  uint8_t reg04;
};

// === FUNKCJE BUZZER ===
void beep(uint16_t frequency, uint16_t duration) {
  if (frequency == 0) {
    digitalWrite(BUZZER_PIN, LOW);
    delay(duration);
    return;
  }
  
  uint32_t period = 1000000 / frequency;
  uint32_t half_period = period / 2;
  uint32_t cycles = (duration * 1000) / period;
  
  for (uint32_t i = 0; i < cycles; i++) {
    digitalWrite(BUZZER_PIN, HIGH);
    delayMicroseconds(half_period);
    digitalWrite(BUZZER_PIN, LOW);
    delayMicroseconds(half_period);
  }
}

// Pozostałe funkcje beep...
void beep_success() {
  beep(BEEP_HIGH, 100);
}

void beep_warning() {
  beep(BEEP_MID, 200);
}

void beep_error() {
  beep(BEEP_LOW, 300);
}

void beep_pattern_no_communication() {
  for (int i = 0; i < 5; i++) {
    beep(BEEP_HIGH, 100);
    delay(100);
  }
}

void beep_pattern_no_magnet() {
  for (int i = 0; i < 3; i++) {
    beep(BEEP_LOW, 400);
    delay(200);
  }
}

void beep_pattern_working() {
  for (int i = 0; i < 2; i++) {
    beep(BEEP_MID, 200);
    delay(150);
  }
}

void beep_pattern_constant_reading() {
  beep(BEEP_MID, 500);
}

// === FUNKCJE SOFTWARE SPI ===
void spi_init() {
  pinMode(CSN_PIN, OUTPUT);
  pinMode(SCK_PIN, OUTPUT);  
  pinMode(MISO_PIN, INPUT);
  
  digitalWrite(CSN_PIN, HIGH);
  digitalWrite(SCK_PIN, HIGH);
  
  delay(10);
}

uint8_t spi_read_register(uint8_t reg_addr) {
  digitalWrite(CSN_PIN, LOW);
  delayMicroseconds(2);
  
  uint8_t command = 0x80 | (reg_addr << 1);
  
  for (int i = 7; i >= 0; i--) {
    digitalWrite(SCK_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(SCK_PIN, HIGH);
    delayMicroseconds(2);
  }
  
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
  
  digitalWrite(CSN_PIN, HIGH);
  delayMicroseconds(10);
  
  return data;
}

// === FUNKCJE DIAGNOSTYCZNE - TERAZ STRUKTURA JEST JUŻ ZDEFINIOWANA ===
MT6816_Status diagnose_mt6816() {
  MT6816_Status status = {false, false, false, 0, 0.0, 0, 0};
  
  status.reg03 = spi_read_register(0x03);
  delay(5);
  status.reg04 = spi_read_register(0x04);
  
  if ((status.reg03 != 0x00 && status.reg03 != 0xFF) || 
      (status.reg04 != 0x00 && status.reg04 != 0xFF)) {
    status.communication_ok = true;
  }
  
  status.magnet_detected = !(status.reg04 & 0x02);
  
  if (status.communication_ok) {
    status.raw_angle = ((uint16_t)status.reg03 << 8) | status.reg04;
    status.raw_angle &= 0x3FFF;
    status.angle_degrees = (status.raw_angle * 360.0) / 16384.0;
  }
  
  return status;
}

bool check_angle_changes() {
  static uint16_t previous_angle = 0;
  static bool first_reading = true;
  
  MT6816_Status status = diagnose_mt6816();
  
  if (!status.communication_ok) return false;
  
  if (first_reading) {
    previous_angle = status.raw_angle;
    first_reading = false;
    return false;
  }
  
  uint16_t diff = abs((int)status.raw_angle - (int)previous_angle);
  if (diff > 10) {
    previous_angle = status.raw_angle;
    return true;
  }
  
  return false;
}

// === GŁÓWNE FUNKCJE ===
void setup() {
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  
  digitalWrite(LED_PIN, HIGH);
  beep(1500, 500);
  delay(500);
  digitalWrite(LED_PIN, LOW);
  
  spi_init();
  
  delay(1000);
}

void loop() {
  static int test_cycle = 0;
  test_cycle++;
  
  digitalWrite(LED_PIN, test_cycle % 2);
  
  MT6816_Status status = diagnose_mt6816();
  
  if (!status.communication_ok) {
    beep_pattern_no_communication();
    delay(2000);
    return;
  }
  
  if (!status.magnet_detected) {
    beep_pattern_no_magnet();
    delay(2000);
    return;
  }
  
  bool angle_changed = false;
  for (int i = 0; i < 30; i++) {
    if (check_angle_changes()) {
      angle_changed = true;
      break;
    }
    delay(100);
  }
  
  if (angle_changed) {
    beep_pattern_working();
    
    float freq = 200 + (status.angle_degrees * 10);
    beep((uint16_t)freq, 200);
    
  } else {
    beep_pattern_constant_reading();
  }
  
  delay(3000);
}
