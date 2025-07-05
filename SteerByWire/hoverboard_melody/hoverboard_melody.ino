// Simple buzzer for STM32 hoverboard with UART feedback

#define BUZZER_PIN PA4
#define LED_PIN PB2    // LED pin same as hoverboard main board

// UART communication defines
#define SERIAL_BAUD         115200      // [-] Baud rate for UART communication
#define START_FRAME         0xABCD      // [-] Start frame definition for reliable serial communication

// Timing variables
unsigned long lastBeepTime = 0;
const unsigned long beepInterval = 3000;  // 3 seconds
unsigned long lastFeedbackTime = 0;
const unsigned long feedbackInterval = 10;  // 10 ms (100 Hz feedback rate)

// Serial feedback structure (same as in main.c)
typedef struct{
  uint16_t  start;
  int16_t   cmd1;
  int16_t   cmd2;
  int16_t   speedR_meas;
  int16_t   speedL_meas;
  int16_t   batVoltage;
  int16_t   boardTemp;
  uint16_t  cmdLed;
  uint16_t  checksum;
} SerialFeedback;

SerialFeedback Feedback;

// Mock sensor data variables (simulating hoverboard sensor readings)
int16_t mock_cmd1 = 0;
int16_t mock_cmd2 = 0;
int16_t mock_speedR = 0;
int16_t mock_speedL = 0;
int16_t mock_batVoltage = 4200;  // Mock 42V battery
int16_t mock_boardTemp = 25;     // Mock 25°C temperature
uint16_t mock_cmdLed = 0;

void setup() {
  // Initialize Serial2 for UART communication on PA2 (RX) and PA3 (TX)
  Serial2.begin(SERIAL_BAUD);
  
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);  // Turn off LED initially
  delay(500);
  
  // 3 beeps on startup
  for (int i = 0; i < 3; i++) {
    beepWithLed();
    delay(200);
  }
  
  lastBeepTime = millis();  // Initialize timer
  lastFeedbackTime = millis();  // Initialize feedback timer
}

void loop() {
  unsigned long currentTime = millis();
  
  // Handle beep timing
  if (currentTime - lastBeepTime >= beepInterval) {
    beepWithLed();
    lastBeepTime = currentTime;
  }
  
  // Handle UART feedback transmission
  if (currentTime - lastFeedbackTime >= feedbackInterval) {
    sendFeedback();
    lastFeedbackTime = currentTime;
    
    // Update mock sensor data (simulate some changing values)
    updateMockData();
  }
}

void beep() {
  for (int i = 0; i < 200; i++) {
    digitalWrite(BUZZER_PIN, HIGH);
    delayMicroseconds(150);
    digitalWrite(BUZZER_PIN, LOW);
    delayMicroseconds(150);
  }
}

void ledOn() {
  digitalWrite(LED_PIN, HIGH);
}

void ledOff() {
  digitalWrite(LED_PIN, LOW);
}

void beepWithLed() {
  ledOn();
  beep();
  ledOff();
}

// Send feedback data via UART (similar to main.c implementation)
void sendFeedback() {
  // Populate feedback structure
  Feedback.start       = (uint16_t)START_FRAME;
  Feedback.cmd1        = mock_cmd1;
  Feedback.cmd2        = mock_cmd2;
  Feedback.speedR_meas = mock_speedR;
  Feedback.speedL_meas = mock_speedL;
  Feedback.batVoltage  = mock_batVoltage;
  Feedback.boardTemp   = mock_boardTemp;
  Feedback.cmdLed      = mock_cmdLed;
  
  // Calculate checksum (XOR of all fields except checksum itself)
  Feedback.checksum = (uint16_t)(Feedback.start ^ Feedback.cmd1 ^ Feedback.cmd2 ^ 
                                Feedback.speedR_meas ^ Feedback.speedL_meas ^ 
                                Feedback.batVoltage ^ Feedback.boardTemp ^ Feedback.cmdLed);
  
  // Send the feedback structure via Serial2 (USART2)
  Serial2.write((uint8_t*)&Feedback, sizeof(Feedback));
}

// Update mock sensor data to simulate changing values
void updateMockData() {
  static uint16_t counter = 0;
  counter++;
  
  // Simulate some varying sensor data
  mock_speedR = (int16_t)(50 * sin(counter * 0.1));  // Simulate right wheel speed
  mock_speedL = (int16_t)(50 * sin(counter * 0.1));  // Simulate left wheel speed
  
  // Battery voltage slowly decreasing simulation
  if (counter % 1000 == 0 && mock_batVoltage > 3000) {
    mock_batVoltage -= 1;
  }
  
  // Temperature variation
  mock_boardTemp = 25 + (int16_t)(5 * sin(counter * 0.05));
  
  // LED command simulation
  mock_cmdLed = (counter / 100) % 2;  // Toggle every second at 100Hz
}
