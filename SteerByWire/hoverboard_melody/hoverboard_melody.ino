// Simple buzzer for STM32 hoverboard with UART feedback and real ADC measurements

#define BUZZER_PIN PA4
#define LED_PIN PB2    // LED pin same as hoverboard main board

// ADC pin definitions (based on STM32F103 hoverboard)
#define BAT_VOLTAGE_PIN PA0    // Battery voltage measurement pin
#define TEMP_SENSOR_PIN PA1    // Temperature sensor pin (internal or external)

// UART communication defines
#define SERIAL_BAUD         115200      // [-] Baud rate for UART communication
#define START_FRAME         0xABCD      // [-] Start frame definition for reliable serial communication

// Battery calibration constants (from config.h)
#define BAT_CELLS               10        // battery number of cells. Normal Hoverboard battery: 10s
#define BAT_CALIB_REAL_VOLTAGE  3970      // input voltage measured by multimeter (multiplied by 100). In this case 39.70 V * 100 = 3970
#define BAT_CALIB_ADC           1492      // adc-value measured by mainboard
#define BAT_FILT_COEF           655       // battery voltage filter coefficient in fixed-point. 655 = 0.01 * 2^16

// Temperature calibration constants (from config.h) 
#define TEMP_CAL_LOW_ADC        1655      // temperature 1: ADC value
#define TEMP_CAL_LOW_DEG_C      358       // temperature 1: measured temperature [°C * 10]. Here 35.8 °C
#define TEMP_CAL_HIGH_ADC       1588      // temperature 2: ADC value
#define TEMP_CAL_HIGH_DEG_C     489       // temperature 2: measured temperature [°C * 10]. Here 48.9 °C
#define TEMP_FILT_COEF          655       // temperature filter coefficient in fixed-point. 655 = 0.01 * 2^16

// Timing variables
unsigned long lastBeepTime = 0;
const unsigned long beepInterval = 3000;  // 3 seconds
unsigned long lastFeedbackTime = 0;
const unsigned long feedbackInterval = 10;  // 10 ms (100 Hz feedback rate)
unsigned long lastAdcTime = 0;
const unsigned long adcInterval = 1;  // 1 ms for ADC readings

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

// Real sensor data variables
int16_t batVoltage = (400 * BAT_CELLS * BAT_CALIB_ADC) / BAT_CALIB_REAL_VOLTAGE;  // Initialize to 4V per cell
int32_t batVoltageFixdt = (400 * BAT_CELLS * BAT_CALIB_ADC) / BAT_CALIB_REAL_VOLTAGE << 16;  // Fixed-point filter
int16_t batVoltageCalib = 0;  // Calibrated battery voltage

int32_t board_temp_adcFixdt = 1600 << 16;  // Fixed-point filter initialized 
int16_t board_temp_adcFilt = 1600;         // Filtered ADC value
int16_t board_temp_deg_c = 25;             // Temperature in degrees Celsius * 10

// Mock sensor data variables (for testing when ADC not available)
int16_t mock_cmd1 = 0;
int16_t mock_cmd2 = 0;
int16_t mock_speedR = 0;
int16_t mock_speedL = 0;
uint16_t mock_cmdLed = 0;

void setup() {
  // Initialize Serial2 for UART communication on PA2 (RX) and PA3 (TX)
  Serial2.begin(SERIAL_BAUD);
  
  // Initialize ADC pins
  pinMode(BAT_VOLTAGE_PIN, INPUT_ANALOG);
  pinMode(TEMP_SENSOR_PIN, INPUT_ANALOG);
  
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
  lastAdcTime = millis();  // Initialize ADC timer
}

void loop() {
  unsigned long currentTime = millis();
  
  // Handle ADC readings
  if (currentTime - lastAdcTime >= adcInterval) {
    readAndFilterADC();
    lastAdcTime = currentTime;
  }
  
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
    //updateMockData();
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
  Feedback.batVoltage  = batVoltageCalib;  // Use real calibrated battery voltage
  Feedback.boardTemp   = board_temp_deg_c; // Use real measured temperature
  Feedback.cmdLed      = mock_cmdLed;
  
  // Calculate checksum (XOR of all fields except checksum itself)
  Feedback.checksum = (uint16_t)(Feedback.start ^ Feedback.cmd1 ^ Feedback.cmd2 ^ 
                                Feedback.speedR_meas ^ Feedback.speedL_meas ^ 
                                Feedback.batVoltage ^ Feedback.boardTemp ^ Feedback.cmdLed);
  
  // Send the feedback structure via Serial2 (USART2)
  Serial2.write((uint8_t*)&Feedback, sizeof(Feedback));
}

// Low-pass filter function (same as in util.c)
void filtLowPass32(int32_t u, uint16_t coef, int32_t *y) {
  int64_t tmp;  
  tmp = ((int64_t)((u << 4) - (*y >> 12)) * coef) >> 4;
  // Overflow protection: 2147483647LL = 2^31 - 1
  if (tmp > 2147483647LL) tmp = 2147483647LL;
  if (tmp < -2147483648LL) tmp = -2147483648LL;
  *y = (int32_t)tmp + (*y);
}

// Read and filter ADC values (similar to main.c implementation)
void readAndFilterADC() {
  static uint16_t adcCounter = 0;
  adcCounter++;
  
  // Filter battery voltage at a slower sampling rate (every 1000 calls, similar to main.c)
  if (adcCounter % 1000 == 0) {
    uint16_t batAdc = analogRead(BAT_VOLTAGE_PIN);
    filtLowPass32(batAdc, BAT_FILT_COEF, &batVoltageFixdt);
    batVoltage = (int16_t)(batVoltageFixdt >> 16);  // convert fixed-point to integer
    
    // Calculate calibrated battery voltage (same as main.c)
    batVoltageCalib = batVoltage * BAT_CALIB_REAL_VOLTAGE / BAT_CALIB_ADC;
  }
  
  // Read and filter temperature every call
  uint16_t tempAdc = analogRead(TEMP_SENSOR_PIN);
  filtLowPass32(tempAdc, TEMP_FILT_COEF, &board_temp_adcFixdt);
  board_temp_adcFilt = (int16_t)(board_temp_adcFixdt >> 16);  // convert fixed-point to integer
  
  // Calculate temperature in degrees Celsius (same formula as main.c)
  board_temp_deg_c = (TEMP_CAL_HIGH_DEG_C - TEMP_CAL_LOW_DEG_C) * (board_temp_adcFilt - TEMP_CAL_LOW_ADC) / 
                     (TEMP_CAL_HIGH_ADC - TEMP_CAL_LOW_ADC) + TEMP_CAL_LOW_DEG_C;
}

// Update mock sensor data to simulate changing values
void updateMockData() {
  static uint16_t counter = 0;
  counter++;
  
  // Simulate some varying sensor data
  mock_speedR = (int16_t)(50 * sin(counter * 0.1));  // Simulate right wheel speed
  mock_speedL = (int16_t)(50 * sin(counter * 0.1));  // Simulate left wheel speed
  
  // LED command simulation
  mock_cmdLed = (counter / 100) % 2;  // Toggle every second at 100Hz
}
