// Simple buzzer for STM32 hoverboard with UART feedback and real ADC measurements
#include <SimpleFOC.h>

// Definicja pinów dla enkoderów - TRY DIFFERENT CONFIGURATIONS:
// SPI Pin combinations for PB5, PB6, PB7 (all 6 permutations):

// Configuration 1: CSN=PB5, SCK=PB6, MISO=PB7
// MagneticSensorSPI sensor_steering(PB5, PB6, PB7); // CSN, SCK, MISO

// Configuration 2: CSN=PB5, SCK=PB7, MISO=PB6
MagneticSensorSPI sensor_steering(PB5, PB7, PB6); // CSN, SCK, MISO

// Configuration 3: CSN=PB6, SCK=PB5, MISO=PB7
// MagneticSensorSPI sensor_steering(PB6, PB5, PB7); // CSN, SCK, MISO

// Configuration 4: CSN=PB6, SCK=PB7, MISO=PB5
// MagneticSensorSPI sensor_steering(PB6, PB7, PB5); // CSN, SCK, MISO

// Configuration 5: CSN=PB7, SCK=PB5, MISO=PB6
// MagneticSensorSPI sensor_steering(PB7, PB5, PB6); // CSN, SCK, MISO

// Configuration 6: CSN=PB7, SCK=PB6, MISO=PB5
// MagneticSensorSPI sensor_steering(PB7, PB6, PB5); // CSN, SCK, MISO

// Alternative pins avoiding hall sensors (PB5/6/7):
// Configuration 7: Using PA pins
// MagneticSensorSPI sensor_steering(PA8, PA9, PA10); // CSN, SCK, MISO

// Configuration 8: Using PB0/1/2
// MagneticSensorSPI sensor_steering(PB0, PB1, PB2); // CSN, SCK, MISO

// Configuration 9: Using PC pins
// MagneticSensorSPI sensor_steering(PC13, PC14, PC15); // CSN, SCK, MISO

// RACK SENSOR CONFIGURATIONS:
// Configuration R1: PC10, PC11, PC12 permutations
// MagneticSensorSPI sensor_rack(PC10, PC11, PC12);  // CSN, SCK, MISO
// MagneticSensorSPI sensor_rack(PC10, PC12, PC11);  // CSN, SCK, MISO
// MagneticSensorSPI sensor_rack(PC11, PC10, PC12);  // CSN, SCK, MISO
// MagneticSensorSPI sensor_rack(PC11, PC12, PC10);  // CSN, SCK, MISO
// MagneticSensorSPI sensor_rack(PC12, PC10, PC11);  // CSN, SCK, MISO
// MagneticSensorSPI sensor_rack(PC12, PC11, PC10);  // CSN, SCK, MISO

// CURRENTLY ACTIVE: Testing Configuration 2
// MagneticSensorSPI sensor_steering(PB5, PB7, PB6); // CSN, SCK, MISO

#define BUZZER_PIN PA4
#define LED_PIN PB2 // LED pin same as hoverboard main board

// ADC pin definitions (based on STM32F103 hoverboard)
#define BAT_VOLTAGE_PIN PA0 // Battery voltage measurement pin
#define TEMP_SENSOR_PIN PA1 // Temperature sensor pin (internal or external)

// UART communication defines
#define SERIAL_BAUD 115200 // [-] Baud rate for UART communication
#define START_FRAME 0xABCD // [-] Start frame definition for reliable serial communication

// Battery calibration constants (from config.h)
#define BAT_CELLS 10                // battery number of cells. Normal Hoverboard battery: 10s
#define BAT_CALIB_REAL_VOLTAGE 3970 // input voltage measured by multimeter (multiplied by 100). In this case 39.70 V * 100 = 3970
#define BAT_CALIB_ADC 1492          // adc-value measured by mainboard
#define BAT_FILT_COEF 655           // battery voltage filter coefficient in fixed-point. 655 = 0.01 * 2^16

// Temperature calibration constants (from config.h)
#define TEMP_CAL_LOW_ADC 1655   // temperature 1: ADC value
#define TEMP_CAL_LOW_DEG_C 358  // temperature 1: measured temperature [°C * 10]. Here 35.8 °C
#define TEMP_CAL_HIGH_ADC 1588  // temperature 2: ADC value
#define TEMP_CAL_HIGH_DEG_C 489 // temperature 2: measured temperature [°C * 10]. Here 48.9 °C
#define TEMP_FILT_COEF 655      // temperature filter coefficient in fixed-point. 655 = 0.01 * 2^16

// Timing variables
unsigned long lastBeepTime = 0;
const unsigned long beepInterval = 3000; // 3 seconds
unsigned long lastFeedbackTime = 0;
const unsigned long feedbackInterval = 10; // 10 ms (100 Hz feedback rate)
unsigned long lastAdcTime = 0;
const unsigned long adcInterval = 1; // 1 ms for ADC readings

// Serial feedback structure (same as in main.c)
typedef struct
{
  uint16_t start;
  int16_t cmd1;
  int16_t cmd2;
  int16_t speedR_meas;
  int16_t speedL_meas;
  int16_t batVoltage;
  int16_t boardTemp;
  uint16_t cmdLed;
  uint16_t checksum;
} SerialFeedback;

SerialFeedback Feedback;

// Real sensor data variables
int16_t batVoltage = (400 * BAT_CELLS * BAT_CALIB_ADC) / BAT_CALIB_REAL_VOLTAGE;            // Initialize to 4V per cell
int32_t batVoltageFixdt = (400 * BAT_CELLS * BAT_CALIB_ADC) / BAT_CALIB_REAL_VOLTAGE << 16; // Fixed-point filter
int16_t batVoltageCalib = 0;                                                                // Calibrated battery voltage

int32_t board_temp_adcFixdt = 1600 << 16; // Fixed-point filter initialized
int16_t board_temp_adcFilt = 1600;        // Filtered ADC value
int16_t board_temp_deg_c = 25;            // Temperature in degrees Celsius * 10

// Mock sensor data variables (for testing when ADC not available)
int16_t mock_speedR = 0;
int16_t mock_speedL = 0;
uint16_t mock_cmdLed = 0;

// Raw sensor readings
float raw_steering_angle = 0.0;
float raw_rack_angle = 0.0;

void setup()
{
  // Initialize Serial2 for UART communication on PA2 (RX) and PA3 (TX)
  Serial2.begin(SERIAL_BAUD);

  // Initialize ADC pins
  pinMode(BAT_VOLTAGE_PIN, INPUT_ANALOG);
  pinMode(TEMP_SENSOR_PIN, INPUT_ANALOG);

  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW); // Turn off LED initially
  delay(500);

  // Startup sequence: 3 quick beeps
  for (int i = 0; i < 1; i++)
  {
    beep();
    delay(100);
    beep2();
    delay(100);
    beep3();
    delay(100);
  }
  delay(1000);

  // COMPREHENSIVE SENSOR TESTING
  // Test 1: Basic initialization
  //beepLong(); // Signal start of sensor testing
  //delay(500);

  sensor_steering.init();
  delay(1000);

  // Test 2: Multiple sensor readings with update()
  for (int i = 0; i < 5; i++)
  {
    sensor_steering.update(); // Important: call update() before getAngle()
    delay(100);
  }

  float test_angle_1 = sensor_steering.getAngle();
  delay(100);
  sensor_steering.update();
  float test_angle_2 = sensor_steering.getAngle();
  delay(100);
  sensor_steering.update();
  float test_angle_3 = sensor_steering.getAngle();

  // Test 3: Check if readings are changing or all zeros
  bool all_zero = (test_angle_1 == 0.0 && test_angle_2 == 0.0 && test_angle_3 == 0.0);
  bool readings_change = (test_angle_1 != test_angle_2) || (test_angle_2 != test_angle_3);

  // Diagnostic beep patterns:
  if (all_zero)
  {
    // All readings are zero - likely connection/initialization problem
    for (int i = 0; i < 5; i++)
    {
      beep(); // 5 high beeps = all zero readings
      delay(200);
    }
  }
  else if (readings_change)
  {
    // Readings are changing - sensor working!
    for (int i = 0; i < 3; i++)
    {
      beep2(); // 3 medium beeps = sensor working, readings changing
      delay(300);
    }
  }
  else
  {
    // Non-zero but constant readings - might be working but not moving
    for (int i = 0; i < 2; i++)
    {
      beep3(); // 2 low beeps = constant non-zero reading
      delay(400);
    }
  }

  delay(2000);

  // Test 4: Check velocity (if sensor is working)
  if (!all_zero)
  {
    sensor_steering.update();
    float velocity = sensor_steering.getVelocity();

    if (velocity != 0.0)
    {
      beepWithLed(); // Velocity detected - sensor definitely working
      delay(500);
      beepWithLed();
    }
  }

  delay(1000);

  // Final test summary beeps
  for (int i = 0; i < 1; i++)
  {
    beep();
    delay(100);
    beep2();
    delay(100);
    beep3();
    delay(100);
  }
}
// // Test rack sensor initialization
// sensor_rack.init();
// delay(500);

// // Read initial rack sensor value
// float initial_rack = sensor_rack.getAngle();

// // Rack sensor status beeps:
// if (initial_rack == 0.0) {
//   // 3 low beeps = rack sensor might be disconnected/not working
//   beep2(); delay(300); beep2(); delay(300); beep2();
// } else if (initial_rack > 0.001 || initial_rack < -0.001) {
//   // 3 high beeps = rack sensor working (non-zero reading)
//   beep(); delay(300); beep(); delay(300); beep();
// } else {
//   // 2 medium beeps = rack sensor connected but reading exactly zero
//   beepMedium(); delay(300); beepMedium();
// }

// delay(2000);

// // Final startup beeps
// for (int i = 0; i < 3; i++)
// {
//   beepWithLed();
//   delay(1000);
// }

// lastBeepTime = millis();     // Initialize timer
// lastFeedbackTime = millis(); // Initialize feedback timer
// lastAdcTime = millis();      // Initialize ADC timer

void loop()
{
  // unsigned long currentTime = millis();

  // // Odczyt pozycji z obu enkoderów
  // raw_steering_angle = sensor_steering.getAngle();
  // raw_rack_angle = sensor_rack.getAngle();

  // // Handle ADC readings
  // if (currentTime - lastAdcTime >= adcInterval)
  // {
  //   readAndFilterADC();

  //   if (raw_steering_angle > 0)
  //   {
  //     beep();
  //     delay(100);
  //   }
  //   else
  //   {
  //     beep2();
  //     delay(1000);
  //     beep2();
  //     delay(1000);
  //     beep2();
  //     delay(1000);
  //   }
  //   lastAdcTime = currentTime;
  // }

  // // Handle beep timing
  // if (currentTime - lastBeepTime >= beepInterval)
  // {
  //   beepWithLed();
  //   lastBeepTime = currentTime;
  // }

  // // Handle UART feedback transmission
  // if (currentTime - lastFeedbackTime >= feedbackInterval)
  // {
  //   sendFeedback();
  //   lastFeedbackTime = currentTime;

  //   // Update mock sensor data (simulate some changing values)
  //   // updateMockData();
  // }
}

void beep()
{
  for (int i = 0; i < 200; i++)
  {
    digitalWrite(BUZZER_PIN, HIGH);
    delayMicroseconds(150);
    digitalWrite(BUZZER_PIN, LOW);
    delayMicroseconds(150);
  }
}

void beep2()
{
  for (int i = 0; i < 300; i++)
  {
    digitalWrite(BUZZER_PIN, HIGH);
    delayMicroseconds(250);
    digitalWrite(BUZZER_PIN, LOW);
    delayMicroseconds(250);
  }
}

void beep3()
{
  for (int i = 0; i < 400; i++)
  {
    digitalWrite(BUZZER_PIN, HIGH);
    delayMicroseconds(350);
    digitalWrite(BUZZER_PIN, LOW);
    delayMicroseconds(350);
  }
}

void beepLong()
{
  digitalWrite(BUZZER_PIN, HIGH);
  delay(500);
  digitalWrite(BUZZER_PIN, LOW);
}

void ledOn()
{
  digitalWrite(LED_PIN, HIGH);
}

void ledOff()
{
  digitalWrite(LED_PIN, LOW);
}

void beepWithLed()
{
  ledOn();
  beep();
  ledOff();
}

// Send feedback data via UART (similar to main.c implementation)
void sendFeedback()
{
  // Convert raw encoder angles to int16_t
  // Method 1: Multiply by 1000 for higher precision (3 decimal places)
  int16_t steering_angle_int = (int16_t)(raw_steering_angle * 1000);
  int16_t rack_angle_int = (int16_t)(raw_rack_angle * 1000);

  // Method 2: Send raw angle as degrees * 10 (1 decimal place)
  // int16_t steering_angle_int = (int16_t)(raw_steering_angle * 180.0 / PI * 10);
  // int16_t rack_angle_int = (int16_t)(raw_rack_angle * 180.0 / PI * 10);

  // Populate feedback structure
  Feedback.start = (uint16_t)START_FRAME;
  Feedback.cmd1 = steering_angle_int;                         // Send steering encoder angle (raw * 1000)
  Feedback.cmd2 = rack_angle_int;                             // Send rack encoder angle (raw * 1000)
  Feedback.speedR_meas = (int16_t)(raw_steering_angle * 100); // Alternative: send with less precision in speedR
  Feedback.speedL_meas = (int16_t)(raw_rack_angle * 100);     // Alternative: send with less precision in speedL
  Feedback.batVoltage = batVoltageCalib;                      // Use real calibrated battery voltage
  Feedback.boardTemp = board_temp_deg_c;                      // Use real measured temperature
  Feedback.cmdLed = mock_cmdLed;

  // Calculate checksum (XOR of all fields except checksum itself)
  Feedback.checksum = (uint16_t)(Feedback.start ^ Feedback.cmd1 ^ Feedback.cmd2 ^ Feedback.speedR_meas ^ Feedback.speedL_meas ^ Feedback.batVoltage ^ Feedback.boardTemp ^ Feedback.cmdLed);

  // Send the feedback structure via Serial2 (USART2)
  Serial2.write((uint8_t *)&Feedback, sizeof(Feedback));
}

// Low-pass filter function (same as in util.c)
void filtLowPass32(int32_t u, uint16_t coef, int32_t *y)
{
  int64_t tmp;
  tmp = ((int64_t)((u << 4) - (*y >> 12)) * coef) >> 4;
  // Overflow protection: 2147483647LL = 2^31 - 1
  if (tmp > 2147483647LL)
    tmp = 2147483647LL;
  if (tmp < -2147483648LL)
    tmp = -2147483648LL;
  *y = (int32_t)tmp + (*y);
}

// Read and filter ADC values (similar to main.c implementation)
void readAndFilterADC()
{
  static uint16_t adcCounter = 0;
  adcCounter++;

  // Filter battery voltage at a slower sampling rate (every 1000 calls, similar to main.c)
  if (adcCounter % 1000 == 0)
  {
    uint16_t batAdc = analogRead(BAT_VOLTAGE_PIN);
    filtLowPass32(batAdc, BAT_FILT_COEF, &batVoltageFixdt);
    batVoltage = (int16_t)(batVoltageFixdt >> 16); // convert fixed-point to integer

    // Calculate calibrated battery voltage (same as main.c)
    batVoltageCalib = batVoltage * BAT_CALIB_REAL_VOLTAGE / BAT_CALIB_ADC;
  }

  // Read and filter temperature every call
  uint16_t tempAdc = analogRead(TEMP_SENSOR_PIN);
  filtLowPass32(tempAdc, TEMP_FILT_COEF, &board_temp_adcFixdt);
  board_temp_adcFilt = (int16_t)(board_temp_adcFixdt >> 16); // convert fixed-point to integer

  // Calculate temperature in degrees Celsius (same formula as main.c)
  board_temp_deg_c = (TEMP_CAL_HIGH_DEG_C - TEMP_CAL_LOW_DEG_C) * (board_temp_adcFilt - TEMP_CAL_LOW_ADC) / (TEMP_CAL_HIGH_ADC - TEMP_CAL_LOW_ADC) + TEMP_CAL_LOW_DEG_C;
}
