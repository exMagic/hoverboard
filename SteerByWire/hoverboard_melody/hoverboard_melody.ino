// Simple buzzer for STM32 hoverboard
#include <Wire.h>  // Include the Wire library for I2C

#define BUZZER_PIN PA4
#define LED_PIN PB2    // LED pin same as hoverboard main board
#define I2C_SLAVE_ADDRESS 0x08  // Address of the I2C slave device

unsigned long lastBeepTime = 0;
unsigned long lastMessageTime = 0;
const unsigned long beepInterval = 3000;  // 3 seconds
const unsigned long messageInterval = 2000;  // 2 seconds for messages
unsigned long messageCounter = 0;

void setup() {
  // Initialize serial communication
  // Using Serial - will map to hardware UART with correct board settings
  Serial.begin(115200);  // Use Serial for your RS232 cable
  delay(2000);  // Give time for serial to initialize properly
  
  Wire.begin();  // Initialize I2C as master
  Wire.setSCL(PB10);  // Set SCL to PB10 for I2C2
  Wire.setSDA(PB11);  // Set SDA to PB11 for I2C2
  
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);  // Turn off LED initially
  delay(500);
  
  Serial.println("=== Hoverboard Test System Started ===");
  Serial.println("RS232 Communication Active - 115200 baud");
  Serial.println("Sending status messages every 2 seconds...");
  Serial.println("");
  
  // 3 beeps on startup
  for (int i = 0; i < 3; i++) {
    beepWithLed();
    Serial.print("Startup beep #");
    Serial.println(i + 1);
    delay(200);
  }
  
  lastBeepTime = millis();
  lastMessageTime = millis();
}

void loop() {
  // Send message every 2 seconds
  if (millis() - lastMessageTime >= messageInterval) {
    sendStatusMessage();
    lastMessageTime = millis();
  }
  
  // Beep every 3 seconds
  if (millis() - lastBeepTime >= beepInterval) {
    beepWithLed();
    Serial.println(">>> BEEP! <<<");
    lastBeepTime = millis();
  }
}

void beep() {
  for (int i = 0; i < 20; i++) {
    digitalWrite(BUZZER_PIN, HIGH);
    delayMicroseconds(450);
    digitalWrite(BUZZER_PIN, LOW);
    delayMicroseconds(450);
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

void sendStatusMessage() {
  messageCounter++;
  
  Serial.print("[");
  Serial.print(messageCounter);
  Serial.print("] Status: System running OK | Uptime: ");
  Serial.print(millis() / 1000);
  Serial.print("s | Temperature: ");
  Serial.print(20 + (messageCounter % 15));  // Fake temperature 20-35°C
  Serial.println("°C");
  
  // Send some fake sensor data every 5 messages
  if (messageCounter % 5 == 0) {
    Serial.print("    Battery: 36.2V | Motors: OK | LED: ");
    Serial.println(digitalRead(LED_PIN) ? "ON" : "OFF");
  }
  
  Wire.beginTransmission(I2C_SLAVE_ADDRESS);
  Wire.write("Status: System OK");
  Wire.endTransmission();
}
