// Simple buzzer for STM32 hoverboard
#define BUZZER_PIN PA4
#define LED_PIN PB2    // LED pin same as hoverboard main board

unsigned long lastBeepTime = 0;
const unsigned long beepInterval = 3000;  // 3 seconds

void setup() {
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
}

void loop() {
  if (millis() - lastBeepTime >= beepInterval) {
    beepWithLed();
    lastBeepTime = millis();
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
