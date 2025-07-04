/*
 * Simple Arduino Sketch for Hoverboard Mainboard Buzzer
 * 
 * This sketch provides simple beeping functionality on the hoverboard mainboard.
 * 
 * Hardware:
 * - STM32F103 (hoverboard mainboard)
 * - Buzzer connected to:
 *   * Board Variant 0: PA4 (Pin A4)
 * 
 * Notes:
 * - Use STM32duino core for Arduino IDE
 * - Select "Generic STM32F1 series" board
 * - Choose "STM32F103RC" variant
 */



#define BUZZER_PIN PA4
#define BEEP_FREQUENCY 2000     // Beep frequency in Hz
#define BEEP_DURATION 100       // Short beep duration in ms

void setup() {
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  
  // Delay to ensure system is ready
  delay(500);
  
  // 3 quick beeps on startup
  Serial.println("Playing 3 startup beeps...");
  for (int i = 0; i < 3; i++) {
    playTone(BEEP_FREQUENCY, BEEP_DURATION);
    delay(200);  // Short pause between beeps
  }  
}

void loop() {
  // One short beep every 3 seconds
  playTone(BEEP_FREQUENCY, BEEP_DURATION);
  delay(3000);  // 3 second delay
}

void playTone(int frequency, int duration) {
  if (frequency == 0) {
    delay(duration);
    return;
  }
  
  // Calculate the period for the square wave
  int period = 1000000 / frequency;  // Period in microseconds
  int halfPeriod = period / 2;
  
  // Calculate how many cycles we need for the duration
  long cycles = (long)frequency * duration / 1000;
  
  // Generate square wave for the specified duration
  for (long i = 0; i < cycles; i++) {
    digitalWrite(BUZZER_PIN, HIGH);
    delayMicroseconds(halfPeriod);
    digitalWrite(BUZZER_PIN, LOW);
    delayMicroseconds(halfPeriod);
  }
}
