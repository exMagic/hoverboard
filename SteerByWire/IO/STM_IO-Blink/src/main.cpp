#include <Arduino.h>

#define LED_PIN PB2   

void setup() {

  pinMode(LED_PIN, OUTPUT);
  

  digitalWrite(LED_PIN, LOW);
}

void loop() {
  // Turn on all LEDs
  digitalWrite(LED_PIN, HIGH);
  
  delay(100);
  
  // Turn off all LEDs
  digitalWrite(LED_PIN, LOW);

  
  delay(200);
}
