#include <Arduino.h>

/*
 * STM_IO-Blink
 * Simple LED blink project for STM32F103RC (Hoverboard)
 * 
 * Hardware: STM32F103RC based hoverboard
 * Pin: PB2 (LED)
 * 
 * This project demonstrates basic GPIO control by blinking an LED
 * connected to pin PB2 at 1Hz frequency (500ms ON, 500ms OFF).
 */

// Pin definitions - try multiple common LED pins
#define LED_PIN1 PB2    // Common hoverboard LED pin
#define LED_PIN2 PB5    // Alternative LED pin
#define LED_PIN3 PC13   // Common on many STM32 boards

void setup() {
  // Initialize LED pins as outputs
  pinMode(LED_PIN1, OUTPUT);
  pinMode(LED_PIN2, OUTPUT);
  pinMode(LED_PIN3, OUTPUT);
  
  // Turn off LEDs initially
  digitalWrite(LED_PIN1, LOW);
  digitalWrite(LED_PIN2, LOW);
  digitalWrite(LED_PIN3, LOW);
  
  // Optional: Initialize serial communication for debugging
  Serial.begin(115200);
  Serial.println("STM_IO-Blink initialized");
  Serial.println("Testing multiple LED pins: PB2, PB5, PC13");
  Serial.println("Blink frequency: 1Hz (500ms ON/OFF)");
}

void loop() {
  // Turn on all LEDs
  digitalWrite(LED_PIN1, HIGH);
  digitalWrite(LED_PIN2, HIGH);
  digitalWrite(LED_PIN3, HIGH);
  
  Serial.println("LEDs: ON");
  delay(500);
  
  // Turn off all LEDs
  digitalWrite(LED_PIN1, LOW);
  digitalWrite(LED_PIN2, LOW);
  digitalWrite(LED_PIN3, LOW);
  
  Serial.println("LEDs: OFF");
  delay(500);
}
