#include <Wire.h>

void setup() {
  Wire.begin(0x08);  // Initialize I2C as slave with address 0x08
  Wire.onReceive(receiveEvent);  // Register receive event
  Serial.begin(115200);  // For debugging
  Serial.println("HELLO");
}

void loop() {
  // Do nothing, wait for I2C messages
  
}

void receiveEvent(int bytes) {
  while (Wire.available()) {
    char c = Wire.read();  // Read each byte
    Serial.print(c);       // Print received data to serial monitor
  }
  Serial.println();
}