// ########################## DEFINES ##########################
#define BUZZER_PIN 4            // Buzzer pin (PA4 = pin 4 in Arduino)
#define BEEP_FREQUENCY 2000     // Beep frequency in Hz
#define BEEP_DURATION 100       // Short beep duration in ms

// ########################## SETUP ##########################
void setup() 
{
  // Initialize buzzer pin
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  
  // Delay to ensure system is ready
  delay(500);
  
  // Beep 3 times on startup
  for(int i = 0; i < 3; i++) {
    tone(BUZZER_PIN, BEEP_FREQUENCY, BEEP_DURATION);
    delay(BEEP_DURATION + 200);  // Wait for beep to finish + pause
  }
}

// ########################## LOOP ##########################
void loop() 
{
  // One short beep every 3 seconds
  tone(BUZZER_PIN, BEEP_FREQUENCY, BEEP_DURATION);
  delay(3000);  // 3 second delay
}
