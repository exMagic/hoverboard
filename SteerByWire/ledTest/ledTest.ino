// Test identyfikacji pinów Hall sensors przez LED
#define BUZZER_PIN PA4
#define LED_PIN PC13

// Piny do testowania (wszystkie możliwe piny Hall)
uint8_t test_pins[] = {PB7, PB6, PB10, PB8, PC4, PC5, PC13};
String pin_names[] = {"PB5", "PB6", "PB7", "PB8", "PC4", "PC5", "PC13"};
uint8_t num_pins = 7;

void beep_identify(int pin_number) {
  // Beep pattern dla identyfikacji pinu
  // pin_number + 1 krótkich beepów (np. pin 0 = 1 beep, pin 1 = 2 beepy)
  for (int i = 0; i <= pin_number; i++) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(100);
    digitalWrite(BUZZER_PIN, LOW);
    delay(200);
  }
  delay(1000); // Pauza między pinami
}

void setup() {
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  
  // Sygnał startowy
  digitalWrite(LED_PIN, HIGH);
  for (int i = 0; i < 3; i++) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(300);
    digitalWrite(BUZZER_PIN, LOW);
    delay(200);
  }
  digitalWrite(LED_PIN, LOW);
  
  delay(2000); // Przygotowanie przed testem
}

void loop() {
  // Test każdego pinu pojedynczo
  for (int i = 0; i < num_pins; i++) {
    
    // Konfiguruj pin jako OUTPUT
    pinMode(test_pins[i], OUTPUT);
    
    // Sygnał identyfikacyjny przez buzzer (liczba beepów = numer pinu + 1)
    beep_identify(i);
    
    // Zapal LED przez pin testowy na 3 sekundy
    digitalWrite(test_pins[i], HIGH);
    delay(3000);
    digitalWrite(test_pins[i], LOW);
    
    // Pauza między testami
    delay(2000);
  }
  
  // Długa pauza przed kolejnym cyklem
  delay(5000);
}
