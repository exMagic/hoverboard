// Bootloader.ino
#include <stdint.h>
#include <stm32f1xx.h>
#define BUZZER_PIN PA4
#define APP_ADDRESS 0x08005000UL
void buzzStart() { for(int i=0;i<4;i++){ tone(BUZZER_PIN,300,300); delay(500);} }
typedef void (*app_entry_t)(void);

void setup() {
  buzzStart();
  // możesz tu dodać diagnostykę buzzerem, LED itp.
}

void loop() {
  // 1) Wyłącz przerwania i SysTick
  __disable_irq();
  SysTick->CTRL = 0;

  // 2) Przeładuj wektor przerwań na aplikację
  SCB->VTOR = APP_ADDRESS;

  // 3) Ustaw MSP na wartość z początku wektora aplikacji
  uint32_t msp = *(uint32_t*)(APP_ADDRESS + 0);
  __set_MSP(msp);

  // 4) Pobierz adres Reset_Handler aplikacji i do niej skocz
  uint32_t reset_handler = *(uint32_t*)(APP_ADDRESS + 4);
  app_entry_t app = (app_entry_t)reset_handler;

  __enable_irq();
  app();
  
  // nigdy tu nie wracamy
  while (1);
}
