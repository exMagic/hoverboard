/* ===========================================================
   TEST POJEDYNCZEGO MT6816  –  SOFTWARE SPI
   Płyta   :  STM32F103RCT6 (hoverboard mainboard)
   Enkoder :  MT6816-STD   (wymaga linii MOSI **i** MISO)
   Piny    :  SCK  = PB5
              CS   = PB6
              MOSI = PB7
              MISO = PA2
   Diagnostyka: Buzzer na PA4 (krótkie sygnały dźwiękowe)

   Wzorce buzzera
   ──────────────────────────────────────────────────────────
   • 5 szybkich beepów   – brak komunikacji SPI
   • 3 długie beepy      – brak magnesu  (No_Mag_Warning = 1)
   • 2 średnie beepy     – czujnik działa; dodatkowy ton → kąt
   • 1 bardzo długi beep – czujnik działa, lecz kąt się nie zmienia
   =========================================================== */

#define SCK_PIN   PB7
#define CS_PIN    PB6
#define MOSI_PIN  PB5
#define MISO_PIN  PB10

#define BUZZER_PIN PA4
#define LED_PIN    PB2      // pomocnicza dioda

// częstotliwości sygnałów
#define FREQ_OK      2000    // wysoki ton  (sukces)
#define FREQ_WARN    1000    // średni ton (ostrzeżenie)
#define FREQ_ERROR     500   // niski ton   (błąd)

void beep (uint16_t f, uint16_t ms) {
  if (!f) { digitalWrite(BUZZER_PIN, LOW); delay(ms); return; }
  const uint32_t T = 1000000UL / f;
  const uint32_t half = T / 2;
  const uint32_t cyc  = (ms * 1000UL) / T;
  for (uint32_t i = 0; i < cyc; i++) {
    digitalWrite(BUZZER_PIN, HIGH); delayMicroseconds(half);
    digitalWrite(BUZZER_PIN, LOW ); delayMicroseconds(half);
  }
}

/* ----------- proste makro do strojenia opóźnień ------------- */
#define SPI_DELAY()  delayMicroseconds(3)

/* -----------------------  SPI  ------------------------------ */
void spiInit() {
  pinMode(CS_PIN  , OUTPUT);
  pinMode(SCK_PIN , OUTPUT);
  pinMode(MOSI_PIN, OUTPUT);
  pinMode(MISO_PIN, INPUT);           // NIE pull-up / pull-down
  digitalWrite(CS_PIN , HIGH);        // CS nieaktywny
  digitalWrite(SCK_PIN, HIGH);        // CPOL=1  (SPI-Mode 3)
}

void spiWriteBit(bool bit) {
  digitalWrite(SCK_PIN, LOW);
  digitalWrite(MOSI_PIN, bit);
  SPI_DELAY();
  digitalWrite(SCK_PIN, HIGH);
  SPI_DELAY();
}

bool spiReadBit() {
  digitalWrite(SCK_PIN, LOW);
  SPI_DELAY();
  digitalWrite(SCK_PIN, HIGH);
  bool b = digitalRead(MISO_PIN);
  SPI_DELAY();
  return b;
}

uint8_t spiTransfer8(uint8_t out) {
  uint8_t in = 0;
  for (int8_t i = 7; i >= 0; --i) {
    spiWriteBit(out & (1 << i));
    in = (in << 1) | spiReadBit();
  }
  return in;
}

/* -----------------  MT6816  -------------------------------- */
uint8_t readReg(uint8_t addr) {
  uint8_t cmd = 0x80 | (addr << 1);   // bit0=1 (READ)
  digitalWrite(CS_PIN, LOW);
  SPI_DELAY();
  spiTransfer8(cmd);          // wysłanie komendy
  uint8_t data = spiTransfer8(0x00);  // odczyt danych
  digitalWrite(CS_PIN, HIGH);
  return data;
}

struct EncStatus {
  bool ok, magnet;
  uint16_t raw;
  float deg;
};

EncStatus readMT() {
  EncStatus s = {};
  uint8_t hi = readReg(0x03);
  uint8_t lo = readReg(0x04);
  if ((hi == 0x00 || hi == 0xFF) && (lo == 0x00 || lo == 0xFF)) return s;
  s.ok = true;
  s.magnet = !(lo & 0x02);          // No_Mag_Warning = bit1
  s.raw = (hi << 6) | ((lo >> 2) & 0x3F);   // 14 bitów
  s.deg = s.raw * 360.0f / 16384.0f;
  return s;
}

/* -----------------  SETUP  --------------------------------- */
void setup() {
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_PIN   , OUTPUT);
  spiInit();

  digitalWrite(LED_PIN, HIGH);           // sygnał startowy
  beep(FREQ_OK, 400);
  digitalWrite(LED_PIN, LOW);
}

/* -----------------  LOOP  ---------------------------------- */
void loop() {
  static uint16_t lastRaw = 0;
  EncStatus e = readMT();

  if (!e.ok) {                     // brak komunikacji
    for (byte i = 0; i < 5; i++) { beep(FREQ_OK, 100); delay(350); }
    delay(1500);
    return;
  }
  if (!e.magnet) {                 // brak magnesu
    for (byte i = 0; i < 3; i++) { beep(FREQ_ERROR, 400); delay(300); }
    delay(1500);
    return;
  }

  // czujnik działa
  for (byte i = 0; i < 2; i++) { beep(FREQ_WARN, 250); delay(200); }

  // dodatkowy ton reprezentujący kąt: 200 Hz + 8 Hz/°
  uint16_t f = 200 + (uint16_t)(e.deg * 8);
  if (f > 3500) f = 3500;
  beep(f, 250);

  // sprawdź, czy kąt się zmienił
  if (abs((int)e.raw - (int)lastRaw) < 20) {   // ~0.4°
    beep(FREQ_WARN, 800);          // brak ruchu
  }
  lastRaw = e.raw;

  delay(1000);
}
