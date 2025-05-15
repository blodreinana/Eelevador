#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// ————— PINOS —————
#define BUTTON_UP    2   // D2 aumenta potencia
#define BUTTON_DOWN  3   // D3 diminui potencia

// mascaras para port manipulation
const uint8_t maskIN1 = _BV(PORTD5);  // pin 5 → PD5
const uint8_t maskIN2 = _BV(PORTD6);  // pin 6 → PD6
const uint8_t maskIN3 = _BV(PORTB1);  // pin 9 → PB1
const uint8_t maskIN4 = _BV(PORTB2);  // pin 10→ PB2

LiquidCrystal_I2C lcd(0x27, 16, 2);

// full-step sequence (4 fases)
const uint8_t seqFull[4][4] = {
  {1,0,1,0},  // IN1+IN3
  {0,1,1,0},  // IN2+IN3
  {0,1,0,1},  // IN2+IN4
  {1,0,0,1}   // IN1+IN4
};

int potencia         = 0;    // –100…+100 %
int lastDisplayValue = 127;  // valor inválido p/ forçar 1º LCD
int stepIdx          = 0;

// temporizadores
unsigned long lastBtnUpMs = 0, lastBtnDnMs = 0;
unsigned long lastStepUs  = 0;

// parâmetros de timing
const unsigned long debounceMs    =  50;   
const unsigned long maxIntervalUs = 20000;  
const unsigned long minIntervalUs =  200;  
void setup() {
  // botões
  pinMode(BUTTON_UP,   INPUT_PULLUP);
  pinMode(BUTTON_DOWN, INPUT_PULLUP);
  // bobinas
  pinMode(5,  OUTPUT);
  pinMode(6,  OUTPUT);
  pinMode(9,  OUTPUT);
  pinMode(10, OUTPUT);

  lcd.init();
  lcd.backlight();
}

void loop() {
  unsigned long nowMs = millis();
  unsigned long nowUs = micros();

  // ——— leitura de botões não bloqueante ———
  if (digitalRead(BUTTON_UP)==LOW && nowMs - lastBtnUpMs > debounceMs) {
    potencia = min(potencia + 1, 100);
    lastBtnUpMs = nowMs;
  }
  if (digitalRead(BUTTON_DOWN)==LOW && nowMs - lastBtnDnMs > debounceMs) {
    potencia = max(potencia - 1, -100);
    lastBtnDnMs = nowMs;
  }

  // ——— atualiza LCD só se mudou ———
  if (potencia != lastDisplayValue) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Potencia: ");
    lcd.print(potencia);#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// ————— PINOS —————
#define BUTTON_UP    2   // D2 aumenta potencia
#define BUTTON_DOWN  3   // D3 diminui potencia

// mascaras para port manipulation
const uint8_t maskIN1 = _BV(PORTD5);  // pin 5 → PD5
const uint8_t maskIN2 = _BV(PORTD6);  // pin 6 → PD6
const uint8_t maskIN3 = _BV(PORTB1);  // pin 9 → PB1
const uint8_t maskIN4 = _BV(PORTB2);  // pin 10→ PB2

LiquidCrystal_I2C lcd(0x27, 16, 2);

// full-step sequence (4 fases)
const uint8_t seqFull[4][4] = {
  {1,0,1,0},  // IN1+IN3
  {0,1,1,0},  // IN2+IN3
  {0,1,0,1},  // IN2+IN4
  {1,0,0,1}   // IN1+IN4
};

int potencia         = 0;    // –100…+100 %
int lastDisplayValue = 127;  // valor inválido p/ forçar 1º LCD
int stepIdx          = 0;

// temporizadores
unsigned long lastBtnUpMs = 0, lastBtnDnMs = 0;
unsigned long lastStepUs  = 0;

// parâmetros de timing
const unsigned long debounceMs    =  50;    // 50 ms debounce
const unsigned long maxIntervalUs = 20000;  // 20 ms → potência 1%
const unsigned long minIntervalUs =  200;   // 0.2 ms → potência 100%

void setup() {
  // botões
  pinMode(BUTTON_UP,   INPUT_PULLUP);
  pinMode(BUTTON_DOWN, INPUT_PULLUP);
  // bobinas
  pinMode(5,  OUTPUT);
  pinMode(6,  OUTPUT);
  pinMode(9,  OUTPUT);
  pinMode(10, OUTPUT);

  lcd.init();
  lcd.backlight();
}

void loop() {
  unsigned long nowMs = millis();
  unsigned long nowUs = micros();

  // ——— leitura de botões não bloqueante ———
  if (digitalRead(BUTTON_UP)==LOW && nowMs - lastBtnUpMs > debounceMs) {
    potencia = min(potencia + 1, 100);
    lastBtnUpMs = nowMs;
  }
  if (digitalRead(BUTTON_DOWN)==LOW && nowMs - lastBtnDnMs > debounceMs) {
    potencia = max(potencia - 1, -100);
    lastBtnDnMs = nowMs;
  }

  // ——— atualiza LCD só se mudou ———
  if (potencia != lastDisplayValue) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Potencia: ");
    lcd.print(potencia);
    lcd.print("%");
    lcd.setCursor(0, 1);
    if      (potencia > 0) lcd.print("Dir: Frente");
    else if (potencia < 0) lcd.print("Dir: Reverso");
    else                   lcd.print("Status: Freiado");
    lastDisplayValue = potencia;
  }

  // ——— geração de pulsos com micros() e port manipulation ———
  if (potencia != 0) {
    // mapeia |potencia| → intervalo entre steps
    unsigned long interval = map(abs(potencia),
                                0.5, 5,
                                maxIntervalUs,
                                minIntervalUs);
    if (nowUs - lastStepUs >= interval) {
      lastStepUs = nowUs;
      // avança ou retrocede o índice full-step
      stepIdx = (potencia > 0)
                ? (stepIdx + 1) % 4
                : (stepIdx + 3) % 4;
      // escreve direto nos registradores
      uint8_t regD = PORTD;
      uint8_t regB = PORTB;
      // PD5
      if (seqFull[stepIdx][0]) regD |= maskIN1; else regD &= ~maskIN1;
      // PD6
      if (seqFull[stepIdx][1]) regD |= maskIN2; else regD &= ~maskIN2;
      // PB1
      if (seqFull[stepIdx][2]) regB |= maskIN3; else regB &= ~maskIN3;
      // PB2
      if (seqFull[stepIdx][3]) regB |= maskIN4; else regB &= ~maskIN4;
      // aplica
      PORTD = regD;
      PORTB = regB;
    }
  }
  // se potencia == 0, não mexe no stepIdx → ultima fase fica energizada (freio)
}

    lcd.print("%");aaa
    lcd.setCursor(0, 1);
    if      (potencia > 0) lcd.print("Dir: Frente");
    else if (potencia < 0) lcd.print("Dir: Reverso");
    else                   lcd.print("Status: Freiado");
    lastDisplayValue = potencia;
  }

  // ——— geração de pulsos com micros() e port manipulation ———
  if (potencia != 0) {
    // mapeia |potencia| → intervalo entre steps
    unsigned long interval = map(abs(potencia),
                                1, 100,
                                maxIntervalUs,
                                minIntervalUs);
    if (nowUs - lastStepUs >= interval) {
      lastStepUs = nowUs;
      // avança ou retrocede o índice full-step
      stepIdx = (potencia > 0)
                ? (stepIdx + 1) % 4
                : (stepIdx + 3) % 4;
      // escreve direto nos registradores
      uint8_t regD = PORTD;
      uint8_t regB = PORTB;
      // PD5
      if (seqFull[stepIdx][0]) regD |= maskIN1; else regD &= ~maskIN1;
      // PD6
      if (seqFull[stepIdx][1]) regD |= maskIN2; else regD &= ~maskIN2;
      // PB1
      if (seqFull[stepIdx][2]) regB |= maskIN3; else regB &= ~maskIN3;
      // PB2
      if (seqFull[stepIdx][3]) regB |= maskIN4; else regB &= ~maskIN4;
      // aplica
      PORTD = regD;
      PORTB = regB;
    }
  }
  // se potencia == 0, não mexe no stepIdx → ultima fase fica energizada (freio)
}
