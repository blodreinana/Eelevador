#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#define STEP_PIN 5
#define DIR_PIN  6

#define BUTTON_UP    2
#define BUTTON_DOWN  3

LiquidCrystal_I2C lcd(0x27, 16, 2);

int potencia = 0; // -100 a +100
int lastPotencia = 127;
unsigned long lastBtnUpTime = 0;
unsigned long lastBtnDnTime = 0;
unsigned long lastStepTime = 0;

const unsigned long debounceMs = 100;

// Ajustado para 1/4-step: frequência máxima quadruplicada
const float MAX_FREQ = 12000.0; // 3000 * 4

void setup() {
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(BUTTON_UP, INPUT_PULLUP);
  pinMode(BUTTON_DOWN, INPUT_PULLUP);

  lcd.init();
  lcd.backlight();

  lcd.setCursor(0, 1);
  lcd.print("Modo: 1/4-step");
}

void loop() {
  unsigned long nowMs = millis();
  unsigned long nowUs = micros();

  // —— Botões ——
  if (digitalRead(BUTTON_UP) == LOW && nowMs - lastBtnUpTime > debounceMs) {
    potencia = min(potencia + 1, 100);
    lastBtnUpTime = nowMs;
  }

  if (digitalRead(BUTTON_DOWN) == LOW && nowMs - lastBtnDnTime > debounceMs) {
    potencia = max(potencia - 1, -100);
    lastBtnDnTime = nowMs;
  }

  // —— Display ——
  if (potencia != lastPotencia) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Potencia: ");
    lcd.print(potencia);
    lcd.print("%");

    lcd.setCursor(0, 1);
    lcd.print("Modo: 1/4-step");

    lastPotencia = potencia;
  }

  // —— Controle real de STEP ——
  if (potencia != 0) {
    digitalWrite(DIR_PIN, potencia > 0 ? HIGH : LOW);

    float pot = constrain(abs(potencia), 1, 100);

    // curva suave + limite de frequência
    float freq = pow(pot / 100.0, 2.2) * MAX_FREQ;
    unsigned long intervalo = 1000000.0 / freq; // micros por passo

    if (nowUs - lastStepTime >= intervalo) {
      lastStepTime = nowUs;

      digitalWrite(STEP_PIN, HIGH);
      delayMicroseconds(2);
      digitalWrite(STEP_PIN, LOW);
    }
  }
}
