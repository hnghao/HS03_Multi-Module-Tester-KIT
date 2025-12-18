#ifndef URM37_MODE_H
#define URM37_MODE_H

#include <Arduino.h>
#include <LiquidCrystal_I2C.h>

// ==== dùng chung từ chương trình chính ====
extern LiquidCrystal_I2C lcd;
extern void lcdPrintLine(uint8_t row, const char* text);
extern bool headerEnabled;

extern AppState appState;

// ===== URM37 Trig/Echo(PWM) pins on ESP32-S3 =====
static const uint8_t PIN_URECHO = 41;   // ECHO/PWM output từ URM37 (nhớ hạ mức về 3.3V)
static const uint8_t PIN_URTRIG = 42;   // TRIG/COMP của URM37

// ===== URM37 timing (theo code mẫu DFRobot) =====
static const uint32_t INVALID_LIMIT_US = 50000;  // >= 50ms -> invalid
static const uint32_t PULSE_TIMEOUT_US = 60000;  // timeout pulseIn (nên > 50ms)
static const uint32_t URM37_READ_INTERVAL_MS = 200;

static bool urm37_prevHeaderEnabled = true;

static inline void urm37Trigger() {
  digitalWrite(PIN_URTRIG, HIGH);
  delayMicroseconds(5);

  digitalWrite(PIN_URTRIG, LOW);
  delayMicroseconds(20);
  digitalWrite(PIN_URTRIG, HIGH);
}

static bool urm37ReadDistanceCm(uint16_t &distanceCm, uint32_t &lowTimeUs) {
  urm37Trigger();

  lowTimeUs = pulseIn(PIN_URECHO, LOW, PULSE_TIMEOUT_US);

  if (lowTimeUs == 0 || lowTimeUs >= INVALID_LIMIT_US) return false;

  distanceCm = (uint16_t)(lowTimeUs / 50UL);  // 50us = 1cm
  return true;
}

inline void startURM37Mode() {
  appState = STATE_DFROBOT_URM37;

  // Giữ nguyên hiển thị dòng 0 như sketch bạn gửi
  urm37_prevHeaderEnabled = headerEnabled;
  headerEnabled = false;

  pinMode(PIN_URTRIG, OUTPUT);
  digitalWrite(PIN_URTRIG, HIGH);
  pinMode(PIN_URECHO, INPUT);

  lcd.clear();
  lcdPrintLine(0, "URM37 PWM Distance");
  lcdPrintLine(1, "Init the sensor...");
  lcdPrintLine(2, "LCD SDA=GPIO6");
  lcdPrintLine(3, "LCD SCL=GPIO7");
  delay(800);
}

inline void updateURM37Mode(unsigned long now) {
  static uint32_t lastMs = 0;
  if ((uint32_t)(now - lastMs) < URM37_READ_INTERVAL_MS) return;
  lastMs = (uint32_t)now;

  uint16_t cm = 0;
  uint32_t lowUs = 0;
  bool ok = urm37ReadDistanceCm(cm, lowUs);

  lcdPrintLine(0, "URM37 PWM Distance");

  if (ok) {
    char line1[21];
    snprintf(line1, sizeof(line1), "Distance: %u cm", (unsigned)cm);
    lcdPrintLine(1, line1);

    lcdPrintLine(2, "Status: OK");

    char line3[21];
    snprintf(line3, sizeof(line3), "Low: %lu us", (unsigned long)lowUs);
    lcdPrintLine(3, line3);
  } else {
    lcdPrintLine(1, "Distance: ---- cm");
    lcdPrintLine(2, "Status: Invalid");
    lcdPrintLine(3, "Check wiring/level");
  }
}

inline void stopURM37Mode() {
  headerEnabled = urm37_prevHeaderEnabled;
}

#endif
