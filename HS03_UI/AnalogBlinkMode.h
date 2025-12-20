#pragma once
#include <Arduino.h>
#include <LiquidCrystal_I2C.h>

// Dùng LED_GREEN_PIN (GPIO4) để tránh đụng NEOPIXEL_PIN=5
#ifndef ANALOG_BLINK_LED_PIN
  #define ANALOG_BLINK_LED_PIN 5
#endif

extern LiquidCrystal_I2C lcd;
extern AppState appState;

extern char headerLabel[16];
void updateHeaderRow();
void lcdPrintLine(uint8_t row, const char *text);

// Blink timing
static int              blinkLedState      = LOW;
static unsigned long    blinkPrevMillis    = 0;
static const unsigned long BLINK_INTERVAL_MS = 500;

inline void startAnalogBlinkMode() {
  appState = STATE_ANALOG_BLINK;

  // Header
  strncpy(headerLabel, "Blink LED", sizeof(headerLabel));
  headerLabel[sizeof(headerLabel) - 1] = '\0';
  updateHeaderRow();

  pinMode(ANALOG_BLINK_LED_PIN, OUTPUT);
  blinkLedState   = LOW;
  digitalWrite(ANALOG_BLINK_LED_PIN, blinkLedState);
  blinkPrevMillis = millis();

  lcdPrintLine(1, "Blink LED 500ms");
  {
    char b[21];
    snprintf(b, sizeof(b), "Pin: GPIO%d", (int)ANALOG_BLINK_LED_PIN);
    lcdPrintLine(2, b);
  }
  lcdPrintLine(3, "Click: Back Analog");
}

inline void updateAnalogBlinkMode(unsigned long now) {
  if (now - blinkPrevMillis >= BLINK_INTERVAL_MS) {
    blinkPrevMillis = now;
    blinkLedState = (blinkLedState == LOW) ? HIGH : LOW;
    digitalWrite(ANALOG_BLINK_LED_PIN, blinkLedState);
  }
}

inline void stopAnalogBlinkMode() {
  digitalWrite(ANALOG_BLINK_LED_PIN, LOW);
}
