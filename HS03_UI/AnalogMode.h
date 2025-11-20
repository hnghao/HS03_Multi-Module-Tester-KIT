#ifndef ANALOG_MODE_H
#define ANALOG_MODE_H

// Analog mode: đọc ADS1115 CH0, tiêu đề "ReadAnalog"
void startAnalogMode() {
  appState = STATE_ANALOG;
  lcd.clear();

  // Giữ nguyên tiêu đề như yêu cầu
  strncpy(headerLabel, "ReadAnalog", sizeof(headerLabel));
  headerLabel[sizeof(headerLabel) - 1] = '\0';
  updateHeaderRow();

  lcdPrintLine(1, "ADS1115 CH0     ");
  lcdPrintLine(2, "Raw:            ");
  lcdPrintLine(3, "Volt:           ");

  lastAnalogUpdate = 0;
}

void updateAnalogMode(unsigned long now) {
  if (now - lastAnalogUpdate < ANALOG_UPDATE_INTERVAL) return;
  lastAnalogUpdate = now;

  if (!ads1115_ok) {
    lcdPrintLine(1, "ADS1115 ERROR   ");
    lcdPrintLine(2, "Check wiring    ");
    lcdPrintLine(3, "Btn: Back       ");
    return;
  }

  int16_t raw   = ads1115.readADC_SingleEnded(0);
  float   volts = ads1115.computeVolts(raw);

  char buf[21];
  snprintf(buf, sizeof(buf), "Raw:%7d", raw);
  lcdPrintLine(2, buf);

  snprintf(buf, sizeof(buf), "Volt:%6.3f V", volts);
  lcdPrintLine(3, buf);
}

#endif
