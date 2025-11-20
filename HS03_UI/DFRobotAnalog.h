#ifndef DFROBOT_ANALOG_H
#define DFROBOT_ANALOG_H

// Mode DFRobot: đọc ADS1115 CH0 cho cảm biến CO Fermion
void startDFRobotAnalogMode() {
  appState = STATE_DFROBOT_ANALOG;
  lcd.clear();

  strncpy(headerLabel, "DFRobot", sizeof(headerLabel));
  headerLabel[sizeof(headerLabel) - 1] = '\0';
  updateHeaderRow();

  lcdPrintLine(1, "CO Sensor CH0   ");
  lcdPrintLine(2, "Raw:            ");
  lcdPrintLine(3, "Volt:           ");

  lastDFRobotUpdate = 0;
}

void updateDFRobotAnalogMode(unsigned long now) {
  if (now - lastDFRobotUpdate < DFROBOT_UPDATE_INTERVAL) return;
  lastDFRobotUpdate = now;

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
