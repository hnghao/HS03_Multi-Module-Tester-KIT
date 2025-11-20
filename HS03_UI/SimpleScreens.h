#ifndef SIMPLE_SCREENS_H
#define SIMPLE_SCREENS_H

// Màn hình đơn giản cho các mục chưa triển khai
void showMainFunctionScreen(int index) {
  appState = STATE_SIMPLE_SCREEN;
  lcd.clear();

  strncpy(headerLabel, mainMenuItems[index], sizeof(headerLabel));
  headerLabel[sizeof(headerLabel) - 1] = '\0';
  updateHeaderRow();

  char line1[21];
  snprintf(line1, sizeof(line1), "%2d.%s", index + 1, mainMenuItems[index]);
  lcdPrintLine(1, line1);
  lcdPrintLine(2, "Feature pending ");
  lcdPrintLine(3, "Btn: Back       ");
}

// Màn hình Test VL53L1X (stub)
void showVL53L1XTestScreen() {
  appState = STATE_SIMPLE_SCREEN;
  lcd.clear();

  strncpy(headerLabel, "VL53L1X Test", sizeof(headerLabel));
  headerLabel[sizeof(headerLabel) - 1] = '\0';
  updateHeaderRow();

  lcdPrintLine(1, "I2C VL53L1X     ");
  lcdPrintLine(2, "Not implemented ");
  lcdPrintLine(3, "Btn: Back       ");
}

#endif
