#ifndef I2C_SCAN_MODE_H
#define I2C_SCAN_MODE_H

// Quét I2C trên bus1: SDA=GPIO1, SCL=GPIO2
// BỎ QUA địa chỉ của ADS1115 (0x48..0x4B), chỉ hiển thị thiết bị khác
void startI2CScanMode() {
  appState = STATE_I2C_SCAN;
  lcd.clear();

  strncpy(headerLabel, "I2C Scan", sizeof(headerLabel));
  headerLabel[sizeof(headerLabel) - 1] = '\0';
  updateHeaderRow();

  lcdPrintLine(1, "Bus1 SDA:1 SCL:2");
  lcdPrintLine(2, "Scanning...      ");
  lcdPrintLine(3, "Please wait...   ");

  I2CScanBus.begin(I2C_SCAN_SDA_PIN, I2C_SCAN_SCL_PIN, 100000);
  delay(50);

  uint8_t foundAddrs[16];
  uint8_t foundCount = 0;

  for (uint8_t addr = 3; addr < 0x78; addr++) {
    I2CScanBus.beginTransmission(addr);
    uint8_t error = I2CScanBus.endTransmission(true);
    if (error == 0) {
      // Bỏ qua địa chỉ ADS1115 (0x48..0x4B) – không hiển thị trên LCD
      if (addr >= 0x48 && addr <= 0x4B) {
        continue;
      }

      if (foundCount < 16) {
        foundAddrs[foundCount] = addr;
      }
      foundCount++;
    }
    delay(2);
  }

  if (foundCount == 0) {
    lcdPrintLine(1, "Bus1 SDA:1 SCL:2");
    lcdPrintLine(2, "No device found  ");
    lcdPrintLine(3, "Btn: Back        ");
  } else {
    char row1[21];
    snprintf(row1, sizeof(row1), "Found:%2u device(s)", foundCount);
    lcdPrintLine(1, row1);

    char row2[21] = "";
    char row3[21] = "";

    // In tối đa 8 địa chỉ (4 địa chỉ/dòng)
    for (uint8_t i = 0; i < foundCount && i < 8; i++) {
      char tmp[10];
      snprintf(tmp, sizeof(tmp), " 0x%02X ", foundAddrs[i]);

      if (i < 4) {
        strncat(row2, tmp, sizeof(row2) - strlen(row2) - 1);
      } else {
        strncat(row3, tmp, sizeof(row3) - strlen(row3) - 1);
      }
    }

    lcdPrintLine(2, row2[0] ? row2 : "                ");
    lcdPrintLine(3, row3[0] ? row3 : "Btn: Back       ");
  }
}

// Không cần update liên tục, chỉ chờ nút Back
void updateI2CScanMode(unsigned long now) {
  (void)now;
}

#endif
