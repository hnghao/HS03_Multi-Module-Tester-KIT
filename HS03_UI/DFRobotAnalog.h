#pragma once
#include <Arduino.h>
#include <Adafruit_ADS1X15.h>
#include <LiquidCrystal_I2C.h>

// Các biến dùng chung được định nghĩa trong HS03_UI.ino
extern LiquidCrystal_I2C lcd;
extern Adafruit_ADS1115 ads1115;
extern bool ads1115_ok;

extern AppState appState;
extern const unsigned long DFROBOT_UPDATE_INTERVAL;
extern unsigned long lastDFRobotUpdate;

extern char headerLabel[16];
void updateHeaderRow();
void lcdPrintLine(uint8_t row, const char *text);

// ==============================
//  LỌC NHIỄU EMA CHO 4 KÊNH (DFRobot)
// ==============================
static bool  dfFilterInit      = false;
static float dfFilteredRaw[4]  = {0, 0, 0, 0};
static const float DF_FILTER_ALPHA = 0.15f;

// ==============================
//  BẮT ĐẦU MODE DFRobot Analog
// ==============================
void startDFRobotAnalogMode() {
  appState = STATE_DFROBOT_ANALOG;

  strncpy(headerLabel, "DFRobot", sizeof(headerLabel));
  headerLabel[sizeof(headerLabel) - 1] = '\0';
  updateHeaderRow();

  lcdPrintLine(1, "DFRobot Analog A1-3");
  lcdPrintLine(2, "Gia tri RAW + Volt ");
  lcdPrintLine(3, "Dang khoi dong...  ");

  dfFilterInit       = false;
  lastDFRobotUpdate  = 0;
}

// ==============================
//  CẬP NHẬT MODE DFRobot Analog
// ==============================
void updateDFRobotAnalogMode(unsigned long now) {
  if (!ads1115_ok) {
    lcdPrintLine(1, "ADS1115 khong OK   ");
    lcdPrintLine(2, "Kiem tra day & I2C ");
    lcdPrintLine(3, "Nhan Back de thoat ");
    return;
  }

  if (now - lastDFRobotUpdate < DFROBOT_UPDATE_INTERVAL) return;
  lastDFRobotUpdate = now;

  // Đọc & lọc cho 4 kênh, nhưng chỉ in A1..A3
  for (uint8_t ch = 0; ch < 4; ch++) {
    int16_t raw = ads1115.readADC_SingleEnded(ch);

    if (!dfFilterInit) {
      dfFilteredRaw[ch] = raw;
    } else {
      dfFilteredRaw[ch] =
        DF_FILTER_ALPHA * raw +
        (1.0f - DF_FILTER_ALPHA) * dfFilteredRaw[ch];
    }
  }
  dfFilterInit = true;

  char line[21];

  // A1
  {
    int16_t raw = (int16_t)dfFilteredRaw[1];
    float   v   = ads1115.computeVolts(raw);
    snprintf(line, sizeof(line), "A1:%6d %1.3fV", raw, v);
    lcdPrintLine(1, line);
  }

  // A2
  {
    int16_t raw = (int16_t)dfFilteredRaw[2];
    float   v   = ads1115.computeVolts(raw);
    snprintf(line, sizeof(line), "A2:%6d %1.3fV", raw, v);
    lcdPrintLine(2, line);
  }

  // A3
  {
    int16_t raw = (int16_t)dfFilteredRaw[3];
    float   v   = ads1115.computeVolts(raw);
    snprintf(line, sizeof(line), "A3:%6d %1.3fV", raw, v);
    lcdPrintLine(3, line);
  }
}
