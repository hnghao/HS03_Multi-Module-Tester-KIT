#pragma once
#include <Arduino.h>
#include <Adafruit_ADS1X15.h>
#include <LiquidCrystal_I2C.h>

// Các biến dùng chung được định nghĩa trong HS03_UI.ino
extern LiquidCrystal_I2C lcd;
extern Adafruit_ADS1115 ads1115;
extern bool ads1115_ok;

extern AppState appState;
extern const unsigned long ANALOG_UPDATE_INTERVAL;
extern unsigned long lastAnalogUpdate;

extern char headerLabel[16];
void updateHeaderRow();
void lcdPrintLine(uint8_t row, const char *text);

// ==============================
//  LỌC NHIỄU EMA CHO 4 KÊNH A0..A3
// ==============================
static bool  analogFilterInit      = false;
static float analogFilteredRaw[4]  = {0, 0, 0, 0};   // lưu giá trị ADC đã lọc
// alpha nhỏ -> mượt hơn, chậm hơn
static const float ANALOG_FILTER_ALPHA = 0.15f;

// ==============================
//  BẮT ĐẦU MODE ANALOG
// ==============================
void startAnalogMode() {
  appState = STATE_ANALOG;

  // Header: "Analog"
  strncpy(headerLabel, "Analog", sizeof(headerLabel));
  headerLabel[sizeof(headerLabel) - 1] = '\0';
  updateHeaderRow();

  lcdPrintLine(1, "ADS1115 Analog A1-3");
  lcdPrintLine(2, "Gia tri RAW + Volt ");
  lcdPrintLine(3, "Dang khoi dong...  ");

  analogFilterInit   = false;
  lastAnalogUpdate   = 0;
}

// ==============================
//  CẬP NHẬT MODE ANALOG
// ==============================
void updateAnalogMode(unsigned long now) {
  // Không có ADS1115 thì báo lỗi, không đọc
  if (!ads1115_ok) {
    lcdPrintLine(1, "ADS1115 khong tim thay");
    lcdPrintLine(2, "Kiem tra day & dia chi");
    lcdPrintLine(3, "Nhan Back de thoat   ");
    return;
  }

  // Giới hạn tốc độ update theo ANALOG_UPDATE_INTERVAL
  if (now - lastAnalogUpdate < ANALOG_UPDATE_INTERVAL) return;
  lastAnalogUpdate = now;

  // Đọc & lọc cho 4 kênh, nhưng chỉ in ra A1..A3
  for (uint8_t ch = 0; ch < 4; ch++) {
    int16_t raw = ads1115.readADC_SingleEnded(ch);

    if (!analogFilterInit) {
      analogFilteredRaw[ch] = raw;
    } else {
      analogFilteredRaw[ch] =
        ANALOG_FILTER_ALPHA * raw +
        (1.0f - ANALOG_FILTER_ALPHA) * analogFilteredRaw[ch];
    }
  }
  analogFilterInit = true;

  char line[21];

  // A1: dùng dòng 1
  {
    int16_t raw = (int16_t)analogFilteredRaw[1];
    float   v   = ads1115.computeVolts(raw);
    snprintf(line, sizeof(line), "A1:%6d %1.3fV", raw, v);
    lcdPrintLine(1, line);
  }

  // A2: dùng dòng 2
  {
    int16_t raw = (int16_t)analogFilteredRaw[2];
    float   v   = ads1115.computeVolts(raw);
    snprintf(line, sizeof(line), "A2:%6d %1.3fV", raw, v);
    lcdPrintLine(2, line);
  }

  // A3: dùng dòng 3
  {
    int16_t raw = (int16_t)analogFilteredRaw[3];
    float   v   = ads1115.computeVolts(raw);
    snprintf(line, sizeof(line), "A3:%6d %1.3fV", raw, v);
    lcdPrintLine(3, line);
  }
}
