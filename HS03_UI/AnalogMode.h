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
//  ADS1115 -> HIỂN THỊ 12-BIT (0..4095)
//  ADS1115 single-ended trả về 0..32767 (15-bit hữu dụng)
//  Quy đổi về 12-bit kiểu ESP32: raw12 = raw16 / 8  (>>3)
// ==============================
static inline uint16_t ads1115RawTo12bit(int16_t raw16) {
  if (raw16 <= 0) return 0;
  if (raw16 >= 32767) return 4095;
  return (uint16_t)((uint16_t)raw16 >> 3);
}

// ==============================
//  BẮT ĐẦU MODE ANALOG (READ RAW)
// ==============================
void startAnalogMode() {
  appState = STATE_ANALOG;

  // Header: "Analog"
  strncpy(headerLabel, "Analog", sizeof(headerLabel));
  headerLabel[sizeof(headerLabel) - 1] = '\0';
  updateHeaderRow();

  lcdPrintLine(1, "ADS1115 Analog A1-3");
  lcdPrintLine(2, "RAW12 (0..4095)+Volt");
  lcdPrintLine(3, "Dang khoi dong...  ");

  lastAnalogUpdate = 0;
}

// ==============================
//  CẬP NHẬT MODE ANALOG (RAW, NO FILTER)
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

  char line[21];

  // A1: dòng 1
  {
    int16_t raw16 = ads1115.readADC_SingleEnded(1);     // đọc thô
    uint16_t raw12 = ads1115RawTo12bit(raw16);          // hiển thị 12-bit
    float v = ads1115.computeVolts(raw16);              // volt theo gain ADS1115 (giữ nguyên)
    snprintf(line, sizeof(line), "A1:%4u %1.3fV", (unsigned)raw12, v);
    lcdPrintLine(1, line);
  }

  // A2: dòng 2
  {
    int16_t raw16 = ads1115.readADC_SingleEnded(2);
    uint16_t raw12 = ads1115RawTo12bit(raw16);
    float v = ads1115.computeVolts(raw16);
    snprintf(line, sizeof(line), "A2:%4u %1.3fV", (unsigned)raw12, v);
    lcdPrintLine(2, line);
  }

  // A3: dòng 3
  {
    int16_t raw16 = ads1115.readADC_SingleEnded(3);
    uint16_t raw12 = ads1115RawTo12bit(raw16);
    float v = ads1115.computeVolts(raw16);
    snprintf(line, sizeof(line), "A3:%4u %1.3fV", (unsigned)raw12, v);
    lcdPrintLine(3, line);
  }
}
