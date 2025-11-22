#ifndef ANALOG_MODE_H
#define ANALOG_MODE_H

#include <Adafruit_ADS1X15.h>

// Các biến/hàm đã khai báo trong .ino
extern LiquidCrystal_I2C lcd;
extern AppState appState;
extern char headerLabel[16];

extern unsigned long lastAnalogUpdate;
extern const unsigned long ANALOG_UPDATE_INTERVAL;

extern Adafruit_ADS1115 ads1115;
extern bool ads1115_ok;

extern void updateHeaderRow();
extern void lcdPrintLine(uint8_t row, const char *text);

// Giả sử ADC 12-bit ESP32-S3 tham chiếu khoảng 3.3V
const float ESP32_ADC_REF_VOLT = 3.3f;

// Hàm convert Volt -> giá trị 12-bit (0..4095)
static uint16_t voltsToRaw12(float v) {
  if (v < 0.0f) v = 0.0f;
  if (v > ESP32_ADC_REF_VOLT) v = ESP32_ADC_REF_VOLT;
  float ratio = v / ESP32_ADC_REF_VOLT;
  uint16_t raw12 = (uint16_t)(ratio * 4095.0f + 0.5f);
  return raw12;
}

// =======================
// Bắt đầu chế độ ReadAnalog
// =======================
void startAnalogMode() {
  appState = STATE_ANALOG;
  lcd.clear();

  // Tiêu đề giữ nguyên "ReadAnalog"
  strncpy(headerLabel, "ReadAnalog", sizeof(headerLabel));
  headerLabel[sizeof(headerLabel) - 1] = '\0';
  updateHeaderRow();

  // Khung hiển thị 3 kênh
  lcdPrintLine(1, "A1:              ");
  lcdPrintLine(2, "A2:              ");
  lcdPrintLine(3, "A3:              ");

  lastAnalogUpdate = 0;
}

// =======================
// Cập nhật giá trị ReadAnalog
// =======================
void updateAnalogMode(unsigned long now) {
  if (now - lastAnalogUpdate < ANALOG_UPDATE_INTERVAL) return;
  lastAnalogUpdate = now;

  if (!ads1115_ok) {
    lcdPrintLine(1, "ADS1115 ERROR    ");
    lcdPrintLine(2, "Check wiring     ");
    lcdPrintLine(3, "Btn: Back        ");
    return;
  }

  // Đọc 3 kênh single-ended từ ADS1115:
  // A1 -> CH1, A2 -> CH2, A3 -> CH3
  int16_t raw1_ads = ads1115.readADC_SingleEnded(1);  // A1
  int16_t raw2_ads = ads1115.readADC_SingleEnded(2);  // A2
  int16_t raw3_ads = ads1115.readADC_SingleEnded(3);  // A3

  // Đổi sang Volt
  float v1 = ads1115.computeVolts(raw1_ads);
  float v2 = ads1115.computeVolts(raw2_ads);
  float v3 = ads1115.computeVolts(raw3_ads);

  // Quy đổi sang giá trị 12-bit tương đương ADC ESP32-S3
  uint16_t a1_12 = voltsToRaw12(v1);
  uint16_t a2_12 = voltsToRaw12(v2);
  uint16_t a3_12 = voltsToRaw12(v3);

  char line[21];

  // Dòng 1: A1 (0..4095)
  snprintf(line, sizeof(line), "A1:%5u", a1_12);
  lcdPrintLine(1, line);

  // Dòng 2: A2
  snprintf(line, sizeof(line), "A2:%5u", a2_12);
  lcdPrintLine(2, line);

  // Dòng 3: A3
  snprintf(line, sizeof(line), "A3:%5u", a3_12);
  lcdPrintLine(3, line);
}

#endif
