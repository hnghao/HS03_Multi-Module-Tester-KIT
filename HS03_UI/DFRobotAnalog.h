#ifndef DFROBOT_ANALOG_H
#define DFROBOT_ANALOG_H

#include <Adafruit_ADS1X15.h>

// Các biến toàn cục đã được khai báo ở .ino
extern LiquidCrystal_I2C lcd;
extern AppState appState;
extern char headerLabel[16];

extern unsigned long lastDFRobotUpdate;
extern const unsigned long DFROBOT_UPDATE_INTERVAL;

extern Adafruit_ADS1115 ads1115;
extern bool ads1115_ok;

extern void updateHeaderRow();
extern void lcdPrintLine(uint8_t row, const char *text);

// Giả sử ADC 12-bit của ESP32-S3 dùng tham chiếu ~3.3V
// const float ESP32_ADC_REF_VOLT = 3.3f;

// Mode DFRobot: đọc 3 kênh A1, A2, A3 và hiển thị dạng 12-bit tương đương ESP32
void startDFRobotAnalogMode() {
  appState = STATE_DFROBOT_ANALOG;
  lcd.clear();

  // Header "DFRobot" + countdown (dòng 0 do updateHeaderRow() lo)
  strncpy(headerLabel, "DFRobot", sizeof(headerLabel));
  headerLabel[sizeof(headerLabel) - 1] = '\0';
  updateHeaderRow();

  // Khung hiển thị 3 kênh
  lcdPrintLine(1, "A1:              ");
  lcdPrintLine(2, "A2:              ");
  lcdPrintLine(3, "A3:              ");

  lastDFRobotUpdate = 0;
}

void updateDFRobotAnalogMode(unsigned long now) {
  if (now - lastDFRobotUpdate < DFROBOT_UPDATE_INTERVAL) return;
  lastDFRobotUpdate = now;

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

  float v1 = ads1115.computeVolts(raw1_ads);
  float v2 = ads1115.computeVolts(raw2_ads);
  float v3 = ads1115.computeVolts(raw3_ads);

  // Quy đổi về giá trị 12-bit tương đương ADC ESP32-S3 (0..4095 @ 0..3.3V)
  auto volts_to_raw12 = [](float v) -> uint16_t {
    if (v < 0.0f) v = 0.0f;
    if (v > ESP32_ADC_REF_VOLT) v = ESP32_ADC_REF_VOLT;
    float ratio = v / ESP32_ADC_REF_VOLT;
    uint16_t raw12 = (uint16_t)(ratio * 4095.0f + 0.5f);
    return raw12;
  };

  uint16_t raw1_12 = volts_to_raw12(v1);
  uint16_t raw2_12 = volts_to_raw12(v2);
  uint16_t raw3_12 = volts_to_raw12(v3);

  char line[21];

  // Dòng 1: A1 -> 12-bit + Volt
  // Ví dụ: "A1 4095 3.300V"
  snprintf(line, sizeof(line), "A1 %4u %5.3fV", raw1_12, v1);
  lcdPrintLine(1, line);

  // Dòng 2: A2
  snprintf(line, sizeof(line), "A2 %4u %5.3fV", raw2_12, v2);
  lcdPrintLine(2, line);

  // Dòng 3: A3
  snprintf(line, sizeof(line), "A3 %4u %5.3fV", raw3_12, v3);
  lcdPrintLine(3, line);
}

#endif
