#ifndef MAX6675_MODE_H
#define MAX6675_MODE_H

#include <Arduino.h>
#include "max6675.h"

// Dùng lại LCD & hàm in dòng từ chương trình chính
class LiquidCrystal_I2C;
extern LiquidCrystal_I2C lcd;
extern void lcdPrintLine(uint8_t row, const char* text);

// =========================
// Cấu hình chân MAX6675
// =========================
// ĐÚNG với wiring bạn đã test OK:
//  CLK -> GPIO1, CS -> GPIO2, DO -> GPIO4
#define MAX6675_CLK_PIN  1
#define MAX6675_CS_PIN   2
#define MAX6675_DO_PIN   4

// Object MAX6675 được định nghĩa trong HS03_UI.ino
// Ví dụ ở HS03_UI.ino bạn có dòng:
//   MAX6675 max6675(MAX6675_CLK_PIN, MAX6675_CS_PIN, MAX6675_DO_PIN);
extern MAX6675 max6675;

// =========================
// MODE MAX6675
// =========================

// Gọi khi VỪA vào chức năng "Max6675"
inline void startMAX6675Mode() {
  // RẤT QUAN TRỌNG: giành lại quyền control chân cho MAX6675
  pinMode(MAX6675_CLK_PIN, OUTPUT);
  pinMode(MAX6675_CS_PIN,  OUTPUT);
  pinMode(MAX6675_DO_PIN,  INPUT);
  digitalWrite(MAX6675_CS_PIN, HIGH);  // CS idle ở mức HIGH

  // Cho cảm biến ổn định một chút
  delay(50);

  // Chuẩn bị giao diện LCD
  lcdPrintLine(1, "T(C): ----.-");
  lcdPrintLine(2, "T(F): ----.-");
  lcdPrintLine(3, "Nhan Back de quay lai");
}

// Được gọi trong loop() khi appState == STATE_MAX6675
inline void updateMAX6675Mode(unsigned long now) {
  static unsigned long lastRead = 0;
  const unsigned long READ_INTERVAL = 1000; // đọc mỗi 1 giây

  if (now - lastRead < READ_INTERVAL) return;
  lastRead = now;

  double tempC = max6675.readCelsius();

  // Nếu thermocouple hở / lỗi, nhiều lib trả NAN
  if (isnan(tempC)) {
    lcdPrintLine(1, "Sensor error!");
    lcdPrintLine(2, "Kiem tra day & IC");
    // Dòng 3 giữ nguyên "Nhan Back de quay lai"
    return;
  }

  double tempF = max6675.readFahrenheit();

  char buf[21];

  // Dòng 1: T(C)
  snprintf(buf, sizeof(buf), "T(C): %6.2f%cC", tempC, (char)223);
  lcdPrintLine(1, buf);

  // Dòng 2: T(F)
  snprintf(buf, sizeof(buf), "T(F): %6.2f%cF", tempF, (char)223);
  lcdPrintLine(2, buf);

  // Dòng 3: không đụng vào, luôn là "Nhan Back de quay lai"
}

inline void stopMAX6675Mode() {
  // Không cần làm gì đặc biệt.
  // Khi quay về MENU, Display.h sẽ vẽ lại LCD, các mode khác
  // sẽ tự cấu hình lại chân nếu cần.
}

#endif // MAX6675_MODE_H
