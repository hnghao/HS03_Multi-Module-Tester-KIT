#ifndef RS485_SHTC3_MODE_H
#define RS485_SHTC3_MODE_H

#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include "SHTC3Modbus.h"

// lcd được định nghĩa trong HS03_UI.ino
extern LiquidCrystal_I2C lcd;

// Chân UART nối với RS485-SHTC3 (giống code mẫu)
#define RS485_SHTC3_RX_PIN  9   // ESP32-S3 GPIO2  - RO -> RX
#define RS485_SHTC3_TX_PIN  3   // ESP32-S3 GPIO1  - DI -> TX

// Tạo đối tượng SHTC3 (giống demo của bạn)
static SHTC3 shtc3Sensor(RS485_SHTC3_RX_PIN, RS485_SHTC3_TX_PIN, BAUD_4800, 0x01);

// Thời gian cập nhật (ms)
static const unsigned long RS485_SHTC3_UPDATE_INTERVAL = 1000;
static unsigned long       rs485SHTC3_lastUpdate       = 0;

// Xóa nội dung 1 dòng bất kỳ trên LCD (20 ký tự)
inline void lcdClearLine(uint8_t row) {
  lcd.setCursor(0, row);
  lcd.print("                    "); // 20 spaces
}

// Bắt đầu mode SHTC3
inline void startRS485SHTC3Mode() {
  // Khởi động UART/SoftwareSerial bên trong thư viện với baud 4800
  shtc3Sensor.begin(4800);
  // Nếu muốn có thể chỉnh timeout:
  // shtc3Sensor.setTimeout(1000);

  // Xóa các dòng dữ liệu (1..3), giữ nguyên header dòng 0
  for (uint8_t r = 1; r <= 3; r++) {
    lcdClearLine(r);
  }

  // In nhãn cố định
  lcd.setCursor(0, 1);
  lcd.print("Nhiet do C:");

  lcd.setCursor(0, 2);
  lcd.print("Nhiet do F:");

  lcd.setCursor(0, 3);
  lcd.print("Do am:");

  rs485SHTC3_lastUpdate = 0;  // buộc cập nhật ngay lần đầu
}

// Cập nhật đọc SHTC3 + hiển thị LCD
inline void updateRS485SHTC3Mode(unsigned long now) {
  if (now - rs485SHTC3_lastUpdate < RS485_SHTC3_UPDATE_INTERVAL) return;
  rs485SHTC3_lastUpdate = now;

  // Đọc dữ liệu bằng struct (giống code mẫu)
  dataSHTC3 data = shtc3Sensor.getData();

  // (Tuỳ bạn có muốn debug Serial nữa hay không)
  // Serial.print(F("T = "));
  // Serial.print(data.temperatureC);
  // Serial.print(F(" C, RH = "));
  // Serial.println(data.humidity);

  // DÒNG 1: Nhiet do C + ký tự độ C
  lcdClearLine(1);
  lcd.setCursor(0, 1);
  lcd.print("Nhiet do C: ");
  lcd.print(data.temperatureC, 1);  // 1 số lẻ
  lcd.write((uint8_t)223);          // ký tự độ (°) trên LCD HD44780
  lcd.print("C");

  // DÒNG 2: Nhiet do F + ký tự độ F
  lcdClearLine(2);
  lcd.setCursor(0, 2);
  lcd.print("Nhiet do F: ");
  lcd.print(data.temperatureF, 1);
  lcd.write((uint8_t)223);          // ký tự độ (°)
  lcd.print("F");

  // DÒNG 3: Do am + ký tự %
  lcdClearLine(3);
  lcd.setCursor(0, 3);
  lcd.print("Do am: ");
  lcd.print(data.humidity, 1);
  lcd.print(" %");
}

// Dừng mode SHTC3 (dọn LCD lại dòng 1..3)
inline void stopRS485SHTC3Mode() {
  for (uint8_t r = 1; r <= 3; r++) {
    lcdClearLine(r);
  }
}

#endif // RS485_SHTC3_MODE_H
