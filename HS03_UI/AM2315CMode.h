#ifndef AM2315C_MODE_H
#define AM2315C_MODE_H

#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <AM2315C.h>

// Dùng lại LCD và bus I2CScanBus đã khai báo trong HS03_UI.ino
extern LiquidCrystal_I2C lcd;
extern TwoWire I2CScanBus;

// Đối tượng cảm biến AM2315C dùng chung bus 1 (I2CScanBus)
static AM2315C am2315c(&I2CScanBus);

// Cờ & thời gian đọc
static bool         am2315cReady          = false;
static unsigned long am2315cLastRead      = 0;
static const unsigned long AM2315C_INTERVAL = 1000; // ms

// Khởi động mode AM2315C (gọi khi chọn mục AM2315C trong menu I2C)
inline void startAM2315CMode() {
  // Thử khởi động cảm biến, chỉ cần gọi 1 lần
  am2315cReady = am2315c.begin();

  // Xóa các dòng nội dung (dòng 0 là header do hệ thống lo)
  lcd.setCursor(0, 1);
  lcd.print("Nhiet do:      C ");
  lcd.setCursor(0, 2);
  lcd.print("Do am  :      %  ");
  lcd.setCursor(0, 3);

  if (am2315cReady) {
    lcd.print("Trang thai: OK   ");
  } else {
    lcd.print("Sensor khong thay");
  }

  // Để lần update đầu tiên đọc ngay lập tức
  am2315cLastRead = 0;
}

// Hàm cập nhật định kỳ, được gọi trong loop() khi appState == STATE_AM2315C
inline void updateAM2315CMode(unsigned long now) {
  if (!am2315cReady) {
    // Nếu không tìm thấy cảm biến, chỉ hiện ERR/không đọc gì thêm
    return;
  }

  if (now - am2315cLastRead < AM2315C_INTERVAL) return;
  am2315cLastRead = now;

  int8_t status = am2315c.read();  // đọc cảm biến (thư viện xử lý thời gian nội bộ)

  if (status == AM2315C_OK) {
    float temperature = am2315c.getTemperature();
    float humidity    = am2315c.getHumidity();

    // Nhiệt độ tại dòng 1, cột 10
    lcd.setCursor(10, 1);
    lcd.print("     ");
    lcd.setCursor(10, 1);
    lcd.print(temperature, 1);

    // Độ ẩm tại dòng 2, cột 10
    lcd.setCursor(10, 2);
    lcd.print("     ");
    lcd.setCursor(10, 2);
    lcd.print(humidity, 1);

    // Trạng thái tại dòng 3
    lcd.setCursor(12, 3);
    lcd.print("OK  ");
  } else {
    // Có lỗi khi đọc sensor
    lcd.setCursor(12, 3);
    lcd.print("ERR ");
  }
}

// Nếu sau này bạn muốn dọn dẹp gì đó thì thêm vào đây; hiện tại không cần
inline void stopAM2315CMode() {
  // Không cần làm gì đặc biệt
}

#endif
