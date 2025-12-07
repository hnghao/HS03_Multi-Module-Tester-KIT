#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "max6675.h"

// ======= Cấu hình chân I2C cho LCD2004 =======
#define I2C_SDA_PIN  6
#define I2C_SCL_PIN  7

// Địa chỉ LCD2004 I2C (thường là 0x27 hoặc 0x3F)
LiquidCrystal_I2C lcd(0x27, 20, 4);  // LCD 20x4

// ======= Cấu hình chân cho MAX6675 =======
// Chỉnh lại cho khớp với cách bạn đã đấu dây
const int thermoCLK = 1;  // SCK - nối với chân SCK MAX6675
const int thermoCS  = 2;  // CS  - nối với chân CS  MAX6675
const int thermoDO  = 4;  // SO  - nối với chân SO  MAX6675

// Tạo đối tượng cảm biến MAX6675
MAX6675 thermocouple(thermoCLK, thermoCS, thermoDO);

void setup() {
  // Khởi động I2C với chân tùy chọn cho ESP32-S3
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

  // Khởi động LCD
  lcd.init();        // hoặc lcd.begin() tùy thư viện bạn dùng
  lcd.backlight();   // bật đèn nền

  // Màn hình chào
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(" MAX6675 + ESP32-S3 ");
  lcd.setCursor(0, 1);
  lcd.print("  Doc nhiet do...");
  lcd.setCursor(0, 2);
  lcd.print("  Vui long cho...");
  delay(1500);
}

void loop() {
  // Đọc nhiệt độ
  double tempC = thermocouple.readCelsius();
  double tempF = thermocouple.readFahrenheit();

  lcd.setCursor(0, 0);
  lcd.print("  MAX6675 Temperature ");

  if (isnan(tempC)) {
    // Lỗi cảm biến
    lcd.setCursor(0, 1);
    lcd.print(" Sensor error!       ");
    lcd.setCursor(0, 2);
    lcd.print(" Kiem tra day & IC   ");
    lcd.setCursor(0, 3);
    lcd.print("                     ");
  } else {
    // Dòng 2: °C
    lcd.setCursor(0, 1);
    lcd.print(" T(C): ");
    lcd.print(tempC, 2);
    lcd.print((char)223); // ký tự độ °
    lcd.print("C      "); // thêm khoảng trắng xóa ký tự cũ

    // Dòng 3: °F
    lcd.setCursor(0, 2);
    lcd.print(" T(F): ");
    lcd.print(tempF, 2);
    lcd.print((char)223); // ký tự độ °
    lcd.print("F      ");

    // Dòng 4: trạng thái
    lcd.setCursor(0, 3);
    lcd.print("  Dang cap nhat...   ");
  }

  delay(1000); // đọc mỗi 1 giây
}
