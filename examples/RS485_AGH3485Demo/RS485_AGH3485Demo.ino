#include <SoftwareSerial.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// RS485: RX, TX (giữ nguyên như chương trình bạn đang dùng)
SoftwareSerial rs485(9, 3);  // RX, TX

// LCD2004 I2C
#define I2C_SDA 6
#define I2C_SCL 7
LiquidCrystal_I2C lcd(0x27, 20, 4);  // Địa chỉ 0x27, LCD 20x4

// Lệnh đọc modbus: 01 03 00 00 00 02 C4 0B
byte readCmd[] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x02, 0xC4, 0x0B};

void setup() {
  // Khởi tạo I2C trên ESP32-S3 với chân tự chọn
  Wire.begin(I2C_SDA, I2C_SCL);

  // Khởi tạo LCD
  lcd.init();
  lcd.backlight();
  lcd.clear();

  lcd.setCursor(0, 0);
  lcd.print("AGH3485 RS485");
  lcd.setCursor(0, 1);
  lcd.print("Dang khoi dong...");
  
  // Khởi tạo RS485
  rs485.begin(9600);

  delay(1000);
  lcd.clear();
}

void loop() {
  // ===== Gửi lệnh truy vấn =====
  rs485.write(readCmd, sizeof(readCmd));
  rs485.flush();   // Đảm bảo gửi xong

  delay(30);       // Đợi cảm biến trả dữ liệu (rất quan trọng)

  // ===== Nhận dữ liệu =====
  if (rs485.available() >= 9) {
    byte buf[9];
    rs485.readBytes(buf, 9);

    // Kiểm tra frame 01 03 04 ...
    if (buf[0] == 0x01 && buf[1] == 0x03 && buf[2] == 0x04) {
      int humidity_raw = (buf[3] << 8) | buf[4];
      int16_t temp_raw = (buf[5] << 8) | buf[6];

      float humidity = humidity_raw / 10.0;
      float temperature = temp_raw / 10.0;

      // Hiển thị lên LCD
      lcd.setCursor(0, 1);
      lcd.print("Hum: ");
      lcd.print(humidity, 1);   // 1 số lẻ
      lcd.print(" %RH    ");    // thêm khoảng trắng xoá ký tự thừa

      lcd.setCursor(0, 2);
      lcd.print("Temp: ");
      lcd.print(temperature, 1);
      lcd.print(" C      ");

      // Xoá dòng báo lỗi (nếu có) ở lần đọc trước
      lcd.setCursor(0, 3);
      lcd.print("                    ");
    } else {
      // Frame lỗi -> báo trên LCD
      lcd.setCursor(0, 3);
      lcd.print("Frame loi / lech!");
    }
  } else {
    // Nếu chưa đủ dữ liệu, có thể hiện trạng thái đang chờ (tuỳ chọn)
    // lcd.setCursor(0, 2);
    // lcd.print("Dang cho du lieu...");
  }

  delay(1000);  // Đọc mỗi 1 giây
}
