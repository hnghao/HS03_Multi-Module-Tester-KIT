#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <AM2315C.h>

// --------- Cấu hình chân I2C ---------
#define LCD_SDA   6
#define LCD_SCL   7

#define AM_SDA    1
#define AM_SCL    2

// Địa chỉ LCD2004 (thường là 0x27 hoặc 0x3F, chỉnh lại nếu khác)
#define LCD_ADDR  0x27

// --------- Khai báo đối tượng I2C & thiết bị ---------
LiquidCrystal_I2C lcd(LCD_ADDR, 20, 4);  // 20 cột, 4 hàng

// Tạo bus I2C thứ 2 cho cảm biến (bus ID = 1)
TwoWire I2C_AM(1);

// Truyền con trỏ bus I2C thứ 2 vào thư viện AM2315C
AM2315C am2315c(&I2C_AM);

// --------- Biến thời gian đọc cảm biến ---------
unsigned long lastRead = 0;
const unsigned long READ_INTERVAL = 1000;  // ms, tối thiểu 1000ms giữa 2 lần đo

void setup()
{
  // (Tùy chọn) Bật Serial để debug nếu cần
  Serial.begin(115200);

  // Khởi tạo I2C cho LCD trên Wire (bus 0)
  Wire.begin(LCD_SDA, LCD_SCL);

  // Khởi tạo I2C cho cảm biến trên bus 1
  I2C_AM.begin(AM_SDA, AM_SCL);

  // Khởi tạo LCD
  lcd.init();
  lcd.backlight();
  lcd.clear();

  lcd.setCursor(0, 0);
  lcd.print("AM2315C + LCD2004");
  lcd.setCursor(0, 1);
  lcd.print("Dang khoi dong...");

  // Khởi tạo cảm biến AM2315C
  if (!am2315c.begin())
  {
    lcd.setCursor(0, 2);
    lcd.print("Sensor khong thay ");
    Serial.println("Khong tim thay cam bien AM2315C!");
    // Nếu muốn đứng luôn tại đây:
    while (1) {
      delay(1000);
    }
  }

  lcd.setCursor(0, 2);
  lcd.print("Sensor OK         ");
  Serial.println("AM2315C da san sang!");

  delay(1000);
  lcd.clear();

  // Khung hiển thị cố định
  lcd.setCursor(0, 0);
  lcd.print("Nhiet do:      C");
  lcd.setCursor(0, 1);
  lcd.print("Do am   :      %");
  lcd.setCursor(0, 3);
  lcd.print("Trang thai:      ");
}

void loop()
{
  unsigned long now = millis();

  if (now - lastRead >= READ_INTERVAL)
  {
    lastRead = now;

    int8_t status = am2315c.read();  // đọc cảm biến, blocking ~80ms

    if (status == AM2315C_OK)
    {
      float temperature = am2315c.getTemperature();
      float humidity    = am2315c.getHumidity();

      // In ra Serial (nếu cần)
      Serial.print("T = ");
      Serial.print(temperature, 1);
      Serial.print(" *C,  H = ");
      Serial.print(humidity, 1);
      Serial.println(" %");

      // Hiển thị lên LCD
      // Xóa vị trí cũ rồi ghi lại cho gọn
      lcd.setCursor(10, 0);
      lcd.print("     ");
      lcd.setCursor(10, 0);
      lcd.print(temperature, 1);   // 1 chữ số lẻ

      lcd.setCursor(10, 1);
      lcd.print("     ");
      lcd.setCursor(10, 1);
      lcd.print(humidity, 1);      // 1 chữ số lẻ

      lcd.setCursor(11, 3);
      lcd.print("OK   ");
    }
    else
    {
      // Có lỗi khi đọc sensor
      Serial.print("Loi doc AM2315C, status = ");
      Serial.println(status);

      lcd.setCursor(11, 3);
      lcd.print("ERR  ");

      // (Tùy chọn) có thể hiển thị mã lỗi ra LCD
      // lcd.setCursor(0, 2);
      // lcd.print("Err code:      ");
      // lcd.setCursor(10, 2);
      // lcd.print(status);
    }
  }

  // Có thể làm việc khác ở đây...
}
