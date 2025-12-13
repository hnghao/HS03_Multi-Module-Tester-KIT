#include <Wire.h>
#include <Adafruit_AHTX0.h>
#include <LiquidCrystal_PCF8574.h>

// ================== PIN MAP (theo yêu cầu của bạn) ==================
#define LCD_SDA   6
#define LCD_SCL   7

#define AHT_SDA   1
#define AHT_SCL   2

// ================== I2C CONFIG ==================
#define I2C_FREQ      400000UL
#define LCD_ADDR      0x27     // đổi thành 0x3F nếu LCD bạn dùng địa chỉ khác

// Tạo 2 bus I2C riêng cho ESP32-S3
TwoWire I2C_AHT(0);
TwoWire I2C_LCD(1);

// ================== DEVICES ==================
Adafruit_AHTX0 aht;
LiquidCrystal_PCF8574 lcd(LCD_ADDR);

void setup() {
  // Khởi tạo 2 bus I2C theo đúng chân bạn yêu cầu
  I2C_AHT.begin(AHT_SDA, AHT_SCL, I2C_FREQ);
  I2C_LCD.begin(LCD_SDA, LCD_SCL, I2C_FREQ);

  // Init LCD (20x4) trên bus I2C_LCD
  lcd.begin(20, 4, I2C_LCD);
  lcd.setBacklight(255);
  lcd.clear();

  lcd.setCursor(0, 0);
  lcd.print("ESP32-S3 + AHT20");

  // Init AHT20 trên bus I2C_AHT
  if (!aht.begin(&I2C_AHT)) {
    lcd.setCursor(0, 1);
    lcd.print("AHT20 NOT FOUND!");
    lcd.setCursor(0, 2);
    lcd.print("Check GPIO1/2");
    while (true) delay(100);
  }

  lcd.setCursor(0, 1);
  lcd.print("AHT20 OK");
  delay(800);
  lcd.clear();
}

void loop() {
  sensors_event_t humidity, temp;
  aht.getEvent(&humidity, &temp);

  char line1[21], line2[21];

  snprintf(line1, sizeof(line1), "Temp: %5.1f C", temp.temperature);
  snprintf(line2, sizeof(line2), "Humi: %5.1f %%", humidity.relative_humidity);

  lcd.setCursor(0, 0);
  lcd.print("AHT20 Sensor Read  ");

  lcd.setCursor(0, 1);
  lcd.print(line1);
  lcd.print("   "); // xoa du ky tu cu neu co

  lcd.setCursor(0, 2);
  lcd.print(line2);
  lcd.print("   ");

  lcd.setCursor(0, 3);
  lcd.print("I2C: AHT + LCD");

  delay(500);
}
