#ifndef DHT20_AHT20_MODE_H
#define DHT20_AHT20_MODE_H

#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_AHTX0.h>

// Dùng lại LCD và bus I2CScanBus đã khai báo trong HS03_UI.ino
extern LiquidCrystal_I2C lcd;
extern TwoWire I2CScanBus;
extern bool headerEnabled;   // để không bị header countdown ghi đè dòng 0

static Adafruit_AHTX0 dht20_aht20;
static bool dht20_ok = false;

static unsigned long dht20_lastRead = 0;
static const unsigned long DHT20_INTERVAL_MS = 500;

static bool dht20_savedHeaderEnabled = true;

inline void startDHT20AHT20Mode() {
  // Tắt header để giữ nguyên hiển thị dòng 0 theo chương trình bạn gửi
  dht20_savedHeaderEnabled = headerEnabled;
  headerEnabled = false;

  // Set tốc độ I2C cho bus cảm biến (GPIO1/2)
  I2CScanBus.setClock(400000UL);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("ESP32-S3 + AHT20");

  // Init AHT20/DHT20 trên I2CScanBus (GPIO1/2)
  dht20_ok = dht20_aht20.begin(&I2CScanBus);

  if (!dht20_ok) {
    lcd.setCursor(0, 1);
    lcd.print("AHT20 NOT FOUND!");
    lcd.setCursor(0, 2);
    lcd.print("Check GPIO1/2");
    // KHÔNG while(true) để bạn vẫn nhấn nút thoát về menu I2C được
    return;
  }

  lcd.setCursor(0, 1);
  lcd.print("AHT20 OK");
  delay(800);
  lcd.clear();

  dht20_lastRead = 0;
}

inline void updateDHT20AHT20Mode(unsigned long now) {
  if (!dht20_ok) return;

  if (now - dht20_lastRead < DHT20_INTERVAL_MS) return;
  dht20_lastRead = now;

  sensors_event_t humidity, temp;
  dht20_aht20.getEvent(&humidity, &temp);

  char line1[21], line2[21];
  snprintf(line1, sizeof(line1), "Temp: %5.1f C", temp.temperature);
  snprintf(line2, sizeof(line2), "Humi: %5.1f %%", humidity.relative_humidity);

  lcd.setCursor(0, 0);
  lcd.print("AHT20 Sensor Read  ");

  lcd.setCursor(0, 1);
  lcd.print(line1);
  lcd.print("   "); // xoá dư ký tự cũ nếu có

  lcd.setCursor(0, 2);
  lcd.print(line2);
  lcd.print("   ");

  lcd.setCursor(0, 3);
  lcd.print("I2C: AHT + LCD");
}

inline void stopDHT20AHT20Mode() {
  // Trả I2C bus về 100kHz (mặc định của project)
  I2CScanBus.setClock(100000UL);

  // Bật lại header cho các menu/mode khác
  headerEnabled = dht20_savedHeaderEnabled;
}

#endif
