#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// ===== LCD2004 I2C pins on ESP32-S3 =====
static const uint8_t LCD_SDA = 6;
static const uint8_t LCD_SCL = 7;

// ===== URM37 Trig/Echo(PWM) pins on ESP32-S3 =====
static const uint8_t PIN_URECHO = 41;   // ECHO/PWM output từ URM37 (nhớ hạ mức về 3.3V)
static const uint8_t PIN_URTRIG = 42;   // TRIG/COMP của URM37

// ===== URM37 timing (theo code mẫu DFRobot) =====
static const uint32_t INVALID_LIMIT_US = 50000;  // >= 50ms -> invalid
static const uint32_t PULSE_TIMEOUT_US = 60000;  // timeout pulseIn (nên > 50ms)
static const uint32_t READ_INTERVAL_MS = 200;

// ===== LCD address: tự dò 0x27 / 0x3F =====
LiquidCrystal_I2C lcd_027(0x27, 20, 4);
LiquidCrystal_I2C lcd_03F(0x3F, 20, 4);
LiquidCrystal_I2C* lcd = &lcd_027;

static bool i2cAddressExists(uint8_t addr) {
  Wire.beginTransmission(addr);
  return (Wire.endTransmission() == 0);
}

static void lcdPrintLine(uint8_t row, const String& text) {
  String s = text;
  if (s.length() > 20) s = s.substring(0, 20);
  while (s.length() < 20) s += ' ';
  lcd->setCursor(0, row);
  lcd->print(s);
}

static inline void urm37Trigger() {
  // TRIG bình thường HIGH, kéo LOW 1 nhịp để kích đo, rồi trả lại HIGH
  digitalWrite(PIN_URTRIG, HIGH);
  delayMicroseconds(5);

  digitalWrite(PIN_URTRIG, LOW);
  delayMicroseconds(20);
  digitalWrite(PIN_URTRIG, HIGH);
}

static bool urm37ReadDistanceCm(uint16_t &distanceCm, uint32_t &lowTimeUs) {
  urm37Trigger();

  lowTimeUs = pulseIn(PIN_URECHO, LOW, PULSE_TIMEOUT_US);

  // pulseIn timeout -> 0
  if (lowTimeUs == 0 || lowTimeUs >= INVALID_LIMIT_US) return false;

  // Mỗi 50us = 1cm (theo mẫu)
  distanceCm = (uint16_t)(lowTimeUs / 50UL);
  return true;
}

void setup() {
  // I2C for LCD
  Wire.begin(LCD_SDA, LCD_SCL);

  // Chọn địa chỉ LCD phổ biến
  if (i2cAddressExists(0x27)) lcd = &lcd_027;
  else if (i2cAddressExists(0x3F)) lcd = &lcd_03F;
  else lcd = &lcd_027; // fallback (nếu không dò được)

  lcd->init();
  lcd->backlight();

  // URM37 pins
  pinMode(PIN_URTRIG, OUTPUT);
  digitalWrite(PIN_URTRIG, HIGH);
  pinMode(PIN_URECHO, INPUT);

  lcdPrintLine(0, "URM37 PWM Distance");
  lcdPrintLine(1, "Init the sensor...");
  lcdPrintLine(2, "LCD SDA=GPIO6");
  lcdPrintLine(3, "LCD SCL=GPIO7");
  delay(800);
}

void loop() {
  static uint32_t lastMs = 0;
  if (millis() - lastMs < READ_INTERVAL_MS) return;
  lastMs = millis();

  uint16_t cm = 0;
  uint32_t lowUs = 0;

  bool ok = urm37ReadDistanceCm(cm, lowUs);

  lcdPrintLine(0, "URM37 PWM Distance");

  if (ok) {
    String line1 = "Distance: " + String(cm) + " cm";
    lcdPrintLine(1, line1);

    lcdPrintLine(2, "Status: OK");
    // Dòng 3 hiển thị thời gian xung để debug nhanh (không dùng Serial)
    lcdPrintLine(3, "Low: " + String(lowUs) + " us");
  } else {
    lcdPrintLine(1, "Distance: ---- cm");
    lcdPrintLine(2, "Status: Invalid");
    lcdPrintLine(3, "Check wiring/level");
  }
}
