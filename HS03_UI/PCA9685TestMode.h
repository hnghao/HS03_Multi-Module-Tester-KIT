#ifndef PCA9685_TEST_MODE_H
#define PCA9685_TEST_MODE_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Bus I2C dùng chung cho I2C Scan + ADS1115 (khai báo trong HS03_UI.ino)
extern TwoWire I2CScanBus;

// LCD & hàm in dòng đã có sẵn trong project
class LiquidCrystal_I2C;
extern LiquidCrystal_I2C lcd;
void lcdPrintLine(uint8_t row, const char *text);

// Hàm thoát về menu I2C (định nghĩa trong HS03_UI.ino)
void exitPCA9685ToI2CMenu();

// Đối tượng PCA9685 dùng bus I2CScanBus, địa chỉ mặc định 0x40
static Adafruit_PWMServoDriver pca9685(0x40, I2CScanBus);

// Cấu hình servo
static const uint16_t PCA_FREQUENCY_HZ = 50;   // 50 Hz cho servo
static const uint16_t SERVO_MIN_US     = 500;  // ~0.5ms  ~ 0°
static const uint16_t SERVO_MAX_US     = 2500; // ~2.5ms ~180°

static int16_t pcaCurrentAngle = 90;   // góc hiện tại (0..180)
static bool    pcaInitialized  = false;

// Double-click detection
static bool           pcaClickPending    = false;
static unsigned long  pcaLastClickTime   = 0;
static const unsigned long PCA_DOUBLE_MS = 400;  // ms

// Đổi micro-giây sang tick 0..4095 của PCA9685
static uint16_t pcaUsToTicks(uint16_t us) {
  const uint32_t period_us = 1000000UL / PCA_FREQUENCY_HZ; // ví dụ 20000µs
  uint32_t ticks = (uint32_t)us * 4096UL / period_us;
  if (ticks > 4095) ticks = 4095;
  return (uint16_t)ticks;
}

// Ghi 1 kênh servo
static void pcaWriteServo(uint8_t channel, int angle) {
  if (angle < 0)   angle = 0;
  if (angle > 180) angle = 180;

  uint16_t pulse_us = map(angle, 0, 180, SERVO_MIN_US, SERVO_MAX_US);
  uint16_t ticks    = pcaUsToTicks(pulse_us);
  pca9685.setPWM(channel, 0, ticks);
}

// Ghi tất cả 16 servo cùng 1 góc
static void pcaWriteAllServos(int angle) {
  if (angle < 0)   angle = 0;
  if (angle > 180) angle = 180;

  for (uint8_t ch = 0; ch < 16; ++ch) {
    pcaWriteServo(ch, angle);
  }
}

// Khởi động mode Test PCA9685
inline void startPCA9685TestMode() {
  if (!pcaInitialized) {
    pca9685.begin();                     // dùng I2CScanBus
    pca9685.setPWMFreq(PCA_FREQUENCY_HZ);
    delay(10);
    pcaInitialized = true;
  }

  // Đưa tất cả servo về 90°
  pcaCurrentAngle = 90;
  pcaWriteAllServos(pcaCurrentAngle);

  // Reset trạng thái click
  pcaClickPending  = false;
  pcaLastClickTime = 0;

  // Hiển thị hướng dẫn
  lcdPrintLine(1, "PCA9685: 16 servo");
  lcdPrintLine(2, "Goc servo: 090 deg");
  lcdPrintLine(3, "1Click:I2C  2Click:90");
}

// Được gọi trong loop() khi appState == STATE_PCA9685_TEST
inline void updatePCA9685TestMode(unsigned long now) {
  // Nếu có 1 lần nhấn mà quá thời gian double-click
  // => xem như single-click => thoát về menu I2C
  if (pcaClickPending && (now - pcaLastClickTime > PCA_DOUBLE_MS)) {
    pcaClickPending = false;
    exitPCA9685ToI2CMenu();   // về menu I2C
    return;
  }
}

// Khi thoát mode -> tắt toàn bộ xung servo
inline void stopPCA9685TestMode() {
  for (uint8_t ch = 0; ch < 16; ++ch) {
    pca9685.setPWM(ch, 0, 0);   // off channel
  }
}

// Gọi từ onEncoderTurn() khi appState == STATE_PCA9685_TEST
inline void pca9685OnEncoderTurn(int direction) {
  // Mỗi nấc encoder đổi 2° => đi nhanh hơn nhưng vẫn khá mượt
  int newAngle = pcaCurrentAngle + direction * 2;

  if (newAngle < 0)   newAngle = 0;
  if (newAngle > 180) newAngle = 180;
  if (newAngle == pcaCurrentAngle) return;

  pcaCurrentAngle = newAngle;
  pcaWriteAllServos(pcaCurrentAngle);

  char buf[21];
  snprintf(buf, sizeof(buf), "Goc servo: %3d deg", pcaCurrentAngle);
  lcdPrintLine(2, buf);
}

// Gọi từ onButtonClick() khi appState == STATE_PCA9685_TEST
// - Click thứ 1: set pending, chờ coi có click thứ 2 không
// - Click thứ 2 trong khoảng PCA_DOUBLE_MS => double-click -> reset 90°
// - Nếu không có click thứ 2 => trong updatePCA9685TestMode() sẽ xử lý single-click -> thoát I2C
inline void pcaOnRawButtonClick() {
  unsigned long now = millis();

  if (pcaClickPending && (now - pcaLastClickTime <= PCA_DOUBLE_MS)) {
    // Double-click: reset về 90°
    pcaClickPending = false;

    pcaCurrentAngle = 90;
    pcaWriteAllServos(pcaCurrentAngle);

    char buf[21];
    snprintf(buf, sizeof(buf), "Goc servo: %3d deg", pcaCurrentAngle);
    lcdPrintLine(2, buf);
  } else {
    // Lần click đầu tiên -> chờ thêm trong PCA_DOUBLE_MS
    pcaClickPending  = true;
    pcaLastClickTime = now;
  }
}

#endif // PCA9685_TEST_MODE_H
