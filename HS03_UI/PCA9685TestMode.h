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

// Pin nút encoder (đã define trong HS03_UI.ino)
#ifndef ENCODER_SW_PIN
#define ENCODER_SW_PIN 15
#endif

// Đối tượng PCA9685 dùng bus I2CScanBus, địa chỉ mặc định 0x40
static Adafruit_PWMServoDriver pca9685(0x40, I2CScanBus);

// Cấu hình servo
static const uint16_t PCA_FREQUENCY_HZ = 50;   // 50 Hz cho servo
static const uint16_t SERVO_MIN_US     = 500;  // ~0.5ms  ~ 0°
static const uint16_t SERVO_MAX_US     = 2500; // ~2.5ms ~180°

static int16_t pcaCurrentAngle = 90;   // góc hiện tại (0..180)
static bool    pcaInitialized  = false;

// ===== Hold-to-center detection (GIỮ 3s -> về 90°) =====
static bool          pcaBtnActive    = false;     // đang giữ nút trong mode
static bool          pcaHoldFired    = false;     // đã reset 90° trong lần giữ này
static unsigned long pcaBtnDownTime  = 0;
static int           pcaPrevBtnState = HIGH;

static const unsigned long PCA_HOLD_MS = 3000UL;  // giữ 3 giây

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

static void pcaCenterAllTo90() {
  pcaCurrentAngle = 90;
  pcaWriteAllServos(pcaCurrentAngle);

  char buf[21];
  snprintf(buf, sizeof(buf), "Goc servo: %3d deg", pcaCurrentAngle);
  lcdPrintLine(2, buf);
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
  pcaCenterAllTo90();

  // Reset trạng thái giữ nút
  pcaBtnActive    = false;
  pcaHoldFired    = false;
  pcaBtnDownTime  = 0;
  pcaPrevBtnState = digitalRead(ENCODER_SW_PIN);

  // Hiển thị hướng dẫn
  lcdPrintLine(1, "PCA9685: 16 servo");
  lcdPrintLine(2, "Goc servo: 090 deg");
  lcdPrintLine(3, "1Click:I2C Hold3s:90");
}

// Được gọi trong loop() khi appState == STATE_PCA9685_TEST
inline void updatePCA9685TestMode(unsigned long now) {
  int btnState = digitalRead(ENCODER_SW_PIN);

  // Nếu vì lý do nào đó không nhận "button down" từ onButtonClick()
  // thì vẫn bắt cạnh nhấn xuống ở đây để chắc chắn hoạt động
  if (pcaPrevBtnState == HIGH && btnState == LOW) {
    pcaBtnActive   = true;
    pcaHoldFired   = false;
    pcaBtnDownTime = now;
  }

  // Khi đang giữ: đủ 3s => về 90° (chỉ bắn 1 lần)
  if (pcaBtnActive && !pcaHoldFired && btnState == LOW) {
    if (now - pcaBtnDownTime >= PCA_HOLD_MS) {
      pcaHoldFired = true;
      pcaCenterAllTo90();
      // giữ nút tiếp cũng không thoát, chỉ reset 1 lần
    }
  }

  // Khi nhả nút:
  // - nếu chưa bắn hold (chưa đủ 3s) => coi như click ngắn => thoát menu I2C
  if (pcaPrevBtnState == LOW && btnState == HIGH) {
    if (pcaBtnActive && !pcaHoldFired) {
      pcaBtnActive = false;
      exitPCA9685ToI2CMenu();
      pcaPrevBtnState = btnState;
      return;
    }
    // nếu đã hold->90 thì nhả ra không thoát
    pcaBtnActive = false;
  }

  pcaPrevBtnState = btnState;
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
// Lưu ý: HS03_UI gọi onButtonClick ở cạnh nhấn xuống.
// Ta chỉ "arm" việc giữ nút ở đây; phần click ngắn / hold được xử lý trong updatePCA9685TestMode().
inline void pcaOnRawButtonClick() {
  unsigned long now = millis();
  pcaBtnActive   = true;
  pcaHoldFired   = false;
  pcaBtnDownTime = now;
}

#endif // PCA9685_TEST_MODE_H
