#ifndef PCA9685_TEST_MODE_H
#define PCA9685_TEST_MODE_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <LiquidCrystal_I2C.h>

// I2C bus 1 (SDA=1, SCL=2) được khai báo trong HS03_UI.ino
extern TwoWire I2CScanBus;

// LCD2004 dùng chung trong project
extern LiquidCrystal_I2C lcd;

// Countdown + buzzer
extern void updateCountdown(unsigned long now);
extern void handleBuzzer(unsigned long now);

// Dùng macro chân encoder từ HS03_UI.ino
// #define ENCODER_SW_PIN 15

// ================== CẤU HÌNH PCA9685 ==================

// ĐÚNG THEO SIGNATURE: Adafruit_PWMServoDriver(uint8_t addr, TwoWire &i2c)
// Không dùng &I2CScanBus nữa, mà truyền 0x40 trước, rồi I2CScanBus
static Adafruit_PWMServoDriver pca9685(0x40, I2CScanBus);

// Xung min/max cho servo (giống code mẫu)
#define PCA_SERVOMIN  150   // ~0.5 ms
#define PCA_SERVOMAX  600   // ~2.5 ms

// Số servo (kênh 0..15)
static const uint8_t PCA_NUM_SERVOS = 16;

// Dãy góc: 0 -> 90 -> 180 -> 90 -> 0
static const int PCA_POSITIONS[] = {0, 90, 180, 90, 0};
static const uint8_t PCA_NUM_POSITIONS = sizeof(PCA_POSITIONS) / sizeof(PCA_POSITIONS[0]);

// Thời gian dừng tại mỗi góc (ms)
static const unsigned long PCA_MOVE_DELAY = 1000UL;

// Đổi góc sang xung (0–4095)
inline int pcaAngleToPulse(int angle) {
  angle = constrain(angle, 0, 180);
  return map(angle, 0, 180, PCA_SERVOMIN, PCA_SERVOMAX);
}

// Set TẤT CẢ servo về cùng một góc
inline void pcaSetAllServosToAngle(int angle) {
  int pulselen = pcaAngleToPulse(angle);
  for (uint8_t ch = 0; ch < PCA_NUM_SERVOS; ch++) {
    pca9685.setPWM(ch, 0, pulselen);  // setPWM(channel, on, off)
  }
  Serial.print(F("Set all servos to angle: "));
  Serial.println(angle);
}

// ================== MODE TEST PCA9685 ==================
//
// Khi chọn "Test PCA9685":
//  - LCD hiển thị thông tin test + hướng dẫn
//  - PCA9685 quay tất cả servo qua dãy góc 0→90→180→90→0
//  - Có countdown + buzzer
//  - Nhấn nút encoder để thoát, quay lại menu I2C
//
inline void startPCA9685TestMode() {
  // Khởi tạo PCA9685 trên I2CScanBus (SDA=1, SCL=2)
  pca9685.begin();
  pca9685.setOscillatorFrequency(27000000); // 27 MHz
  pca9685.setPWMFreq(50);                  // 50 Hz cho servo
  delay(10);

  // Màn hình LCD
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Test PCA9685 Servo");
  lcd.setCursor(0, 1);
  lcd.print("0-90-180-90-0 deg ");
  lcd.setCursor(0, 2);
  lcd.print("Nhan nut de thoat");

  // Đưa tất cả servo về 0 độ lúc bắt đầu
  pcaSetAllServosToAngle(0);
  delay(500);

  bool exitRequested = false;

  while (!exitRequested) {
    for (uint8_t i = 0; i < PCA_NUM_POSITIONS; i++) {
      pcaSetAllServosToAngle(PCA_POSITIONS[i]);

      unsigned long start = millis();
      while (millis() - start < PCA_MOVE_DELAY) {
        unsigned long now = millis();
        updateCountdown(now);
        handleBuzzer(now);

        // Nhấn nút encoder để thoát
        if (digitalRead(ENCODER_SW_PIN) == LOW) {
          delay(30); // debounce
          if (digitalRead(ENCODER_SW_PIN) == LOW) {
            exitRequested = true;
            break;
          }
        }

        delay(5); // tránh chiếm CPU 100%
      }

      if (exitRequested) break;
    }
  }

  // Về lại góc 0 độ khi thoát
  pcaSetAllServosToAngle(0);

  // Chờ nhả nút hoàn toàn (tránh dính nhấn khi quay về menu)
  while (digitalRead(ENCODER_SW_PIN) == LOW) {
    unsigned long now = millis();
    updateCountdown(now);
    handleBuzzer(now);
    delay(10);
  }
}

#endif // PCA9685_TEST_MODE_H
