#ifndef ULTRASONIC_JSN_MODE_H
#define ULTRASONIC_JSN_MODE_H

#include <Arduino.h>
#include <LiquidCrystal_I2C.h>

// lcd được khai báo trong HS03_UI.ino
extern LiquidCrystal_I2C lcd;

// Hàm countdown + buzzer
extern void updateCountdown(unsigned long now);
extern void handleBuzzer(unsigned long now);

// Dùng chung chân encoder đã define trong HS03_UI.ino
// #define ENCODER_SW_PIN 15

// ================== CẤU HÌNH CHÂN JSN-SR04T ==================
#define JSN_TRIG_PIN 14   // TRIG của JSN-SR04T nối GPIO14
#define JSN_ECHO_PIN 1    // ECHO nối GPIO9 (qua mạch giảm áp 5V -> 3.3V)

// ================== MODE ULTRASONIC JSN ==================
//
// Logic theo demo:
// - Phát xung 20us trên TRIG
// - Đo pulse ECHO bằng pulseIn()
// - Tính khoảng cách theo (duration / 2) * 0.343 (mm)
// - Hiển thị:
//     Dòng 0: "Gia tri khoang cach"
//     Dòng 1: <giá trị> "mm"
//     Dòng 2: "Nhan nut de thoat"
// - Lặp lại cho đến khi nhấn nút encoder để thoát
//
inline void startUltrasonicJSNMode() {
  // Cấu hình chân
  pinMode(JSN_TRIG_PIN, OUTPUT);
  pinMode(JSN_ECHO_PIN, INPUT);
  digitalWrite(JSN_TRIG_PIN, LOW);

  // Màn hình theo đúng demo + thêm hướng dẫn dòng 2
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Gia tri khoang cach");
  lcd.setCursor(0, 2);               // <-- DÒNG 2
  lcd.print("Nhan nut de thoat ");   // <-- THÊM CHUỖI HƯỚNG DẪN

  // Vòng lặp chính: đo & hiển thị cho đến khi nhấn nút để thoát
  while (true) {
    unsigned long now = millis();
    updateCountdown(now);
    handleBuzzer(now);

    // ----- Gửi xung TRIG giống demo -----
    digitalWrite(JSN_TRIG_PIN, LOW);
    delayMicroseconds(2);

    digitalWrite(JSN_TRIG_PIN, HIGH);
    delayMicroseconds(20);   // 20us như code mẫu
    digitalWrite(JSN_TRIG_PIN, LOW);

    // ----- Đo độ rộng xung ECHO -----
    // Thêm timeout 30ms để tránh treo nếu không có echo
    unsigned long duration = pulseIn(JSN_ECHO_PIN, HIGH, 30000UL);

    float distance = 0.0f;

    if (duration == 0) {
      // Không nhận được echo
      lcd.setCursor(0, 1);
      lcd.print("Khong nhan echo   ");
    } else {
      // Theo demo: distance (mm) = (duration / 2) * 0.343
      distance = (duration / 2.0f) * 0.343f;

      lcd.setCursor(0, 1);
      // Xoá dòng trước
      lcd.print("                    ");
      lcd.setCursor(0, 1);
      lcd.print(distance, 1);  // 1 số lẻ
      lcd.print("mm");
      lcd.print("   ");         // xoá ký tự thừa
      delay(500);
    }

    // ----- Cho cảm biến nghỉ ~100ms, vẫn cập nhật countdown/buzzer
    bool exitRequested = false;
    unsigned long waitStart = millis();
    while (millis() - waitStart < 100UL) {
      unsigned long t = millis();
      updateCountdown(t);
      handleBuzzer(t);

      // Nhấn nút encoder để thoát
      if (digitalRead(ENCODER_SW_PIN) == LOW) {
        delay(30); // debounce
        if (digitalRead(ENCODER_SW_PIN) == LOW) {
          exitRequested = true;
          break;
        }
      }
      delay(5);
    }

    if (exitRequested) {
      break;
    }
  }

  // Đảm bảo TRIG về LOW khi thoát
  digitalWrite(JSN_TRIG_PIN, LOW);

  // Chờ nhả nút hoàn toàn (tránh về Menu bị dính nhấn)
  while (digitalRead(ENCODER_SW_PIN) == LOW) {
    unsigned long now = millis();
    updateCountdown(now);
    handleBuzzer(now);
    delay(10);
  }
}

#endif // ULTRASONIC_JSN_MODE_H
