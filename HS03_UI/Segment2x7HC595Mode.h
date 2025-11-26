#ifndef SEGMENT_2X7_HC595_MODE_H
#define SEGMENT_2X7_HC595_MODE_H

#include <Arduino.h>
#include <Wire.h>

// I2CScanBus, I2C_SCAN_SDA_PIN, I2C_SCAN_SCL_PIN, ENCODER_SW_PIN
// đã khai báo/define trong HS03_UI.ino
extern TwoWire I2CScanBus;

// ====== ESP32-S3 + 2×74HC595 + 2×LED 7 đoạn CA ======
#define PIN_SDI   1   // SDI -> SER(DS) 74HC595
#define PIN_SCLK  2   // SCLK -> SRCLK(SHCP)
#define PIN_LOAD  4   // LOAD/LATCH -> RCLK(STCP)

// Bản đồ bit: Q0..Q7 = A,B,C,D,E,F,G,DP (bit0=A ... bit6=G, bit7=DP)
// Mảng dưới đây là mẫu cho Common-Cathode (CC). Ta sẽ đảo bit để ra Common-Anode (CA).
const uint8_t DIGIT_CC_2X7[10] = {
  // 0     1     2     3     4     5     6     7     8     9
  0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F
};

// Chuyển mẫu CC sang CA, kèm trạng thái DP.
// dpOn = true => bật dấu chấm
inline uint8_t makeByteCA_2x7(uint8_t cc, bool dpOn) {
  uint8_t withDp = cc | (dpOn ? 0x80 : 0x00); // CC: 1 là sáng
  return ~withDp;                             // CA: active LOW -> đảo bit
}

// Gửi 16 bit cho 2 x 74HC595 (mỗi con 8 bit)
// byte đầu tiên -> IC xa hơn, byte sau -> IC gần MCU
inline void push595_2x7(uint8_t leftCA, uint8_t rightCA) {
  digitalWrite(PIN_LOAD, LOW);
  shiftOut(PIN_SDI, PIN_SCLK, MSBFIRST, leftCA);   // LED trái
  shiftOut(PIN_SDI, PIN_SCLK, MSBFIRST, rightCA);  // LED phải
  digitalWrite(PIN_LOAD, HIGH);                    // chốt dữ liệu
}

// Hiển thị cùng một số trên cả 2 LED; blink cả 2 DP
inline void showTwoDigitsSame(uint8_t digit, bool dpBlink) {
  if (digit > 9) digit = 0;
  uint8_t cc = DIGIT_CC_2X7[digit];
  bool dpLeft  = dpBlink;
  bool dpRight = dpBlink;

  uint8_t leftCA  = makeByteCA_2x7(cc, dpLeft);
  uint8_t rightCA = makeByteCA_2x7(cc, dpRight);
  push595_2x7(leftCA, rightCA);
}

// =====================================================
// MODE 2x7 Segment HC595
//  - Đếm 0..9
//  - Chớp DP sau mỗi số
//  - Thoát khi nhấn nút Encoder
// =====================================================
inline void start2x7HC595Mode() {
  // 1. TẠM TẮT I2C bus #1 trên GPIO1/2 để nhường chân cho 74HC595
  I2CScanBus.end();
  delay(5);

  // 2. Cấu hình 3 chân điều khiển 595
  pinMode(PIN_SDI,  OUTPUT);
  pinMode(PIN_SCLK, OUTPUT);
  pinMode(PIN_LOAD, OUTPUT);
  digitalWrite(PIN_SDI,  LOW);
  digitalWrite(PIN_SCLK, LOW);
  digitalWrite(PIN_LOAD, LOW);

  // Tắt hết lúc khởi động (CA: 1 = tắt)
  push595_2x7(0xFF, 0xFF);

  // 3. Biến trạng thái giống code mẫu
  bool dp = false;              // trạng thái dấu chấm
  uint8_t curDigit = 0;         // số hiện tại 0..9
  const unsigned long DIGIT_INTERVAL = 700UL;  // 700 ms mỗi số
  unsigned long lastChange = millis();

  // BỎ SÓC: chờ thả nút (vì vào mode là vừa nhấn chọn)
  while (digitalRead(ENCODER_SW_PIN) == LOW) {
    unsigned long now = millis();
    updateCountdown(now);
    handleBuzzer(now);
    delay(10);
  }

  // 4. Vòng lặp chính: hiển thị 0..9 + chớp DP, thoát khi nhấn nút
  while (true) {
    unsigned long now = millis();

    // Vẫn giữ countdown + buzzer global
    updateCountdown(now);
    handleBuzzer(now);

    // Đổi số sau mỗi DIGIT_INTERVAL
    if (now - lastChange >= DIGIT_INTERVAL) {
      lastChange = now;

      showTwoDigitsSame(curDigit, dp);

      dp       = !dp;                // đổi trạng thái DP
      curDigit = (curDigit + 1) % 10; // 0..9 lặp lại
    }

    // Kiểm tra nút để thoát
    int btn = digitalRead(ENCODER_SW_PIN);
    if (btn == LOW) {
      delay(30); // debounce nhẹ
      if (digitalRead(ENCODER_SW_PIN) == LOW) {
        break;   // Thoát mode 2x7
      }
    }
  }

  // 5. Tắt toàn bộ 2x7 và chờ thả nút
  push595_2x7(0xFF, 0xFF);
  while (digitalRead(ENCODER_SW_PIN) == LOW) {
    unsigned long now = millis();
    updateCountdown(now);
    handleBuzzer(now);
    delay(10);
  }

  // 6. BẬT LẠI I2C bus #1 trên SDA=1, SCL=2 cho các chức năng khác
  I2CScanBus.begin(I2C_SCAN_SDA_PIN, I2C_SCAN_SCL_PIN, 100000);
}

#endif // SEGMENT_2X7_HC595_MODE_H
