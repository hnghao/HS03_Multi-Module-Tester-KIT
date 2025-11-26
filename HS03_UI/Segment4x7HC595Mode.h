#ifndef SEGMENT_4X7_HC595_MODE_H
#define SEGMENT_4X7_HC595_MODE_H

#include <Arduino.h>
#include <Wire.h>

// I2CScanBus, I2C_SCAN_SDA_PIN, I2C_SCAN_SCL_PIN, ENCODER_SW_PIN
// được khai báo/define trong HS03_UI.ino
extern TwoWire I2CScanBus;

//---------------- Cấu hình chân ESP32-S3 ----------------
#define PIN_DIO   4   // SER (DS)  -> GPIO4
#define PIN_SCLK  1   // SHCP      -> GPIO1
#define PIN_RCLK  2   // STCP/LATCH-> GPIO2

//---------------- Cấu hình logic phần cứng ----------------
// 3641BS là Common Cathode (CC) -> giữ LED_IS_CC = 1 để dùng bảng mã CC.
#define LED_IS_CC             1   // để tham khảo; ta dùng bảng mã CC ở dưới

// SEG_ACTIVE_HIGH:
//   1 = segment ON khi xuất mức '1' từ 74HC595 (trực tiếp nối CC chuẩn)
//   0 = segment ON khi xuất mức '0' (qua ULN/transistor, active-LOW)
// Dựa theo code mẫu của bạn, giữ = 0
#define SEG_ACTIVE_HIGH       0

// DIGIT_ACTIVE_HIGH:
//   1 = chọn digit bằng mức '1'
//   0 = chọn digit bằng mức '0' (active-LOW)
#define DIGIT_ACTIVE_HIGH     1

// 4 bit chọn LED nằm ở Q0..Q3 (nếu bạn đấu ở Q4..Q7 thì đổi thành 0b11110000)
#define DIGIT_BITS            0b00001111

// Thứ tự đổ byte trong chuỗi 2 x 74HC595:
// 1 = đổ SEGMENTS trước, rồi DIGITS (IC_SEG ở xa MCU)
// 0 = đổ DIGITS trước, rồi SEGMENTS
#define CHAIN_ORDER_SEGMENTS_FIRST  1

// Nếu dây a..g..dp không đúng Q0..Q7 theo thứ tự, remap ở đây:
static inline uint8_t REMAP_SEG(uint8_t x) { return x; }

//---------------- Bảng số (Common Cathode): a..g ở bit0..6, dp ở bit7 ----
const uint8_t DIGIT_CC[10] = {
  0b00111111, // 0
  0b00000110, // 1
  0b01011011, // 2
  0b01001111, // 3
  0b01100110, // 4
  0b01101101, // 5
  0b01111101, // 6
  0b00000111, // 7
  0b01111111, // 8
  0b01101111  // 9
};
// Quy ước bit: [dp g f e d c b a] = [7..0]

//---------------- Tiện ích tạo byte xuất thực tế ------------------------
static inline uint8_t makeSegByte(uint8_t d, bool dpOn) {
  uint8_t pat = DIGIT_CC[d];            // logic ON = 1 (chuẩn CC)
  if (dpOn) pat |=  (1 << 7);           // bật dp
  else      pat &= ~(1 << 7);           // tắt dp
  pat = REMAP_SEG(pat);                 // nếu cần đổi chân

  // Ánh xạ ra vật lý theo mức kích thật sự của phần cứng
  return SEG_ACTIVE_HIGH ? pat : (uint8_t)~pat;
}

static inline uint8_t allDigitsMask() {
  uint8_t base = DIGIT_BITS;            // ví dụ 0000 1111 (bật 4 digit)
  return DIGIT_ACTIVE_HIGH ? base : (uint8_t)~base;
}

//---------------- Hàm shift và ghi đôi 595 -------------------------------
static inline void shiftOutByte_4x7(uint8_t data) {
  for (int i = 7; i >= 0; --i) {
    digitalWrite(PIN_SCLK, LOW);
    digitalWrite(PIN_DIO, (data >> i) & 0x01);
    digitalWrite(PIN_SCLK, HIGH);
  }
}

static inline void writeDual595(uint8_t segByte, uint8_t digitByte) {
  digitalWrite(PIN_RCLK, LOW);
#if CHAIN_ORDER_SEGMENTS_FIRST
  shiftOutByte_4x7(segByte);
  shiftOutByte_4x7(digitByte);
#else
  shiftOutByte_4x7(digitByte);
  shiftOutByte_4x7(segByte);
#endif
  digitalWrite(PIN_RCLK, HIGH);
}

//=====================================================
//  MODE 4x7 Segment HC595
//  - Đếm 0..9
//  - Chớp dp sau mỗi số
//  - Thoát khi nhấn nút encoder
//=====================================================
inline void start4x7HC595Mode() {
  // 1. TẠM TẮT I2C bus #1 trên GPIO1/2 để nhường chân cho 74HC595
  I2CScanBus.end();
  delay(5);

  // 2. Cấu hình 3 chân điều khiển 595
  pinMode(PIN_DIO,  OUTPUT);
  pinMode(PIN_SCLK, OUTPUT);
  pinMode(PIN_RCLK, OUTPUT);
  digitalWrite(PIN_DIO, LOW);
  digitalWrite(PIN_SCLK, LOW);
  digitalWrite(PIN_RCLK, LOW);

  // Trạng thái tắt an toàn
  const uint8_t SEG_OFF    = SEG_ACTIVE_HIGH   ? 0x00 : 0xFF;
  const uint8_t DIGITS_OFF = DIGIT_ACTIVE_HIGH ? 0x00 : 0xFF;
  writeDual595(SEG_OFF, DIGITS_OFF);

  // 3. Biến trạng thái giống code mẫu
  bool dpOn = false;
  uint8_t curDigit = 0;
  const uint8_t digitsOnMask = allDigitsMask();
  const unsigned long DIGIT_INTERVAL = 600UL; // 600ms mỗi số
  unsigned long lastChange = millis();

  // BỎ SÓC: chờ thả nút (vì vào mode là vừa nhấn chọn)
  while (digitalRead(ENCODER_SW_PIN) == LOW) {
    unsigned long now = millis();
    updateCountdown(now);
    handleBuzzer(now);
    delay(10);
  }

  // 4. Vòng lặp chính: hiển thị 0..9 + chớp dp, thoát khi nhấn nút
  while (true) {
    unsigned long now = millis();

    // Vẫn giữ countdown + buzzer global
    updateCountdown(now);
    handleBuzzer(now);

    // Đổi số sau mỗi DIGIT_INTERVAL
    if (now - lastChange >= DIGIT_INTERVAL) {
      lastChange = now;

      uint8_t seg = makeSegByte(curDigit, dpOn);
      writeDual595(seg, digitsOnMask);

      // chuẩn bị cho lần sau
      dpOn = !dpOn;                 // chớp dp
      curDigit = (curDigit + 1) % 10; // 0..9 lặp lại
    }

    // Kiểm tra nút để thoát
    int btn = digitalRead(ENCODER_SW_PIN);
    if (btn == LOW) {
      delay(30); // debounce nhẹ
      if (digitalRead(ENCODER_SW_PIN) == LOW) {
        break;   // Thoát mode 4x7
      }
    }
  }

  // 5. Tắt toàn bộ 4x7 và chờ thả nút
  writeDual595(SEG_OFF, DIGITS_OFF);
  while (digitalRead(ENCODER_SW_PIN) == LOW) {
    unsigned long now = millis();
    updateCountdown(now);
    handleBuzzer(now);
    delay(10);
  }

  // 6. BẬT LẠI I2C bus #1 trên SDA=1, SCL=2 cho các chức năng khác
  I2CScanBus.begin(I2C_SCAN_SDA_PIN, I2C_SCAN_SCL_PIN, 100000);
}

#endif // SEGMENT_4X7_HC595_MODE_H
