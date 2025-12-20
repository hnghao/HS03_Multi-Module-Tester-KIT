#ifndef SEGMENT_2X7_HC595_MODE_H
#define SEGMENT_2X7_HC595_MODE_H

#include <Arduino.h>

// I2CScanBus, I2C_SCAN_SDA_PIN, ENCODER_SW_PIN
// đã được khai báo trong HS03_UI.ino
extern TwoWire I2CScanBus;

// Các hàm countdown / buzzer dùng chung hệ thống
void updateCountdown(unsigned long now);
void handleBuzzer(unsigned long now);

// ====== ESP32-S3 + 2×74HC595 + 2×LED 7 đoạn CA ======
#define PIN_SDI   1   // SDI -> SER(DS) 74HC595
#define PIN_SCLK  2   // SCLK -> SRCLK(SHCP)
#define PIN_LOAD  4   // LOAD/LATCH -> RCLK(STCP)

// Bản đồ bit (common CATHODE): Q0..Q7 = A,B,C,D,E,F,G,DP
// Ở đây ta đánh số: bit0 = A, bit1 = B, ..., bit6 = G, bit7 = DP
static const uint8_t DIGIT_CC_2X7[10] = {
  //  DP G F E D C B A
  0b00111111, // 0: A,B,C,D,E,F
  0b00000110, // 1: B,C
  0b01011011, // 2: A,B,D,E,G
  0b01001111, // 3: A,B,C,D,G
  0b01100110, // 4: B,C,F,G
  0b01101101, // 5: A,C,D,F,G
  0b01111101, // 6: A,C,D,E,F,G
  0b00000111, // 7: A,B,C
  0b01111111, // 8: A,B,C,D,E,F,G
  0b01101111  // 9: A,B,C,D,F,G
};

// Từ byte CC + trạng thái DP -> tạo byte cho LED CA (active LOW)
inline uint8_t makeByteCA_2x7(uint8_t cc, bool dpOn) {
  uint8_t withDp = cc | (dpOn ? 0x80 : 0x00);   // bit7 = DP
  return ~withDp;                               // CA: active LOW -> đảo bit
}

/*
 * Gửi 16 bit cho 2 x 74HC595 (mỗi con 8 bit)
 *
 * - leftCA  : dữ liệu cho LED bên trái (hàng chục)
 * - rightCA : dữ liệu cho LED bên phải (hàng đơn vị)
 *
 * Thứ tự shiftOut được chỉnh để tens nằm bên trái, ones bên phải.
 */
inline void push595_2x7(uint8_t leftCA, uint8_t rightCA) {
  digitalWrite(PIN_LOAD, LOW);
  // Ghi byte của LED bên PHẢI trước
  shiftOut(PIN_SDI, PIN_SCLK, MSBFIRST, rightCA);
  // Sau đó ghi byte của LED bên TRÁI
  shiftOut(PIN_SDI, PIN_SCLK, MSBFIRST, leftCA);
  digitalWrite(PIN_LOAD, HIGH);
}

// Mode 2x7: intro 3s sáng toàn bộ, sau đó hiển thị 00..99 lặp lại,
// DP của cả 2 LED chớp tắt mỗi 1s
inline void start2x7HC595Mode() {
  // 1. Tạm dừng I2CScanBus vì dùng chung GPIO1/2 cho 74HC595
  I2CScanBus.end();

  // 2. Cấu hình chân 74HC595
  pinMode(PIN_SDI,  OUTPUT);
  pinMode(PIN_SCLK, OUTPUT);
  pinMode(PIN_LOAD, OUTPUT);
  digitalWrite(PIN_SDI,  LOW);
  digitalWrite(PIN_SCLK, LOW);
  digitalWrite(PIN_LOAD, LOW);

  // ===== Debounce nút Encoder để thoát mode =====
  int lastRawBtn = digitalRead(ENCODER_SW_PIN);
  int stableBtn = lastRawBtn;
  int prevStableBtn = stableBtn;
  uint32_t lastBounceMs = millis();

  // 3. Đợi nút encoder thả ra (tránh dính lần nhấn trước đó ở MENU)
  while (digitalRead(ENCODER_SW_PIN) == LOW) {
    delay(10);
  }

  // 4. Intro: BẬT TẤT CẢ CÁC LED trong 3 giây
  //
  // 74HC595 + LED 7 đoạn CA: 0xFF = OFF, 0x00 = tất cả segment ON (vì active LOW)
  unsigned long introStart = millis();
  push595_2x7(0x00, 0x00); // cả 2 LED: A..G + DP đều sáng

  while (millis() - introStart < 3000) {  // 3000ms = 3s
    unsigned long now = millis();
    // vẫn cập nhật countdown + buzzer trong thời gian intro
    updateCountdown(now);
    handleBuzzer(now);
    delay(5);
  }

  // ---- Biến điều khiển hiển thị sau intro ----
  uint8_t number = 0;  // hiển thị 00..99 lặp lại

  // Tốc độ tăng số (ms): có thể chỉnh nếu muốn nhanh/chậm hơn
  const unsigned long STEP_INTERVAL = 150;
  unsigned long lastStepMillis = millis();

  // DP chớp tắt: mỗi 1 giây đổi trạng thái (1s ON, 1s OFF)
  bool dpOn = false;
  const unsigned long DP_INTERVAL = 1000;   // 1000ms -> sau mỗi 1s đổi trạng thái
  unsigned long lastDpMillis = millis();

  // 5. Vòng lặp chính, blocking cho tới khi nhấn nút để thoát
  while (true) {
    unsigned long now = millis();

    // Countdown + buzzer chung hệ thống
    updateCountdown(now);
    handleBuzzer(now);

    // 5.1. Click nút encoder -> thoát mode (debounce ổn định, bắt theo NHẢ)
    // Lý do: tránh trường hợp nhấn nhanh bị "miss" hoặc phải giữ nút.
    static const uint32_t BTN_STABLE_MS = 25;

    int rawBtn = digitalRead(ENCODER_SW_PIN);

    if (rawBtn != lastRawBtn) {
      lastRawBtn   = rawBtn;
      lastBounceMs = now;
    }

    if ((uint32_t)(now - lastBounceMs) >= BTN_STABLE_MS && stableBtn != lastRawBtn) {
      prevStableBtn = stableBtn;
      stableBtn     = lastRawBtn;

      // phát hiện click khi NHẢ nút (LOW -> HIGH)
      if (prevStableBtn == LOW && stableBtn == HIGH) {
        break;
      }
    }

    // 5.2. Cập nhật DP: sau mỗi 1s đổi trạng thái (chớp tắt)
    if (now - lastDpMillis >= DP_INTERVAL) {
      lastDpMillis = now;
      dpOn = !dpOn;
    }

    // 5.3. Tăng số từ 00 -> 99, sau đó quay lại 00 (lặp vô hạn)
    if (now - lastStepMillis >= STEP_INTERVAL) {
      lastStepMillis = now;
      number++;
      if (number > 99) {
        number = 0;
      }
    }

    // 5.4. Tách ra 2 chữ số độc lập: hàng chục / hàng đơn vị
    uint8_t tens = number / 10;   // chữ số bên trái
    uint8_t ones = number % 10;   // chữ số bên phải

    uint8_t leftCA  = makeByteCA_2x7(DIGIT_CC_2X7[tens], dpOn); // LED trái = hàng chục
    uint8_t rightCA = makeByteCA_2x7(DIGIT_CC_2X7[ones], dpOn); // LED phải = hàng đơn vị

    // 5.5. Xuất dữ liệu ra 2 LED 7 đoạn
    push595_2x7(leftCA, rightCA);

    // delay nhỏ để hiển thị mượt mà mà không tốn CPU nhiều
    delay(2);
  }

  // 6. Khi thoát: tắt LED, trả chân về trạng thái an toàn
  push595_2x7(0xFF, 0xFF);  // tất cả segment OFF (sau khi đảo bit)
  pinMode(PIN_SDI,  INPUT);
  pinMode(PIN_SCLK, INPUT);
  pinMode(PIN_LOAD, INPUT);

  // 7. Khởi động lại I2CScanBus trên SDA=1, SCL=2 như cũ
  I2CScanBus.begin(I2C_SCAN_SDA_PIN, I2C_SCAN_SCL_PIN, 100000);
}

#endif // SEGMENT_2X7_HC595_MODE_H
