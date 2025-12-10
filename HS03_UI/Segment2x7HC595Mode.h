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
// bit = 1 -> segment SÁNG (cho LED CC) -> lát nữa sẽ đảo để dùng cho CA
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

// Gửi 16 bit cho 2 x 74HC595 (mỗi con 8 bit)
// Giả sử: byte đầu -> LED trái, byte sau -> LED phải (hoặc ngược lại tùy board)
inline void push595_2x7(uint8_t leftCA, uint8_t rightCA) {
  digitalWrite(PIN_LOAD, LOW);
  shiftOut(PIN_SDI, PIN_SCLK, MSBFIRST, leftCA);   // LED bên trái
  shiftOut(PIN_SDI, PIN_SCLK, MSBFIRST, rightCA);  // LED bên phải
  digitalWrite(PIN_LOAD, HIGH);
}

// Mode 2x7: hiển thị 2 chữ số độc lập, đếm 00..99 rồi dừng, DP chớp sau mỗi 1s
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

  // 3. Đợi nút encoder thả ra (tránh dính lần nhấn trước đó ở MENU)
  while (digitalRead(ENCODER_SW_PIN) == LOW) {
    delay(10);
  }

  // 4. Tắt hết lúc bắt đầu
  push595_2x7(0xFF, 0xFF);

  // ---- Biến điều khiển hiển thị ----
  uint8_t number        = 0;        // hiển thị 00..99
  bool    finishedCount = false;    // true khi đạt 99

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

    // 5.1. Nhấn nút encoder -> thoát mode
    if (digitalRead(ENCODER_SW_PIN) == LOW) {
      while (digitalRead(ENCODER_SW_PIN) == LOW) {
        delay(10);
      }
      break;
    }

    // 5.2. Cập nhật DP: sau mỗi 1s đổi trạng thái (chớp tắt)
    if (now - lastDpMillis >= DP_INTERVAL) {
      lastDpMillis = now;
      dpOn = !dpOn;
    }

    // 5.3. Tăng số từ 00 -> 99, sau đó đứng yên tại 99
    if (!finishedCount && (now - lastStepMillis >= STEP_INTERVAL)) {
      lastStepMillis = now;
      if (number < 99) {
        number++;
      } else {
        number        = 99;
        finishedCount = true;   // không tăng nữa, giữ 99
      }
    }

    // 5.4. Tách ra 2 chữ số độc lập: hàng chục / hàng đơn vị
    uint8_t tens = number / 10;   // chữ số bên trái
    uint8_t ones = number % 10;   // chữ số bên phải

    uint8_t leftCA  = makeByteCA_2x7(DIGIT_CC_2X7[tens], dpOn);
    uint8_t rightCA = makeByteCA_2x7(DIGIT_CC_2X7[ones], dpOn);

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
