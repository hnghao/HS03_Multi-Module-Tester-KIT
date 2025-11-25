#ifndef TM1637_MODE_H
#define TM1637_MODE_H

#include <Arduino.h>
#include <TM1637Display.h>

// Dùng cùng chân như chương trình mẫu:
//  - CLK = GPIO2
//  - DIO = GPIO1
#define TM1637_CLK_PIN  2
#define TM1637_DIO_PIN  1

// I2CScanBus được khai báo trong HS03_UI.ino
extern TwoWire I2CScanBus;

// Các macro I2C_SCAN_SDA_PIN, I2C_SCAN_SCL_PIN, ENCODER_SW_PIN
// đã được define trong HS03_UI.ino trước khi include header này,
// nên có thể dùng trực tiếp ở đây.

// --------------------------------------------------
// startTM1637Mode(): chạy giống hệt chương trình mẫu,
// nhưng chạy "blocking" tới khi bạn nhấn nút để thoát.
// Trong lúc này chỉ TM1637 dùng chân 1–2.
// --------------------------------------------------
inline void startTM1637Mode() {
  // 1. TẠM TẮT I2C bus #1 trên chân 1–2 để nhường chân cho TM1637
  I2CScanBus.end();
  delay(5);

  // 2. Tạo đối tượng TM1637Display sử dụng đúng chân
  TM1637Display display(TM1637_CLK_PIN, TM1637_DIO_PIN);

  // Độ sáng: 0–7 (0 mờ nhất, 7 sáng nhất)
  display.setBrightness(5);

  // Hiệu ứng khởi động: chớp toàn bộ LED 2 lần
  uint8_t dataOn[]  = {0xFF, 0xFF, 0xFF, 0xFF}; // tất cả segment sáng
  uint8_t dataOff[] = {0x00, 0x00, 0x00, 0x00}; // tắt hết

  for (int i = 0; i < 2; i++) {
    display.setSegments(dataOn);   // Bật tất cả
    delay(200);
    display.setSegments(dataOff);  // Tắt tất cả
    delay(200);
  }

  // Thời gian hiển thị cố định: 12:34
  const int DISPLAY_TIME = 1234;

  bool colonState = false;                      // Trạng thái dấu hai chấm
  unsigned long lastToggleTime = millis();      // Lần cuối đổi trạng thái
  const unsigned long TOGGLE_INTERVAL = 1000UL; // 1000 ms = 1 giây

  // Sau khi chớp xong thì hiển thị giờ lần đầu, tắt colon
  display.showNumberDecEx(DISPLAY_TIME, 0x00, true);

  // BỎ SÓC: chờ thả nút (vì lúc vào mode là bạn vừa nhấn nút chọn)
  while (digitalRead(ENCODER_SW_PIN) == LOW) {
    unsigned long now = millis();
    updateCountdown(now);
    handleBuzzer(now);
    delay(10);
  }

  // 3. Vòng lặp chính: nhấp nháy colon, thoát khi nhấn nút
  while (true) {
    unsigned long now = millis();

    // Vẫn giữ logic countdown + buzzer global
    updateCountdown(now);
    handleBuzzer(now);

    // Mỗi 1 giây đổi trạng thái dấu hai chấm
    if (now - lastToggleTime >= TOGGLE_INTERVAL) {
      lastToggleTime = now;
      colonState = !colonState;

      // Bit 6 (0b01000000) điều khiển dấu hai chấm trong TM1637Display
      uint8_t colonMask = colonState ? 0b01000000 : 0x00;

      // Hiển thị số 1234 với colon on/off
      display.showNumberDecEx(DISPLAY_TIME, colonMask, true);
    }

    // Kiểm tra nút bấm để thoát
    int btn = digitalRead(ENCODER_SW_PIN);
    if (btn == LOW) {
      delay(30); // debounce nhẹ
      if (digitalRead(ENCODER_SW_PIN) == LOW) {
        break;   // Thoát khỏi vòng lặp TM1637
      }
    }
  }

  // 4. Tắt toàn bộ TM1637 và chờ thả nút
  display.clear();
  while (digitalRead(ENCODER_SW_PIN) == LOW) {
    unsigned long now = millis();
    updateCountdown(now);
    handleBuzzer(now);
    delay(10);
  }

  // 5. BẬT LẠI I2C bus #1 trên SDA=1, SCL=2 cho các chức năng khác
  I2CScanBus.begin(I2C_SCAN_SDA_PIN, I2C_SCAN_SCL_PIN, 100000);
}

// Hai hàm dưới đây giữ cho tương thích với code cũ,
// nhưng hiện tại không dùng gì đặc biệt.
inline void updateTM1637Mode(unsigned long now) {
  (void)now;
}

inline void stopTM1637Mode() {
  // Không cần làm gì thêm (đã clear trong startTM1637Mode)
}

#endif  // TM1637_MODE_H
