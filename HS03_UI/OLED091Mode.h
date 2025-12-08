#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Dùng chung bus I2CScanBus (SDA=1, SCL=2) đã khai báo trong HS03_UI.ino
extern TwoWire I2CScanBus;

// Dùng chung biến nút encoder để đồng bộ debounce sau khi thoát mode
extern bool lastBtnState;
extern unsigned long lastBtnTime;

// Kích thước OLED 0.91"
#define OLED091_WIDTH   128
#define OLED091_HEIGHT  32

// Địa chỉ I2C của OLED
#define OLED091_I2C_ADDRESS  0x3C

// Không dùng chân RESET riêng
#define OLED091_RESET  -1

// Tạo đối tượng SSD1306 dùng chung với bus I2CScanBus (SDA=1, SCL=2)
static Adafruit_SSD1306 oled091(OLED091_WIDTH, OLED091_HEIGHT, &I2CScanBus, OLED091_RESET);
static bool oled091Initialized = false;

// ----------------- Hàm hiển thị full trắng -----------------
inline void oled091_showFullWhite() {
  oled091.clearDisplay();
  oled091.fillRect(0, 0, OLED091_WIDTH, OLED091_HEIGHT, SSD1306_WHITE);
  oled091.display();
}

// ----------------- Hàm hiển thị chữ HSHOP giữa màn hình -----------------
inline void oled091_showHSHOPCenter() {
  oled091.clearDisplay();

  oled091.setTextSize(2);               // Phóng to x2
  oled091.setTextColor(SSD1306_WHITE);  // Chữ trắng nền đen

  const char* text = "HSHOP";
  const uint8_t textLen = 5;

  // Font mặc định: 6px/char (5 + 1 khoảng trắng)
  int16_t charWidth  = 6 * 2;   // nhân với size=2
  int16_t charHeight = 8 * 2;   // 8px * 2

  int16_t textWidth  = charWidth * textLen;
  int16_t textHeight = charHeight;

  int16_t x = (OLED091_WIDTH  - textWidth)  / 2;
  int16_t y = (OLED091_HEIGHT - textHeight) / 2;

  oled091.setCursor(x, y);
  oled091.print(text);
  oled091.display();
}

// ----------------- Khởi tạo OLED (chỉ 1 lần) -----------------
inline bool oled091_ensureInit() {
  if (!oled091Initialized) {
    if (!oled091.begin(SSD1306_SWITCHCAPVCC, OLED091_I2C_ADDRESS)) {
      // Nếu init lỗi thì thôi, không phá chương trình chính
      return false;
    }
    oled091.clearDisplay();
    oled091.display();
    oled091Initialized = true;
  }
  return true;
}

// ----------------- Mode OLED 0.91" (blocking) -----------------
// Hiệu ứng: 0.5s full trắng, 0.5s "HSHOP" giữa màn hình, lặp lại
// Nhấn nút encoder để THOÁT mode.
inline void startOLED091Mode() {
  if (!oled091_ensureInit()) {
    // Không tìm thấy OLED -> thoát nhẹ nhàng, không làm crash
    delay(10);
    return;
  }

  // Bắt đầu bằng full trắng giống demo gốc
  oled091_showFullWhite();
  bool showWhite = false;               // lần toggle đầu sẽ sang HSHOP
  unsigned long lastToggle = millis();

  // Xử lý nhấn nút để thoát
  const unsigned long EXIT_DEBOUNCE = 120;
  int prevBtn = digitalRead(ENCODER_SW_PIN);
  unsigned long stableSince = millis();

  while (true) {
    unsigned long now = millis();

    // Toggle giữa full trắng và HSHOP mỗi 500ms
    if (now - lastToggle >= 500) {
      lastToggle = now;
      showWhite = !showWhite;
      if (showWhite) {
        oled091_showFullWhite();
      } else {
        oled091_showHSHOPCenter();
      }
    }

    // Đọc nút encoder để thoát
    int curBtn = digitalRead(ENCODER_SW_PIN);
    if (curBtn != prevBtn) {
      prevBtn = curBtn;
      stableSince = now;
    } else {
      if (curBtn == LOW && (now - stableSince) > EXIT_DEBOUNCE) {
        // Chờ thả nút ra để tránh bị dính onButtonClick ngay sau đó
        while (digitalRead(ENCODER_SW_PIN) == LOW) {
          delay(5);
        }
        break;
      }
    }

    delay(5); // cho CPU nghỉ một chút
  }

  // Thoát mode: tắt OLED (cho gọn)
  oled091.clearDisplay();
  oled091.display();

  // Đồng bộ lại trạng thái nút với vòng lặp chính để tránh gọi onButtonClick liền
  lastBtnState = digitalRead(ENCODER_SW_PIN);
  lastBtnTime  = millis();
}
