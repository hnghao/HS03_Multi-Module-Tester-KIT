#pragma once

#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Dùng chung bus I2CScanBus (SDA=1, SCL=2) đã khai báo trong HS03_UI.ino
extern TwoWire I2CScanBus;

// Dùng chung biến nút encoder để đồng bộ debounce sau khi thoát mode
extern bool lastBtnState;
extern unsigned long lastBtnTime;

// Kích thước OLED 0.96"
#define OLED096_WIDTH   128
#define OLED096_HEIGHT  64

// Địa chỉ I2C của OLED (thường 0x3C)
#define OLED096_I2C_ADDRESS  0x3C

// Không dùng chân RESET riêng
#define OLED096_RESET  -1

// Tạo đối tượng SSD1306 dùng chung với bus I2CScanBus
static Adafruit_SSD1306 oled096(OLED096_WIDTH, OLED096_HEIGHT, &I2CScanBus, OLED096_RESET);
static bool oled096Initialized = false;

// ----------------- Hàm hiển thị full trắng -----------------
inline void oled096_showFullWhite() {
  oled096.clearDisplay();
  oled096.fillRect(0, 0, OLED096_WIDTH, OLED096_HEIGHT, SSD1306_WHITE);
  oled096.display();
}

// ----------------- Hàm hiển thị chữ HSHOP giữa màn hình -----------------
inline void oled096_showHSHOPCenter() {
  oled096.clearDisplay();

  oled096.setTextSize(2);               // Phóng to x2
  oled096.setTextColor(SSD1306_WHITE);  // Chữ trắng

  const char* text = "HSHOP";

  // Tính kích thước thực của chữ để canh giữa
  int16_t x1, y1;
  uint16_t w, h;
  oled096.getTextBounds(text, 0, 0, &x1, &y1, &w, &h);

  int16_t x = (OLED096_WIDTH  - w) / 2;
  int16_t y = (OLED096_HEIGHT - h) / 2;

  oled096.setCursor(x, y);
  oled096.print(text);
  oled096.display();
}

// ----------------- Khởi tạo OLED (chỉ 1 lần) -----------------
inline bool oled096_ensureInit() {
  if (!oled096Initialized) {
    if (!oled096.begin(SSD1306_SWITCHCAPVCC, OLED096_I2C_ADDRESS)) {
      // Không tìm thấy OLED -> thoát nhẹ nhàng, không làm crash chương trình
      return false;
    }
    oled096.clearDisplay();
    oled096.display();
    oled096Initialized = true;
  }
  return true;
}

// ----------------- Mode OLED 0.96" (blocking) -----------------
// Hiệu ứng: 0.5s full trắng, 0.5s "HSHOP" giữa màn hình, lặp lại
// Nhấn nút encoder để THOÁT mode.
inline void startOLED096Mode() {
  if (!oled096_ensureInit()) {
    delay(10);
    return;
  }

  // Bắt đầu bằng full trắng giống demo gốc
  oled096_showFullWhite();
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
        oled096_showFullWhite();
      } else {
        oled096_showHSHOPCenter();
      }
    }

    // Đọc nút encoder để thoát
    int curBtn = digitalRead(ENCODER_SW_PIN);
    if (curBtn != prevBtn) {
      prevBtn = curBtn;
      stableSince = now;
    } else {
      if (curBtn == LOW && (now - stableSince) > EXIT_DEBOUNCE) {
        // Chờ nhả nút ra để tránh bị onButtonClick ngay sau đó
        while (digitalRead(ENCODER_SW_PIN) == LOW) {
          delay(5);
        }
        break;
      }
    }

    delay(5);
  }

  // Thoát mode: tắt OLED cho gọn
  oled096.clearDisplay();
  oled096.display();

  // Đồng bộ lại trạng thái nút với vòng lặp chính
  lastBtnState = digitalRead(ENCODER_SW_PIN);
  lastBtnTime  = millis();
}
