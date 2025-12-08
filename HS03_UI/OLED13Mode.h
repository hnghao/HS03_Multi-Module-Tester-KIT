#pragma once

#include <Arduino.h>
#include <U8g2lib.h>

// Dùng chung các biến nút từ HS03_UI.ino
extern bool lastBtnState;
extern unsigned long lastBtnTime;

// Dùng chung chân I2C của bus I2C Scan (SDA=1, SCL=2)
#ifndef OLED13_SDA_PIN
#define OLED13_SDA_PIN I2C_SCAN_SDA_PIN
#endif

#ifndef OLED13_SCL_PIN
#define OLED13_SCL_PIN I2C_SCAN_SCL_PIN
#endif

// Tạo đối tượng U8g2 cho SH1106 128x64 I2C, dùng SW I2C trên SDA/SCL của bus scan
// (không dùng Wire/I2CScanBus để tránh đụng tới LCD)
static U8G2_SH1106_128X64_NONAME_F_SW_I2C u8g2_oled13(
  U8G2_R0,
  /* clock = */ OLED13_SCL_PIN,
  /* data  = */ OLED13_SDA_PIN,
  /* reset = */ U8X8_PIN_NONE
);

static bool oled13Initialized = false;

// ----------------- Khởi tạo OLED 1.3" (1 lần) -----------------
inline void oled13EnsureInit() {
  if (!oled13Initialized) {
    u8g2_oled13.begin();
    oled13Initialized = true;
  }
}

// ----------------- Full trắng toàn màn hình -----------------
inline void oled13_showFullWhite() {
  u8g2_oled13.clearBuffer();
  u8g2_oled13.drawBox(0, 0, 128, 64);
  u8g2_oled13.sendBuffer();
}

// ----------------- Hiển thị chữ HSHOP giữa màn hình -----------------
inline void oled13_showHSHOPCenter() {
  u8g2_oled13.clearBuffer();

  // Font chữ to, dễ đọc
  u8g2_oled13.setFont(u8g2_font_ncenB14_tr);

  const char *text = "HSHOP";

  // Tính kích thước để canh giữa
  uint16_t textWidth  = u8g2_oled13.getUTF8Width(text);
  int16_t  ascent     = u8g2_oled13.getAscent();
  int16_t  descent    = u8g2_oled13.getDescent();
  uint16_t textHeight = ascent - descent;

  int16_t x = (128 - (int16_t)textWidth) / 2;
  int16_t y = (64 + textHeight) / 2 - 1;  // canh theo baseline

  u8g2_oled13.setCursor(x, y);
  u8g2_oled13.print(text);

  u8g2_oled13.sendBuffer();
}

// ----------------- Mode OLED 1.3" (blocking) -----------------
// Hiệu ứng: 0.5s full trắng, 0.5s "HSHOP", nhấn nút encoder để THOÁT
inline void startOLED13Mode() {
  oled13EnsureInit();

  oled13_showFullWhite();
  bool showWhite = false;
  unsigned long lastToggle = millis();

  // Debounce thoát mode
  const unsigned long EXIT_DEBOUNCE = 120;
  int prevBtn = digitalRead(ENCODER_SW_PIN);
  unsigned long stableSince = millis();

  while (true) {
    unsigned long now = millis();

    // Toggle giữa full trắng và HSHOP mỗi 500ms
    if (now - lastToggle >= 1000) {
      lastToggle = now;
      showWhite = !showWhite;
      if (showWhite) {
        oled13_showFullWhite();
      } else {
        oled13_showHSHOPCenter();
      }
    }

    // Đọc nút encoder để thoát
    int curBtn = digitalRead(ENCODER_SW_PIN);
    if (curBtn != prevBtn) {
      prevBtn = curBtn;
      stableSince = now;
    } else {
      if (curBtn == LOW && (now - stableSince) > EXIT_DEBOUNCE) {
        // Chờ nhả nút ra để tránh onButtonClick() ăn nhầm
        while (digitalRead(ENCODER_SW_PIN) == LOW) {
          delay(5);
        }
        break;
      }
    }

    delay(5);
  }

  // Thoát mode: tắt nội dung OLED
  u8g2_oled13.clearBuffer();
  u8g2_oled13.sendBuffer();

  // Đồng bộ lại trạng thái nút với vòng loop chính
  lastBtnState = digitalRead(ENCODER_SW_PIN);
  lastBtnTime  = millis();
}
