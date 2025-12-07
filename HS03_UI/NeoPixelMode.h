#ifndef NEO_PIXEL_MODE_H
#define NEO_PIXEL_MODE_H

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <LiquidCrystal_I2C.h>

// neoStrip & lcd được khai báo trong HS03_UI.ino
extern Adafruit_NeoPixel neoStrip;
extern LiquidCrystal_I2C lcd;

// Hàm countdown + buzzer + in LCD
extern void updateCountdown(unsigned long now);
extern void handleBuzzer(unsigned long now);
extern void lcdPrintLine(uint8_t row, const char* text);

// Các biến encoder trong HS03_UI.ino (để đồng bộ lại sau khi thoát mode)
extern int  lastClkState;
extern unsigned long lastEncoderTime;

// ENCODER_* đã #define trong HS03_UI.ino
// #define ENCODER_CLK_PIN 17
// #define ENCODER_DT_PIN  16
// #define ENCODER_SW_PIN  15

// Cờ báo thoát hiệu ứng NeoPixel
static bool neoAbort = false;

// Độ sáng hiện tại (0..255), mặc định giống code mẫu ~80
static uint8_t neoBrightness = 80;

// Trạng thái riêng cho encoder trong mode NeoPixel
static int            neoLastClkState    = HIGH;
static unsigned long  neoLastEncMillis   = 0;
static const unsigned long NEO_ENC_DEBOUNCE = 3;   // ms

// ====== HÀM VẼ THANH BRIGHTNESS LÊN LCD (dòng 3) ======
inline void neoUpdateBrightnessBar() {
  // Dạng: "Brt [########      ]" (20 ký tự)
  char line[21];
  line[0] = 'B';
  line[1] = 'r';
  line[2] = 't';
  line[3] = ' ';
  line[4] = '[';

  // Thanh bar dài 14 ô (cột 5..18)
  uint8_t segments = (uint32_t)neoBrightness * 14 / 255;  // 0..14
  for (uint8_t i = 0; i < 14; i++) {
    line[5 + i] = (i < segments) ? '#' : ' ';
  }

  line[19] = ']';
  line[20] = '\0';

  lcdPrintLine(3, line);
}

// Áp dụng brightness hiện tại cho strip
inline void neoApplyBrightness() {
  neoStrip.setBrightness(neoBrightness);
  neoStrip.show();
  neoUpdateBrightnessBar();
}

// ====== ĐỌC XOAY ENCODER ĐỂ CHỈNH ĐỘ SÁNG ======
inline void neoUpdateEncoderBrightness() {
  int clk = digitalRead(ENCODER_CLK_PIN);
  if (clk != neoLastClkState) {
    if (clk == LOW && (millis() - neoLastEncMillis) > NEO_ENC_DEBOUNCE) {
      int dt  = digitalRead(ENCODER_DT_PIN);
      int dir = (dt == HIGH) ? +1 : -1;   // giống hướng trong HS03_UI.ino

      // Mỗi nấc thay đổi 4 đơn vị brightness
      int val = (int)neoBrightness + dir * 4;
      if (val < 1)   val = 1;
      if (val > 255) val = 255;

      if (val != neoBrightness) {
        neoBrightness = (uint8_t)val;
        neoApplyBrightness();     // cập nhật brightness + thanh bar
      }

      neoLastEncMillis = millis();
    }
    neoLastClkState = clk;
  }
}

// ====== HÀM HIỆN MỘT MÀU LÊN TOÀN BỘ DẢI LED ======
inline void neoShowColor(uint32_t color) {
  uint16_t n = neoStrip.numPixels();
  for (uint16_t i = 0; i < n; i++) {
    neoStrip.setPixelColor(i, color);
  }
  neoStrip.show();
}

// Delay có kèm countdown/buzzer + encoder brightness + nút thoát
inline bool neoDelayWithServices(unsigned long totalMs) {
  unsigned long start = millis();
  while (millis() - start < totalMs) {
    unsigned long now = millis();
    updateCountdown(now);
    handleBuzzer(now);

    // Xoay encoder để chỉnh độ sáng
    neoUpdateEncoderBrightness();

    // Nhấn nút encoder để thoát
    if (digitalRead(ENCODER_SW_PIN) == LOW) {
      delay(30); // debounce
      if (digitalRead(ENCODER_SW_PIN) == LOW) {
        neoAbort = true;
        return true; // thoát mode
      }
    }

    delay(5); // tránh chiếm CPU 100%
  }
  return false;
}

// ============================
// API dùng trong HS03_UI.ino
// ============================

// Được gọi khi chọn menu "Neopixel"
inline void startNeoPixelMode() {
  neoAbort = false;

  // Chuẩn bị strip theo code mẫu mới
  neoStrip.clear();
  neoStrip.show();
  neoStrip.setBrightness(neoBrightness);

  // Hiển thị thanh brightness ban đầu
  neoUpdateBrightnessBar();

  // Chờ nhả nút (vì vừa nhấn để chọn NeoPixel)
  while (digitalRead(ENCODER_SW_PIN) == LOW) {
    unsigned long now = millis();
    updateCountdown(now);
    handleBuzzer(now);
    delay(10);
  }

  // Đồng bộ encoder state (tránh giật menu sau khi thoát)
  neoLastClkState   = digitalRead(ENCODER_CLK_PIN);
  neoLastEncMillis  = millis();
  lastClkState      = neoLastClkState;
  lastEncoderTime   = neoLastEncMillis;

  // Vòng lặp chính: ĐỎ -> XANH LÁ -> XANH DƯƠNG -> TRẮNG
  while (!neoAbort) {
    // Màu ĐỎ
    neoShowColor(neoStrip.Color(255, 0, 0));
    if (neoDelayWithServices(1000)) break;

    // Màu XANH LÁ
    neoShowColor(neoStrip.Color(0, 255, 0));
    if (neoDelayWithServices(1000)) break;

    // Màu XANH DƯƠNG
    neoShowColor(neoStrip.Color(0, 0, 255));
    if (neoDelayWithServices(1000)) break;

    // Màu TRẮNG
    neoShowColor(neoStrip.Color(255, 255, 255));
    if (neoDelayWithServices(1000)) break;
  }

  // Tắt led khi thoát
  neoStrip.clear();
  neoStrip.show();

  // Chờ người dùng nhả nút nếu còn giữ
  while (digitalRead(ENCODER_SW_PIN) == LOW) {
    unsigned long now = millis();
    updateCountdown(now);
    handleBuzzer(now);
    delay(10);
  }

  // Đồng bộ lại encoder cho logic menu chính
  lastClkState    = digitalRead(ENCODER_CLK_PIN);
  lastEncoderTime = millis();
}

// Hàm này giữ lại cho hợp lệ với switch(appState),
// nhưng hiện tại không dùng (mode chạy blocking trong startNeoPixelMode)
inline void updateNeoPixelMode(unsigned long now) {
  (void)now;
}

// Được gọi nếu code cũ còn nhảy vào STATE_NEOPIXEL và nhấn nút thoát
inline void stopNeoPixelMode() {
  neoAbort = true;
  neoStrip.clear();
  neoStrip.show();
}

#endif // NEO_PIXEL_MODE_H
