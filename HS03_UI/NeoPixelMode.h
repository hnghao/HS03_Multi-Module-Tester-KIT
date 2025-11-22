#ifndef NEOPIXEL_MODE_H
#define NEOPIXEL_MODE_H

#include <Adafruit_NeoPixel.h>

// Các biến/hàm dùng chung khai báo extern để dùng sang từ .ino
extern LiquidCrystal_I2C lcd;
extern AppState appState;
extern char headerLabel[16];

extern Adafruit_NeoPixel neoStrip;
extern unsigned long lastNeoUpdate;
extern const unsigned long NEOPIXEL_UPDATE_INTERVAL;

extern void lcdPrintLine(uint8_t row, const char *text);
extern void updateHeaderRow();

// Vị trí led hiện tại trong dải NeoPixel
static uint16_t neoCurrentPixel = 0;

// Pha màu hiện tại: 0 = Xanh lá, 1 = Đỏ, 2 = Xanh dương
static uint8_t neoColorPhase = 0;

// Hàm trả về màu theo pha
static uint32_t getPhaseColor() {
  switch (neoColorPhase) {
    case 0: // xanh lá
      return neoStrip.Color(0, 150, 0);
    case 1: // đỏ
      return neoStrip.Color(150, 0, 0);
    case 2: // xanh dương
    default:
      return neoStrip.Color(0, 0, 150);
  }
}

// Bắt đầu chế độ Neopixel
void startNeoPixelMode() {
  appState = STATE_NEOPIXEL;
  lcd.clear();

  strncpy(headerLabel, "Neopixel", sizeof(headerLabel));
  headerLabel[sizeof(headerLabel) - 1] = '\0';
  updateHeaderRow();

  lcdPrintLine(1, "WS2812 @GPIO5    ");
  lcdPrintLine(2, "Max 150 LEDs     ");
  lcdPrintLine(3, "Btn: Back        ");

  neoStrip.clear();
  neoStrip.show();

  neoCurrentPixel = 0;
  neoColorPhase   = 0;
  lastNeoUpdate   = 0;
}

// Cập nhật hiệu ứng: chạy từng led một, đổi qua 3 màu cơ bản
void updateNeoPixelMode(unsigned long now) {
  if (now - lastNeoUpdate < NEOPIXEL_UPDATE_INTERVAL) return;
  lastNeoUpdate = now;

  uint16_t count = neoStrip.numPixels();
  if (count == 0) return;
  if (neoCurrentPixel >= count) neoCurrentPixel = 0;

  // Tắt hết led trước
  neoStrip.clear();

  // Chọn màu theo pha (xanh lá, đỏ, xanh dương)
  uint32_t color = getPhaseColor();
  neoStrip.setPixelColor(neoCurrentPixel, color);
  neoStrip.show();

  // Sang pixel tiếp theo
  neoCurrentPixel++;
  if (neoCurrentPixel >= count) {
    neoCurrentPixel = 0;
    // Khi chạy hết dải -> đổi sang màu tiếp theo
    neoColorPhase = (neoColorPhase + 1) % 3;
  }
}

// Thoát chế độ Neopixel: tắt hết led
void stopNeoPixelMode() {
  neoStrip.clear();
  neoStrip.show();
}

#endif
