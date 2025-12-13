#ifndef MATRIX_LED_MODE_H
#define MATRIX_LED_MODE_H

#include <Arduino.h>
#include <LedControl.h>
#include <MD_MAX72xx.h>
#include <SPI.h>

// LedControl matrixLc được định nghĩa ở HS03_UI.ino
extern LedControl matrixLc;

// ======================
// Khởi tạo / tắt MAX7219 bằng LedControl
// ======================

// Dùng cho bước khởi tạo ban đầu trong setup()
inline void matrixInitDevices() {
  for (uint8_t dev = 0; dev < MATRIX_DEVICE_COUNT; dev++) {
    matrixLc.shutdown(dev, false);          // bật chip
    matrixLc.setIntensity(dev, 8);          // độ sáng trung bình 0..15
    matrixLc.clearDisplay(dev);             // xóa toàn bộ
  }
}

// Tắt hoàn toàn tất cả MAX7219 (tiết kiệm năng lượng)
inline void matrixShutdownAll() {
  for (uint8_t dev = 0; dev < MATRIX_DEVICE_COUNT; dev++) {
    matrixLc.shutdown(dev, true);           // shutdown = OFF
  }
}

// ======================
// MODE 8x32 (MD_MAX72XX, hiệu ứng mưa)
// ======================

// Đa số module MAX7219 8x8, 8x32 đều là kiểu FC16
#define MATRIX_HARDWARE_TYPE  MD_MAX72XX::FC16_HW

// Các macro MATRIX_DIN_PIN, MATRIX_CLK_PIN, MATRIX_CS_PIN, MATRIX_DEVICE_COUNT
// được định nghĩa ở HS03_UI.ino
static MD_MAX72XX matrixMx(
  MATRIX_HARDWARE_TYPE,
  MATRIX_DIN_PIN,
  MATRIX_CLK_PIN,
  MATRIX_CS_PIN,
  MATRIX_DEVICE_COUNT
);

// Kích thước ma trận
static const uint8_t MATRIX32_HEIGHT = 8;                        // 8 hàng
static const uint8_t MATRIX32_WIDTH  = 8 * MATRIX_DEVICE_COUNT;  // 8 hoặc 32 cột tùy số module

// Cấu hình hiệu ứng mưa
static const uint16_t MATRIX32_UPDATE_INTERVAL = 70;   // ms giữa mỗi bước rơi
static const uint8_t  MATRIX32_TAIL_LENGTH     = 4;    // độ dài vệt mưa (đuôi)
static const uint8_t  MATRIX32_START_PROB      = 30;   // % xác suất xuất hiện giọt mới trên mỗi cột

// Thời gian bật toàn bộ led khi mới vào mode (intro)
static const uint16_t MATRIX32_INTRO_DURATION  = 2000; // 2 giây

// headY[col] = vị trí “đầu giọt mưa” trên cột đó (-1 nghĩa là cột đang không có giọt mưa)
static int8_t        matrix32_headY[MATRIX32_WIDTH];
static unsigned long matrix32_lastUpdate   = 0;
static unsigned long matrix32_introStart   = 0;
static bool          matrix32_introPhase   = true;

// Một bước cập nhật hiệu ứng mưa
inline void matrix8x32_rainStep() {
  // Mỗi frame vẽ lại từ đầu
  matrixMx.clear();

  for (uint8_t col = 0; col < MATRIX32_WIDTH; col++) {
    // Nếu cột đang có giọt mưa -> cho rơi xuống thêm 1 hàng
    if (matrix32_headY[col] >= 0) {
      matrix32_headY[col]++;
    }

    // Nếu giọt mưa đã rơi xuống dưới cả đuôi thì reset cột
    if (matrix32_headY[col] - (int8_t)MATRIX32_TAIL_LENGTH >= MATRIX32_HEIGHT) {
      matrix32_headY[col] = -1;
    }

    // Nếu không có giọt mưa ở cột này -> random tạo giọt mới
    if (matrix32_headY[col] < 0) {
      if (random(100) < MATRIX32_START_PROB) {
        matrix32_headY[col] = 0;  // bắt đầu từ hàng trên cùng
      }
    }

    // Vẽ giọt mưa + đuôi cho cột này
    if (matrix32_headY[col] >= 0) {
      for (uint8_t t = 0; t < MATRIX32_TAIL_LENGTH; t++) {
        int8_t y = matrix32_headY[col] - t;  // các điểm phía trên “đầu” tạo thành vệt
        if (y >= 0 && y < (int8_t)MATRIX32_HEIGHT) {
          // setPoint(row, col, trạng_thái)
          matrixMx.setPoint((uint8_t)y, col, true);
        }
      }
    }
  }
}

// ======================
// Khởi động mode Matrix 8x32
// ======================
inline void startMatrix8x32Mode() {
  // Khởi tạo driver MD_MAX72XX (Soft-SPI)
  matrixMx.begin();
  matrixMx.control(MD_MAX72XX::INTENSITY, 2);  // Độ sáng 0..15 (tùy chỉnh)
  matrixMx.clear();

  // Khởi tạo random
  randomSeed(analogRead(0));

  // Ban đầu không có giọt mưa nào
  for (uint8_t col = 0; col < MATRIX32_WIDTH; col++) {
    matrix32_headY[col] = -1;
  }

  // Đánh dấu đang ở phase intro: bật toàn bộ led trong 2s
  matrix32_introPhase = true;
  matrix32_introStart = millis();

  // Chuẩn bị biến thời gian cho phase mưa
  matrix32_lastUpdate = millis();
}

// ======================
// Cập nhật mode Matrix 8x32 (gọi trong loop khi STATE_MATRIX_8X32)
// ======================
inline void updateMatrix8x32Mode() {
  unsigned long now = millis();

  // Phase 1: mới vào mode -> bật toàn bộ led trong 2s
  if (matrix32_introPhase) {
    // Bật tất cả điểm (hỗ trợ cả 1 module 8x8 hoặc 4 module 8x8)
    for (uint8_t row = 0; row < MATRIX32_HEIGHT; row++) {
      for (uint8_t col = 0; col < MATRIX32_WIDTH; col++) {
        matrixMx.setPoint(row, col, true);
      }
    }

    // Nếu chưa hết 2s thì chỉ giữ nguyên pattern "full on"
    if (now - matrix32_introStart < MATRIX32_INTRO_DURATION) {
      return;
    }

    // Hết 2s -> chuyển sang phase mưa
    matrix32_introPhase = false;
    matrix32_lastUpdate = now;   // bắt đầu đếm thời gian cho hiệu ứng mưa
    matrixMx.clear();            // clear 1 lần trước khi vào mưa
    return;                      // frame này chỉ dùng để chuyển trạng thái
  }

  // Phase 2: hiệu ứng mưa rơi
  if (now - matrix32_lastUpdate < MATRIX32_UPDATE_INTERVAL) return;
  matrix32_lastUpdate = now;

  matrix8x32_rainStep();
}

// Dừng tất cả hiệu ứng + tắt luôn chip để tiết kiệm
inline void stopMatrixLedMode() {
  matrixMx.clear();     // clear bằng MD_MAX72XX (8x32 hoặc 8x8)
  matrixShutdownAll();  // shutdown toàn bộ MAX7219 qua LedControl
}

#endif  // MATRIX_LED_MODE_H
