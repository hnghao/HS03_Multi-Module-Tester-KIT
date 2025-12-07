#ifndef I2C_SCAN_MODE_H
#define I2C_SCAN_MODE_H

#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// Các biến/ hàm được định nghĩa trong HS03_UI.ino / Display.h
extern LiquidCrystal_I2C lcd;
extern TwoWire I2CScanBus;

extern char headerLabel[16];
void updateHeaderRow();
void lcdPrintLine(uint8_t row, const char* text);

// Enum AppState được khai báo trong HS03_UI.ino
extern AppState appState;

// ========================
// Biến nội bộ cho I2C Scan
// ========================
static bool     i2cScanDone         = false;
static uint8_t  i2cFoundAddrs[32];
static uint8_t  i2cFoundCount       = 0;

static char     i2cAddrLine[128];    // Chuỗi "0x27 0x3F 0x48..."
static char     i2cScrollBuf[192];   // Chuỗi dùng cho hiệu ứng chạy chữ
static uint16_t i2cScrollLen        = 0;
static uint16_t i2cScrollIndex      = 0;
static unsigned long lastI2CScrollMillis = 0;

// Thời gian giữa 2 bước scroll (ms)
static const unsigned long I2C_SCROLL_INTERVAL = 400;

// Nếu muốn tự động quét lại sau vài giây, dùng thêm 2 biến này (hiện đang tắt)
static unsigned long lastI2CScanMillis = 0;
static const unsigned long I2C_SCAN_INTERVAL = 3000UL;

// Địa chỉ ADS1115 cần ẨN khỏi kết quả scan (ADDR nối GND -> 0x48)
static const uint8_t ADS1115_SKIP_ADDR = 0x48;

// ========================
// Hàm thực hiện quét I2C
// ========================
inline void performI2CScan() {
  i2cFoundCount = 0;
  i2cAddrLine[0] = '\0';

  // Duyệt địa chỉ 1..126 (0 và 127 thường không dùng cho thiết bị)
  for (uint8_t address = 1; address < 127; address++) {
    // BỎ QUA địa chỉ ADS1115 để không hiện trong danh sách
    if (address == ADS1115_SKIP_ADDR) {
      continue;
    }

    I2CScanBus.beginTransmission(address);
    uint8_t error = I2CScanBus.endTransmission();
    if (error == 0) {
      if (i2cFoundCount < (uint8_t)(sizeof(i2cFoundAddrs) / sizeof(i2cFoundAddrs[0]))) {
        i2cFoundAddrs[i2cFoundCount++] = address;
      }
    }
  }

  // Ghép chuỗi địa chỉ: "0x27 0x3F 0x48 ..."
  if (i2cFoundCount == 0) {
    strncpy(i2cAddrLine, "-- none --", sizeof(i2cAddrLine));
    i2cAddrLine[sizeof(i2cAddrLine) - 1] = '\0';
  } else {
    i2cAddrLine[0] = '\0';
    for (uint8_t i = 0; i < i2cFoundCount; i++) {
      char buf[8];
      snprintf(buf, sizeof(buf), "0x%02X", i2cFoundAddrs[i]);

      size_t curLen = strlen(i2cAddrLine);
      if (curLen + strlen(buf) + 2 >= sizeof(i2cAddrLine)) {
        // Hết chỗ -> dừng thêm để tránh tràn bộ đệm
        break;
      }

      if (curLen > 0) {
        strcat(i2cAddrLine, " ");
      }
      strcat(i2cAddrLine, buf);
    }
  }

  // Xây scroll buffer nếu có từ 4 thiết bị trở lên
  if (i2cFoundCount >= 4) {
    // 20 khoảng trắng bên trái + chuỗi địa chỉ + 3 khoảng trắng bên phải
    const char* prefix = "                    "; // 20 spaces
    const char* suffix = "   ";

    i2cScrollBuf[0] = '\0';
    strncat(i2cScrollBuf, prefix, sizeof(i2cScrollBuf) - 1);
    strncat(i2cScrollBuf, i2cAddrLine, sizeof(i2cScrollBuf) - strlen(i2cScrollBuf) - 1);
    strncat(i2cScrollBuf, suffix, sizeof(i2cScrollBuf) - strlen(i2cScrollBuf) - 1);

    i2cScrollLen   = strlen(i2cScrollBuf);
    i2cScrollIndex = 0;
  } else {
    i2cScrollBuf[0] = '\0';
    i2cScrollLen    = 0;
    i2cScrollIndex  = 0;
  }

  // Hiển thị kết quả quét
  char line1[21];
  snprintf(line1, sizeof(line1), "Found: %2u device", i2cFoundCount);
  if (i2cFoundCount != 1) {
    // Thêm 's' nếu nhiều hơn 1
    size_t l = strlen(line1);
    if (l < sizeof(line1) - 1) {
      line1[l] = 's';
      line1[l + 1] = '\0';
    }
  }
  lcdPrintLine(1, line1);

  // Dòng 2: địa chỉ I2C
  if (i2cFoundCount == 0) {
    lcdPrintLine(2, "Addr: none        ");
  } else if (i2cFoundCount < 4) {
    // Không cần scroll, in thẳng
    lcdPrintLine(2, i2cAddrLine);
  } else {
    // Có từ 4 địa chỉ trở lên, sẽ chạy chữ (frame đầu tiên)
    char window[21];
    if (i2cScrollLen <= 20) {
      strncpy(window, i2cScrollBuf, 20);
      window[20] = '\0';
    } else {
      memcpy(window, i2cScrollBuf, 20);
      window[20] = '\0';
    }
    lcdPrintLine(2, window);
  }

  // Dòng 3: hướng dẫn
  lcdPrintLine(3, "Nhan nut de Back  ");

  i2cScanDone         = true;
  lastI2CScanMillis   = millis();
  lastI2CScrollMillis = millis();
}

// =========================
// API gọi từ HS03_UI.ino
// =========================

// Được gọi khi chọn "Scan" trong I2C Submenu
inline void startI2CScanMode() {
  appState = STATE_I2C_SCAN;

  // Cập nhật header
  strncpy(headerLabel, "I2C Scan", sizeof(headerLabel));
  headerLabel[sizeof(headerLabel) - 1] = '\0';
  updateHeaderRow();

  // Thông báo đang quét
  lcdPrintLine(1, "Dang quet I2C ... ");
  lcdPrintLine(2, "                  ");
  lcdPrintLine(3, "Vui long cho ...  ");

  // Reset biến trạng thái
  i2cScanDone          = false;
  i2cFoundCount        = 0;
  i2cAddrLine[0]       = '\0';
  i2cScrollBuf[0]      = '\0';
  i2cScrollLen         = 0;
  i2cScrollIndex       = 0;
  lastI2CScanMillis    = 0;
  lastI2CScrollMillis  = 0;

  // Quét ngay một lần khi vào mode
  performI2CScan();
}

// Được gọi liên tục trong loop() khi appState == STATE_I2C_SCAN
inline void updateI2CScanMode(unsigned long now) {
  // Nếu sau này muốn quét lại định kỳ, có thể bật đoạn này:
  /*
  if (now - lastI2CScanMillis > I2C_SCAN_INTERVAL) {
    performI2CScan();
    lastI2CScanMillis = now;
  }
  */

  // Chỉ cần xử lý scroll nếu có từ 4 địa chỉ trở lên
  if (i2cFoundCount >= 4 && i2cScrollLen > 20) {
    if (now - lastI2CScrollMillis >= I2C_SCROLL_INTERVAL) {
      lastI2CScrollMillis = now;

      char window[21];
      for (uint8_t i = 0; i < 20; i++) {
        uint16_t idx = i2cScrollIndex + i;
        if (idx >= i2cScrollLen) {
          idx -= i2cScrollLen; // quay vòng
        }
        window[i] = i2cScrollBuf[idx];
      }
      window[20] = '\0';

      lcdPrintLine(2, window);

      // Tăng index để chạy chữ
      i2cScrollIndex++;
      if (i2cScrollIndex >= i2cScrollLen) {
        i2cScrollIndex = 0;
      }
    }
  }
}

#endif // I2C_SCAN_MODE_H
