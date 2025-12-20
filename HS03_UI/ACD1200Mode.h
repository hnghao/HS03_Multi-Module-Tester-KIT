#ifndef ACD1200_MODE_H
#define ACD1200_MODE_H

#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// ===================== DÙNG CHUNG TRONG HS03_UI =====================
extern TwoWire I2CScanBus;         // bus sensor: SDA=GPIO1, SCL=GPIO2
extern LiquidCrystal_I2C lcd;

extern bool headerEnabled;         // để tắt header khi mode dùng full LCD 4 dòng

// ===================== ACD1200 CONFIG (GIỮ NGUYÊN HIỂN THỊ NHƯ CODE BẠN) =====================
static const uint8_t  ACD1200_ADDR          = 0x2A;    // 7-bit
static const uint16_t CMD_READ_CO2          = 0x0300;
static const uint16_t CMD_READ_FW_VER       = 0xD100; // 10 ASCII bytes
static const uint16_t CMD_READ_SENSOR_ID    = 0xD201; // 10 ASCII bytes

static const uint32_t ACD1200_READ_PERIOD_MS = 2000;  // chu kỳ đọc
static const uint16_t ACD1200_ACQ_DELAY_MS   = 80;    // delay sau khi gửi lệnh

static const uint32_t ACD1200_I2C_FREQ       = 100000; // <=100k

// ===================== LCD PRINT (GIỐNG HỆT CODE BẠN) =====================
static inline void acd1200_lcdPrintLine(uint8_t row, const char *text) {
  char buf[21];
  size_t n = 0;
  while (n < 20 && text[n] != '\0') { buf[n] = text[n]; n++; }
  while (n < 20) buf[n++] = ' ';
  buf[20] = '\0';
  lcd.setCursor(0, row);
  lcd.print(buf);
}

// ===================== CRC8 (poly 0x31 init 0xFF) =====================
static inline uint8_t acd1200_crc8_0x31(const uint8_t* data, size_t len) {
  uint8_t crc = 0xFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t b = 0; b < 8; b++) {
      if (crc & 0x80) crc = (uint8_t)((crc << 1) ^ 0x31);
      else           crc = (uint8_t)(crc << 1);
    }
  }
  return crc;
}

// ===================== I2C LOW LEVEL (dùng I2CScanBus) =====================
static inline bool acd1200_ping() {
  I2CScanBus.beginTransmission(ACD1200_ADDR);
  return (I2CScanBus.endTransmission(true) == 0);
}

static inline bool acd1200_writeCmd16(uint16_t cmd) {
  uint8_t hi = (uint8_t)(cmd >> 8);
  uint8_t lo = (uint8_t)(cmd & 0xFF);

  I2CScanBus.beginTransmission(ACD1200_ADDR);
  I2CScanBus.write(hi);
  I2CScanBus.write(lo);
  return (I2CScanBus.endTransmission(true) == 0);
}

static inline bool acd1200_readBytes(uint8_t *buf, size_t n) {
  size_t got = I2CScanBus.requestFrom((int)ACD1200_ADDR, (int)n, (int)true);
  if (got != n) return false;
  for (size_t i = 0; i < n; i++) {
    int c = I2CScanBus.read();
    if (c < 0) return false;
    buf[i] = (uint8_t)c;
  }
  return true;
}

static inline bool acd1200_readAscii10(uint16_t cmd, char out11[11]) {
  if (!acd1200_writeCmd16(cmd)) return false;
  delay(ACD1200_ACQ_DELAY_MS);

  uint8_t b[10];
  if (!acd1200_readBytes(b, sizeof(b))) return false;

  for (int i = 0; i < 10; i++) out11[i] = (char)b[i];
  out11[10] = '\0';
  return true;
}

// ===================== READ CO2 (GIỐNG HỆT CODE BẠN) =====================
static inline bool acd1200_readCO2(uint32_t &co2_ppm, uint16_t &temp_raw) {
  if (!acd1200_writeCmd16(CMD_READ_CO2)) return false;
  delay(ACD1200_ACQ_DELAY_MS);

  uint8_t d[9];
  if (!acd1200_readBytes(d, sizeof(d))) return false;

  // CRC check: 3 nhóm (2 byte + CRC)
  uint8_t crc1 = acd1200_crc8_0x31(&d[0], 2);
  uint8_t crc2 = acd1200_crc8_0x31(&d[3], 2);
  uint8_t crc3 = acd1200_crc8_0x31(&d[6], 2);
  if (crc1 != d[2] || crc2 != d[5] || crc3 != d[8]) return false;

  co2_ppm = ((uint32_t)d[0] << 24) | ((uint32_t)d[1] << 16) |
            ((uint32_t)d[3] << 8)  |  (uint32_t)d[4];

  temp_raw = ((uint16_t)d[6] << 8) | (uint16_t)d[7];
  return true;
}

// ===================== STATE =====================
static bool          acd1200_savedHeaderEnabled_ = true;
static uint32_t      acd1200_tLast_              = 0;
static char          acd1200_fw_[11]             = "----------";
static char          acd1200_id_[11]             = "----------";
static uint16_t      acd1200_failCount_          = 0;

// ===================== API cho HS03_UI =====================
static inline void startACD1200Mode() {
  // Mode dùng full LCD 4 dòng -> tắt header
  acd1200_savedHeaderEnabled_ = headerEnabled;
  headerEnabled = false;

  // Đảm bảo bus I2C sensor đang đúng GPIO1/2 và 100k
  I2CScanBus.begin(I2C_SCAN_SDA_PIN, I2C_SCAN_SCL_PIN, ACD1200_I2C_FREQ);
  I2CScanBus.setClock(ACD1200_I2C_FREQ);
  I2CScanBus.setTimeOut(150);

  lcd.clear();
  acd1200_lcdPrintLine(0, "ACD1200 CO2 (I2C)");
  acd1200_lcdPrintLine(1, "LCD:GPIO6/7");
  acd1200_lcdPrintLine(2, "CO2:GPIO1/2");
  acd1200_lcdPrintLine(3, "Init...");
  delay(1200);

  // Nếu có cảm biến thì đọc FW/ID 1 lần để hiển thị cố định
  if (acd1200_ping()) {
    (void)acd1200_readAscii10(CMD_READ_FW_VER, acd1200_fw_);
    (void)acd1200_readAscii10(CMD_READ_SENSOR_ID, acd1200_id_);
  } else {
    // báo lỗi gọn
    acd1200_lcdPrintLine(0, "ACD1200 NOT FOUND");
    acd1200_lcdPrintLine(1, "Check SDA=GPIO1");
    acd1200_lcdPrintLine(2, "Check SCL=GPIO2");
    acd1200_lcdPrintLine(3, "SET must FLOAT");
    delay(1500);
  }

  acd1200_tLast_ = millis();
  acd1200_failCount_ = 0;
}

static inline void updateACD1200Mode(unsigned long now) {
  if (now - acd1200_tLast_ < ACD1200_READ_PERIOD_MS) return;
  acd1200_tLast_ = now;

  if (!acd1200_ping()) {
    acd1200_lcdPrintLine(0, "ACD1200 DISCONNECT");
    acd1200_lcdPrintLine(1, "Retry...");
    acd1200_lcdPrintLine(2, "SDA1=1 SCL1=2");
    acd1200_lcdPrintLine(3, "SET=FLOAT (I2C)");
    delay(200);
    return;
  }

  uint32_t co2ppm = 0;
  uint16_t tempRaw = 0;

  if (acd1200_readCO2(co2ppm, tempRaw)) {
    acd1200_failCount_ = 0;

    char line0[21], line1[21], line2[21], line3[21];
    snprintf(line0, sizeof(line0), "CO2:%5lu ppm", (unsigned long)co2ppm);
    snprintf(line1, sizeof(line1), "TEMP(raw): 0x%04X", tempRaw);
    snprintf(line2, sizeof(line2), "FW:%-10s", acd1200_fw_);
    snprintf(line3, sizeof(line3), "ID:%-10s", acd1200_id_);

    acd1200_lcdPrintLine(0, line0);
    acd1200_lcdPrintLine(1, line1);
    acd1200_lcdPrintLine(2, line2);
    acd1200_lcdPrintLine(3, line3);
  } else {
    acd1200_failCount_++;
    acd1200_lcdPrintLine(0, "ACD1200 READ FAILED");
    acd1200_lcdPrintLine(1, "I2C/CRC error");
    char line2[21];
    snprintf(line2, sizeof(line2), "Fail count: %u", acd1200_failCount_);
    acd1200_lcdPrintLine(2, line2);
    acd1200_lcdPrintLine(3, "Check wiring/level");
  }
}

static inline void stopACD1200Mode() {
  // trả lại header cho các mode khác
  headerEnabled = acd1200_savedHeaderEnabled_;
}

#endif // ACD1200_MODE_H
