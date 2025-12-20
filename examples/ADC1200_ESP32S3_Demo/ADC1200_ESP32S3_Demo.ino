#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// ===================== PIN MAP =====================
// LCD2004 I2C bus (Wire)
static const int LCD_SDA = 6;
static const int LCD_SCL = 7;

// CO2 sensor I2C bus (TwoWire(1))
static const int CO2_SDA = 1;
static const int CO2_SCL = 2;

// ===================== I2C FREQ =====================
static const uint32_t LCD_I2C_FREQ = 100000;  // LCD 100k
static const uint32_t CO2_I2C_FREQ = 100000;  // ACD1200 <= 100k

// ===================== LCD CONFIG =====================
static const uint8_t LCD_ADDR = 0x27;         // đổi 0x3F nếu LCD của bạn dùng địa chỉ đó
LiquidCrystal_I2C lcd(LCD_ADDR, 20, 4);

// ===================== ACD1200 CONFIG =====================
static const uint8_t  ACD1200_ADDR = 0x2A;    // 7-bit
static const uint16_t CMD_READ_CO2       = 0x0300;
static const uint16_t CMD_READ_FW_VER    = 0xD100; // 10 ASCII bytes
static const uint16_t CMD_READ_SENSOR_ID = 0xD201; // 10 ASCII bytes

static const uint32_t READ_PERIOD_MS = 2000;  // chu kỳ đọc
static const uint16_t ACQ_DELAY_MS   = 80;    // delay sau khi gửi lệnh

TwoWire CO2Bus(1);

// ===================== LCD HELPERS =====================
static void lcdPrintLine(uint8_t row, const char *text) {
  char buf[21];
  size_t n = 0;
  while (n < 20 && text[n] != '\0') { buf[n] = text[n]; n++; }
  while (n < 20) buf[n++] = ' ';
  buf[20] = '\0';
  lcd.setCursor(0, row);
  lcd.print(buf);
}

// ===================== CRC8 (poly 0x31 init 0xFF) =====================
static uint8_t crc8_0x31(const uint8_t* data, size_t len) {
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

// ===================== I2C LOW LEVEL =====================
static bool co2Ping() {
  CO2Bus.beginTransmission(ACD1200_ADDR);
  return (CO2Bus.endTransmission() == 0);
}

static bool co2WriteCmd16(uint16_t cmd) {
  uint8_t hi = (uint8_t)(cmd >> 8);
  uint8_t lo = (uint8_t)(cmd & 0xFF);

  CO2Bus.beginTransmission(ACD1200_ADDR);
  CO2Bus.write(hi);
  CO2Bus.write(lo);
  return (CO2Bus.endTransmission() == 0);
}

static bool co2ReadBytes(uint8_t *buf, size_t n) {
  size_t got = CO2Bus.requestFrom((int)ACD1200_ADDR, (int)n);
  if (got != n) return false;
  for (size_t i = 0; i < n; i++) {
    int c = CO2Bus.read();
    if (c < 0) return false;
    buf[i] = (uint8_t)c;
  }
  return true;
}

static bool co2ReadAscii10(uint16_t cmd, char out11[11]) {
  if (!co2WriteCmd16(cmd)) return false;
  delay(ACQ_DELAY_MS);

  uint8_t b[10];
  if (!co2ReadBytes(b, sizeof(b))) return false;

  for (int i = 0; i < 10; i++) out11[i] = (char)b[i];
  out11[10] = '\0';
  return true;
}

// ===================== READ CO2 =====================
static bool readCO2(uint32_t &co2_ppm, uint16_t &temp_raw) {
  if (!co2WriteCmd16(CMD_READ_CO2)) return false;
  delay(ACQ_DELAY_MS);

  uint8_t d[9];
  if (!co2ReadBytes(d, sizeof(d))) return false;

  // CRC check: 3 nhóm (2 byte + CRC)
  uint8_t crc1 = crc8_0x31(&d[0], 2);
  uint8_t crc2 = crc8_0x31(&d[3], 2);
  uint8_t crc3 = crc8_0x31(&d[6], 2);
  if (crc1 != d[2] || crc2 != d[5] || crc3 != d[8]) return false;

  co2_ppm = ((uint32_t)d[0] << 24) | ((uint32_t)d[1] << 16) |
            ((uint32_t)d[3] << 8)  |  (uint32_t)d[4];

  temp_raw = ((uint16_t)d[6] << 8) | (uint16_t)d[7];
  return true;
}

// ===================== APP =====================
static uint32_t tLast = 0;
static char fw[11] = "----------";
static char id[11] = "----------";
static uint16_t failCount = 0;

void setup() {
  // LCD bus
  Wire.begin(LCD_SDA, LCD_SCL, LCD_I2C_FREQ);
  lcd.init();
  lcd.backlight();
  lcd.clear();

  lcdPrintLine(0, "ACD1200 CO2 (I2C)");
  lcdPrintLine(1, "LCD:GPIO6/7");
  lcdPrintLine(2, "CO2:GPIO1/2");
  lcdPrintLine(3, "Init...");
  delay(1200);

  // CO2 bus
  CO2Bus.begin(CO2_SDA, CO2_SCL, CO2_I2C_FREQ);

  // Nếu có cảm biến thì đọc FW/ID 1 lần để hiển thị cố định
  if (co2Ping()) {
    (void)co2ReadAscii10(CMD_READ_FW_VER, fw);
    (void)co2ReadAscii10(CMD_READ_SENSOR_ID, id);
  } else {
    // báo lỗi gọn
    lcdPrintLine(0, "ACD1200 NOT FOUND");
    lcdPrintLine(1, "Check SDA=GPIO1");
    lcdPrintLine(2, "Check SCL=GPIO2");
    lcdPrintLine(3, "SET must FLOAT");
    delay(1500);
  }

  tLast = millis();
}

void loop() {
  if (millis() - tLast < READ_PERIOD_MS) return;
  tLast = millis();

  if (!co2Ping()) {
    lcdPrintLine(0, "ACD1200 DISCONNECT");
    lcdPrintLine(1, "Retry...");
    lcdPrintLine(2, "SDA1=1 SCL1=2");
    lcdPrintLine(3, "SET=FLOAT (I2C)");
    delay(200);
    return;
  }

  uint32_t co2ppm = 0;
  uint16_t tempRaw = 0;

  if (readCO2(co2ppm, tempRaw)) {
    failCount = 0;

    char line0[21], line1[21], line2[21], line3[21];
    snprintf(line0, sizeof(line0), "CO2:%5lu ppm", (unsigned long)co2ppm);
    snprintf(line1, sizeof(line1), "TEMP(raw): 0x%04X", tempRaw);
    snprintf(line2, sizeof(line2), "FW:%-10s", fw);
    snprintf(line3, sizeof(line3), "ID:%-10s", id);

    lcdPrintLine(0, line0);
    lcdPrintLine(1, line1);
    lcdPrintLine(2, line2);
    lcdPrintLine(3, line3);
  } else {
    failCount++;
    lcdPrintLine(0, "ACD1200 READ FAILED");
    lcdPrintLine(1, "I2C/CRC error");
    char line2[21];
    snprintf(line2, sizeof(line2), "Fail count: %u", failCount);
    lcdPrintLine(2, line2);
    lcdPrintLine(3, "Check wiring/level");
  }
}
