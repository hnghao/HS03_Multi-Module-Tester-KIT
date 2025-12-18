#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// ===== AGR12 =====
#define AGR12_ADDR  0x50
#define CMD_REG     0xAC
#define CMD_MEASURE 0x12

// Chu kỳ đọc & thời gian đo
static const uint32_t READ_INTERVAL_MS = 200;  // mỗi 1 giây đọc 1 lần
static const uint32_t MEASURE_TIME_MS  = 150;  // thời gian cảm biến đo
static const uint8_t  RETRY_MAX        = 3;
static const uint8_t  FILTER_SIZE      = 5;

// Chân I2C (bit-bang) - AGR12 trên ESP32-S3
static const uint8_t AGR_SDA_PIN = 1;  // GPIO1 = SDA
static const uint8_t AGR_SCL_PIN = 2;  // GPIO2 = SCL

// ===== LCD2004 I2C (hardware I2C) =====
static const uint8_t LCD_SDA_PIN = 6;  // GPIO6 = SDA
static const uint8_t LCD_SCL_PIN = 7;  // GPIO7 = SCL
static const uint8_t LCD_COLS = 20;
static const uint8_t LCD_ROWS = 4;

LiquidCrystal_I2C* lcd = nullptr;

// ====== Filter ======
float pressureBuf[FILTER_SIZE];
uint8_t bufIndex = 0;
bool bufFull = false;
float lastPressure = 0;

// ===== Soft I2C (open-drain) =====
static inline void SDA_low() { pinMode(AGR_SDA_PIN, OUTPUT); digitalWrite(AGR_SDA_PIN, LOW); }
static inline void SDA_rel() { pinMode(AGR_SDA_PIN, INPUT_PULLUP); }
static inline void SCL_low() { pinMode(AGR_SCL_PIN, OUTPUT); digitalWrite(AGR_SCL_PIN, LOW); }
static inline void SCL_rel() { pinMode(AGR_SCL_PIN, INPUT_PULLUP); }

static inline void i2c_delay() { delayMicroseconds(6); }

static bool SCL_rel_wait(uint16_t timeout_us = 2000) {
  SCL_rel();
  while (digitalRead(AGR_SCL_PIN) == LOW && timeout_us--) delayMicroseconds(1);
  return (digitalRead(AGR_SCL_PIN) == HIGH);
}

static void i2c_start() {
  SDA_rel(); SCL_rel_wait();
  i2c_delay();
  SDA_low();
  i2c_delay();
  SCL_low();
  i2c_delay();
}

static void i2c_stop() {
  SDA_low();
  i2c_delay();
  SCL_rel_wait();
  i2c_delay();
  SDA_rel();
  i2c_delay();
}

static bool i2c_writeByte(uint8_t data) {
  for (uint8_t i = 0; i < 8; i++) {
    if (data & 0x80) SDA_rel(); else SDA_low();
    i2c_delay();
    if (!SCL_rel_wait()) return false;
    i2c_delay();
    SCL_low();
    i2c_delay();
    data <<= 1;
  }
  SDA_rel();
  i2c_delay();
  if (!SCL_rel_wait()) return false;
  i2c_delay();
  bool ack = (digitalRead(AGR_SDA_PIN) == LOW);
  SCL_low();
  i2c_delay();
  return ack;
}

static uint8_t i2c_readByte(bool ack) {
  uint8_t data = 0;
  SDA_rel();
  for (uint8_t i = 0; i < 8; i++) {
    data <<= 1;
    i2c_delay();
    SCL_rel_wait();
    i2c_delay();
    if (digitalRead(AGR_SDA_PIN)) data |= 1;
    SCL_low();
    i2c_delay();
  }
  if (ack) SDA_low(); else SDA_rel();
  i2c_delay();
  SCL_rel_wait();
  i2c_delay();
  SCL_low();
  i2c_delay();
  SDA_rel();
  return data;
}

static uint8_t xorChecksum2(const uint8_t *buf, uint8_t len) {
  uint8_t cs = 0;
  for (uint8_t i = 0; i < len; i++) cs ^= buf[i];
  return cs;
}

// ====== Trigger + Read data ======
static bool agr12Trigger() {
  i2c_start();
  if (!i2c_writeByte((AGR12_ADDR << 1) | 0)) { i2c_stop(); return false; }
  if (!i2c_writeByte(CMD_REG))              { i2c_stop(); return false; }
  if (!i2c_writeByte(CMD_MEASURE))          { i2c_stop(); return false; }
  i2c_stop();
  return true;
}

static bool agr12ReadData(float &out_kPa) {
  uint8_t rx[3];

  i2c_start();
  if (!i2c_writeByte((AGR12_ADDR << 1) | 1)) { i2c_stop(); return false; }
  rx[0] = i2c_readByte(true);
  rx[1] = i2c_readByte(true);
  rx[2] = i2c_readByte(false);
  i2c_stop();

  if (xorChecksum2(rx, 2) != rx[2]) return false;

  int16_t raw = (int16_t)((uint16_t(rx[0]) << 8) | rx[1]);
  out_kPa = raw / 10.0f;
  return true;
}

static float filterPressure(float val) {
  pressureBuf[bufIndex++] = val;
  if (bufIndex >= FILTER_SIZE) {
    bufIndex = 0;
    bufFull = true;
  }
  uint8_t count = bufFull ? FILTER_SIZE : bufIndex;
  float sum = 0;
  for (uint8_t i = 0; i < count; i++) sum += pressureBuf[i];
  return sum / count;
}

// ===== LCD helpers =====
static uint8_t findLcdAddress() {
  // quét nhanh địa chỉ I2C
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) return addr;
  }
  return 0; // không thấy
}

static void lcdPrintLine(uint8_t row, const char* text) {
  if (!lcd) return;
  char buf[LCD_COLS + 1];
  uint8_t i = 0;
  for (; i < LCD_COLS && text[i]; i++) buf[i] = text[i];
  for (; i < LCD_COLS; i++) buf[i] = ' ';
  buf[LCD_COLS] = '\0';
  lcd->setCursor(0, row);
  lcd->print(buf);
}

static void lcdShowPressure(float pFilt) {
  // Line0: tiêu đề
  lcdPrintLine(0, "AGR12 Soft-I2C mode");

  // Line1: "Ap suat:"
  lcdPrintLine(1, "Ap suat:");

  // Line2: "xx.x kPa"
  char line2[21];
  char num[12];
  dtostrf(pFilt, 0, 1, num); // 1 số lẻ
  snprintf(line2, sizeof(line2), "%s kPa", num);
  lcdPrintLine(2, line2);

  // Line3: trống / trạng thái
  lcdPrintLine(3, "");
}

static void lcdShowHoldValue(const char* reason, float hold_kPa) {
  // reason: "Loi trigger" hoặc "Loi doc"
  lcdPrintLine(0, "AGR12 Soft-I2C mode");
  lcdPrintLine(1, reason);
  lcdPrintLine(2, "Giu gia tri cu:");

  char line3[21];
  char num[12];
  dtostrf(hold_kPa, 0, 1, num);
  snprintf(line3, sizeof(line3), "%s kPa", num);
  lcdPrintLine(3, line3);
}

// ====== FSM millis() ======
enum ReadState : uint8_t { IDLE, WAIT_MEASURE, READ_TRY };
ReadState state = IDLE;

uint32_t tNextCycle = 0;
uint32_t tMeasureStart = 0;
uint8_t retryLeft = 0;

void setup() {
  // --- LCD I2C ---
  Wire.begin(LCD_SDA_PIN, LCD_SCL_PIN);
  Wire.setClock(100000);

  uint8_t addr = findLcdAddress();
  if (addr == 0) addr = 0x27; // fallback phổ biến

  lcd = new LiquidCrystal_I2C(addr, LCD_COLS, LCD_ROWS);
  lcd->init();
  lcd->backlight();
  lcd->clear();

  // --- AGR12 soft I2C ---
  SDA_rel();
  SCL_rel();
  delay(30);

  lcdPrintLine(0, "AGR12 Soft-I2C mode");
  lcdPrintLine(1, "Dang khoi dong...");
  lcdPrintLine(2, "");
  lcdPrintLine(3, "");

  tNextCycle = millis(); // chạy ngay lần đầu
}

void loop() {
  uint32_t now = millis();

  switch (state) {
    case IDLE:
      if ((int32_t)(now - tNextCycle) >= 0) {
        retryLeft = RETRY_MAX;

        // Trigger đo
        if (agr12Trigger()) {
          tMeasureStart = now;
          state = WAIT_MEASURE;
        } else {
          // Nếu trigger fail -> hẹn vòng sau (giữ giá trị cũ)
          lcdShowHoldValue("Loi trigger", lastPressure);
          tNextCycle = now + READ_INTERVAL_MS;
        }
      }
      break;

    case WAIT_MEASURE:
      // chờ đủ thời gian đo bằng millis (không delay)
      if ((uint32_t)(now - tMeasureStart) >= MEASURE_TIME_MS) {
        state = READ_TRY;
      }
      break;

    case READ_TRY: {
      float p;
      if (agr12ReadData(p)) {
        lastPressure = p;
        float pFilt = filterPressure(p);

        // Hiển thị lên LCD thay cho Serial
        lcdShowPressure(pFilt);

        tNextCycle = now + READ_INTERVAL_MS;
        state = IDLE;
      } else {
        // retry “nhanh” nhưng vẫn không block loop
        if (retryLeft > 0) retryLeft--;

        if (retryLeft == 0) {
          lcdShowHoldValue("Loi doc", lastPressure);

          tNextCycle = now + READ_INTERVAL_MS;
          state = IDLE;
        } else {
          // Thử lại: trigger lại -> chờ lại
          if (agr12Trigger()) {
            tMeasureStart = now;
            state = WAIT_MEASURE;
          } else {
            if (retryLeft == 0) {
              tNextCycle = now + READ_INTERVAL_MS;
              state = IDLE;
            }
          }
        }
      }
    } break;
  }

  // Ở đây bạn có thể thêm tác vụ khác (UI, đọc nút, LCD...) mà không bị delay chặn.
}
