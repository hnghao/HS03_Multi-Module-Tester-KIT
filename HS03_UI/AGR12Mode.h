#ifndef AGR12_MODE_H
#define AGR12_MODE_H

#include <Arduino.h>
#include <Wire.h>

// Dùng LCD + helper sẵn có của project
class LiquidCrystal_I2C;
extern LiquidCrystal_I2C lcd;
extern void lcdPrintLine(uint8_t row, const char* text);
extern bool headerEnabled;

// Nếu project có I2CScanBus thì restore lại sau khi thoát mode
extern TwoWire I2CScanBus;

#ifndef I2C_SCAN_SDA_PIN
#define I2C_SCAN_SDA_PIN 1
#endif
#ifndef I2C_SCAN_SCL_PIN
#define I2C_SCAN_SCL_PIN 2
#endif

// ===== AGR12 constants (đặt tên UNIQUE để không đụng header khác) =====
static const uint8_t  AGR12_ADDR_         = 0x50;
static const uint8_t  AGR12_CMD_REG_      = 0xAC;
static const uint8_t  AGR12_CMD_MEASURE_  = 0x12;

static const uint32_t AGR12_READ_INTERVAL_MS_ = 200;  // giữ nguyên như code bạn gửi
static const uint32_t AGR12_MEASURE_TIME_MS_  = 150;
static const uint8_t  AGR12_RETRY_MAX_        = 3;
static const uint8_t  AGR12_FILTER_SIZE_      = 5;

// Soft-I2C pins (AGR12)
static const uint8_t AGR12_SDA_PIN_ = 1;
static const uint8_t AGR12_SCL_PIN_ = 2;

// ===== Filter state =====
static float   agr12_pressureBuf_[AGR12_FILTER_SIZE_];
static uint8_t agr12_bufIndex_   = 0;
static bool    agr12_bufFull_    = false;
static float   agr12_lastPressure_ = 0.0f;

// ===== Soft I2C (open-drain) =====
static inline void agr12_SDA_low_() { pinMode(AGR12_SDA_PIN_, OUTPUT); digitalWrite(AGR12_SDA_PIN_, LOW); }
static inline void agr12_SDA_rel_() { pinMode(AGR12_SDA_PIN_, INPUT_PULLUP); }
static inline void agr12_SCL_low_() { pinMode(AGR12_SCL_PIN_, OUTPUT); digitalWrite(AGR12_SCL_PIN_, LOW); }
static inline void agr12_SCL_rel_() { pinMode(AGR12_SCL_PIN_, INPUT_PULLUP); }

static inline void agr12_i2c_delay_() { delayMicroseconds(6); }

static bool agr12_SCL_rel_wait_(uint16_t timeout_us = 2000) {
  agr12_SCL_rel_();
  while (digitalRead(AGR12_SCL_PIN_) == LOW && timeout_us--) delayMicroseconds(1);
  return (digitalRead(AGR12_SCL_PIN_) == HIGH);
}

static void agr12_i2c_start_() {
  agr12_SDA_rel_(); agr12_SCL_rel_wait_();
  agr12_i2c_delay_();
  agr12_SDA_low_();
  agr12_i2c_delay_();
  agr12_SCL_low_();
  agr12_i2c_delay_();
}

static void agr12_i2c_stop_() {
  agr12_SDA_low_();
  agr12_i2c_delay_();
  agr12_SCL_rel_wait_();
  agr12_i2c_delay_();
  agr12_SDA_rel_();
  agr12_i2c_delay_();
}

static bool agr12_i2c_writeByte_(uint8_t data) {
  for (uint8_t i = 0; i < 8; i++) {
    if (data & 0x80) agr12_SDA_rel_(); else agr12_SDA_low_();
    agr12_i2c_delay_();
    if (!agr12_SCL_rel_wait_()) return false;
    agr12_i2c_delay_();
    agr12_SCL_low_();
    agr12_i2c_delay_();
    data <<= 1;
  }
  agr12_SDA_rel_();
  agr12_i2c_delay_();
  if (!agr12_SCL_rel_wait_()) return false;
  agr12_i2c_delay_();
  bool ack = (digitalRead(AGR12_SDA_PIN_) == LOW);
  agr12_SCL_low_();
  agr12_i2c_delay_();
  return ack;
}

static uint8_t agr12_i2c_readByte_(bool ack) {
  uint8_t data = 0;
  agr12_SDA_rel_();
  for (uint8_t i = 0; i < 8; i++) {
    data <<= 1;
    agr12_i2c_delay_();
    agr12_SCL_rel_wait_();
    agr12_i2c_delay_();
    if (digitalRead(AGR12_SDA_PIN_)) data |= 1;
    agr12_SCL_low_();
    agr12_i2c_delay_();
  }
  if (ack) agr12_SDA_low_(); else agr12_SDA_rel_();
  agr12_i2c_delay_();
  agr12_SCL_rel_wait_();
  agr12_i2c_delay_();
  agr12_SCL_low_();
  agr12_i2c_delay_();
  agr12_SDA_rel_();
  return data;
}

static uint8_t agr12_xorChecksum2_(const uint8_t *buf, uint8_t len) {
  uint8_t cs = 0;
  for (uint8_t i = 0; i < len; i++) cs ^= buf[i];
  return cs;
}

// ===== Trigger + Read data =====
static bool agr12Trigger_() {
  agr12_i2c_start_();
  if (!agr12_i2c_writeByte_((AGR12_ADDR_ << 1) | 0)) { agr12_i2c_stop_(); return false; }
  if (!agr12_i2c_writeByte_(AGR12_CMD_REG_))         { agr12_i2c_stop_(); return false; }
  if (!agr12_i2c_writeByte_(AGR12_CMD_MEASURE_))     { agr12_i2c_stop_(); return false; }
  agr12_i2c_stop_();
  return true;
}

static bool agr12ReadData_(float &out_kPa) {
  uint8_t rx[3];

  agr12_i2c_start_();
  if (!agr12_i2c_writeByte_((AGR12_ADDR_ << 1) | 1)) { agr12_i2c_stop_(); return false; }
  rx[0] = agr12_i2c_readByte_(true);
  rx[1] = agr12_i2c_readByte_(true);
  rx[2] = agr12_i2c_readByte_(false);
  agr12_i2c_stop_();

  if (agr12_xorChecksum2_(rx, 2) != rx[2]) return false;

  int16_t raw = (int16_t)((uint16_t(rx[0]) << 8) | rx[1]);
  out_kPa = raw / 10.0f;
  return true;
}

static float agr12FilterPressure_(float val) {
  agr12_pressureBuf_[agr12_bufIndex_++] = val;
  if (agr12_bufIndex_ >= AGR12_FILTER_SIZE_) {
    agr12_bufIndex_ = 0;
    agr12_bufFull_ = true;
  }
  uint8_t count = agr12_bufFull_ ? AGR12_FILTER_SIZE_ : agr12_bufIndex_;
  float sum = 0;
  for (uint8_t i = 0; i < count; i++) sum += agr12_pressureBuf_[i];
  return sum / count;
}

// ===== LCD show (GIỮ NGUYÊN HIỂN THỊ) =====
static void agr12LcdShowPressure_(float pFilt) {
  lcdPrintLine(0, "AGR12 Soft-I2C mode");
  lcdPrintLine(1, "Ap suat:");

  char line2[21];
  char num[12];
  dtostrf(pFilt, 0, 1, num);
  snprintf(line2, sizeof(line2), "%s kPa", num);
  lcdPrintLine(2, line2);

  lcdPrintLine(3, "");
}

static void agr12LcdShowHoldValue_(const char* reason, float hold_kPa) {
  lcdPrintLine(0, "AGR12 Soft-I2C mode");
  lcdPrintLine(1, reason);
  lcdPrintLine(2, "Giu gia tri cu:");

  char line3[21];
  char num[12];
  dtostrf(hold_kPa, 0, 1, num);
  snprintf(line3, sizeof(line3), "%s kPa", num);
  lcdPrintLine(3, line3);
}

// ===== FSM millis() =====
enum AGR12_ReadState_ : uint8_t { AGR12_IDLE_, AGR12_WAIT_MEASURE_, AGR12_READ_TRY_ };
static AGR12_ReadState_ agr12_state_ = AGR12_IDLE_;

static uint32_t agr12_tNextCycle_   = 0;
static uint32_t agr12_tMeasureStart_ = 0;
static uint8_t  agr12_retryLeft_     = 0;

static bool agr12_savedHeaderEnabled_ = true;

inline void startAGR12Mode() {
  // Giữ nguyên line0 của mode (không để header countdown ghi đè)
  agr12_savedHeaderEnabled_ = headerEnabled;
  headerEnabled = false;

  // init soft-i2c lines
  agr12_SDA_rel_();
  agr12_SCL_rel_();
  delay(30);

  agr12_bufIndex_ = 0;
  agr12_bufFull_ = false;
  agr12_lastPressure_ = 0.0f;
  agr12_state_ = AGR12_IDLE_;
  agr12_tNextCycle_ = millis();

  lcd.clear();
  lcdPrintLine(0, "AGR12 Soft-I2C mode");
  lcdPrintLine(1, "Dang khoi dong...");
  lcdPrintLine(2, "");
  lcdPrintLine(3, "");
}

inline void updateAGR12Mode(uint32_t now) {
  switch (agr12_state_) {
    case AGR12_IDLE_:
      if ((int32_t)(now - agr12_tNextCycle_) >= 0) {
        agr12_retryLeft_ = AGR12_RETRY_MAX_;

        if (agr12Trigger_()) {
          agr12_tMeasureStart_ = now;
          agr12_state_ = AGR12_WAIT_MEASURE_;
        } else {
          agr12LcdShowHoldValue_("Loi trigger", agr12_lastPressure_);
          agr12_tNextCycle_ = now + AGR12_READ_INTERVAL_MS_;
        }
      }
      break;

    case AGR12_WAIT_MEASURE_:
      if ((uint32_t)(now - agr12_tMeasureStart_) >= AGR12_MEASURE_TIME_MS_) {
        agr12_state_ = AGR12_READ_TRY_;
      }
      break;

    case AGR12_READ_TRY_: {
      float p;
      if (agr12ReadData_(p)) {
        agr12_lastPressure_ = p;
        float pFilt = agr12FilterPressure_(p);

        agr12LcdShowPressure_(pFilt);

        agr12_tNextCycle_ = now + AGR12_READ_INTERVAL_MS_;
        agr12_state_ = AGR12_IDLE_;
      } else {
        if (agr12_retryLeft_ > 0) agr12_retryLeft_--;

        if (agr12_retryLeft_ == 0) {
          agr12LcdShowHoldValue_("Loi doc", agr12_lastPressure_);
          agr12_tNextCycle_ = now + AGR12_READ_INTERVAL_MS_;
          agr12_state_ = AGR12_IDLE_;
        } else {
          if (agr12Trigger_()) {
            agr12_tMeasureStart_ = now;
            agr12_state_ = AGR12_WAIT_MEASURE_;
          }
        }
      }
    } break;
  }
}

inline void stopAGR12Mode() {
  // thả bus soft-i2c
  agr12_SDA_rel_();
  agr12_SCL_rel_();

  // restore header countdown
  headerEnabled = agr12_savedHeaderEnabled_;

  // restore I2CScanBus (nếu các mode khác cần)
  I2CScanBus.begin(I2C_SCAN_SDA_PIN, I2C_SCAN_SCL_PIN, 100000UL);
  I2CScanBus.setClock(100000UL);
}

#endif
