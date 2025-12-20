#ifndef BUI_ASAIR_MODE_H
#define BUI_ASAIR_MODE_H

#include <Arduino.h>
#include <Wire.h>
#include <limits.h>

// ===================== DÙNG CHUNG TỪ PROJECT =====================
extern TwoWire I2CScanBus;                 // bus I2C sensor: SDA=GPIO1, SCL=GPIO2
class LiquidCrystal_I2C;
extern LiquidCrystal_I2C lcd;

extern bool headerEnabled;
extern void lcdPrintLine(uint8_t row, const char *text);

// ===================== APM10 (ASAIR) CONFIG =====================
static const uint8_t  APM10_ADDR            = 0x08;
static const uint8_t  APM10_FRAME_LEN       = 30;      // 10 groups * (Hi,Lo,CRC)
static const uint32_t APM10_READ_PERIOD_MS  = 1000;
static const uint8_t  APM10_MAX_BAD_STREAK  = 5;

// PM groups in frame (each group = 3 bytes)
static const int APM10_PM1_GROUP  = 0;   // bytes 0..2
static const int APM10_PM25_GROUP = 1;   // bytes 3..5
// PM10 group varies by module -> auto pick + lock

// Sensor bus speed (10kHz)
static const uint32_t APM10_I2C_FREQ = 10000UL;

// ===================== VN_AQI =====================
// AQI breakpoints Ii = 0,50,100,150,200,300,400,500
static const int VN_AQI_I[8]   = {0, 50, 100, 150, 200, 300, 400, 500};
// PM10 (µg/m3)
static const int VN_BP_PM10[8] = {0, 50, 150, 250, 350, 420, 500, 600};
// PM2.5 (µg/m3)
static const int VN_BP_PM25[8] = {0, 25, 50, 80, 150, 250, 350, 500};

// ===================== INTERNAL STATE =====================
static bool          apm10_savedHeaderEnabled_ = true;
static unsigned long apm10_lastReadMs_         = 0;
static uint8_t       apm10_badStreak_          = 0;

// Track variability to avoid “reserved” slots
static uint16_t apm10_lastWord_[10]    = {0};
static uint16_t apm10_changeCount_[10] = {0};

// PM10 lock
static int      apm10_lockedPM10Group_ = -1;
static uint16_t apm10_lockConfidence_  = 0;

// ===================== HELPERS =====================
static inline void apm10ShowScreen(const char *l0, const char *l1, const char *l2, const char *l3) {
  lcd.clear();
  lcdPrintLine(0, l0);
  lcdPrintLine(1, l1);
  lcdPrintLine(2, l2);
  lcdPrintLine(3, l3);
}

// CRC8: init 0xFF, poly 0x31
static inline uint8_t apm10_crc8(const uint8_t *data, uint8_t len) {
  uint8_t crc = 0xFF;
  for (uint8_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t b = 0; b < 8; b++) {
      crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x31) : (uint8_t)(crc << 1);
    }
  }
  return crc;
}

static inline bool apm10_isInvalidValue(uint16_t v) {
  return (v == 0xFFFFu) || (v == 0x0000u);
}

static inline bool apm10_ping(uint8_t addr7) {
  I2CScanBus.beginTransmission(addr7);
  return (I2CScanBus.endTransmission(true) == 0);
}

// Start measurement: payload 00 10 05 00 F6
static inline bool apm10_start() {
  const uint8_t cmd[] = {0x00, 0x10, 0x05, 0x00, 0xF6};
  I2CScanBus.beginTransmission(APM10_ADDR);
  I2CScanBus.write(cmd, sizeof(cmd));
  return (I2CScanBus.endTransmission(true) == 0);
}

// Stop measurement: payload 01 04
static inline bool apm10_stop() {
  const uint8_t cmd[] = {0x01, 0x04};
  I2CScanBus.beginTransmission(APM10_ADDR);
  I2CScanBus.write(cmd, sizeof(cmd));
  return (I2CScanBus.endTransmission(true) == 0);
}

// Read frame: write 03 00 (STOP) then read 30 bytes
static inline bool apm10_readFrame(uint8_t buf[APM10_FRAME_LEN]) {
  I2CScanBus.beginTransmission(APM10_ADDR);
  I2CScanBus.write((uint8_t)0x03);
  I2CScanBus.write((uint8_t)0x00);
  if (I2CScanBus.endTransmission(true) != 0) return false;

  delayMicroseconds(300);

  int got = I2CScanBus.requestFrom((int)APM10_ADDR, (int)APM10_FRAME_LEN, (int)true);
  if (got != (int)APM10_FRAME_LEN) {
    while (I2CScanBus.available()) (void)I2CScanBus.read();
    return false;
  }
  for (int i = 0; i < (int)APM10_FRAME_LEN; i++) {
    if (!I2CScanBus.available()) return false;
    buf[i] = (uint8_t)I2CScanBus.read();
  }
  return true;
}

// Triplet = [Hi][Lo][CRC]
static inline bool apm10_readTripletU16(const uint8_t *buf, int off, uint16_t &out) {
  uint8_t pair[2] = { buf[off], buf[off + 1] };
  uint8_t crc     = buf[off + 2];
  if (apm10_crc8(pair, 2) != crc) return false;
  out = (uint16_t)((pair[0] << 8) | pair[1]);
  return true;
}

static inline void apm10_resetTracking() {
  for (int i = 0; i < 10; i++) {
    apm10_lastWord_[i]    = 0;
    apm10_changeCount_[i] = 0;
  }
  apm10_lockedPM10Group_ = -1;
  apm10_lockConfidence_  = 0;
}

static inline void apm10_updateVariability(const uint16_t w[10], const bool ok[10]) {
  for (int i = 0; i < 10; i++) {
    if (!ok[i] || apm10_isInvalidValue(w[i])) continue;
    if (w[i] != apm10_lastWord_[i]) {
      if (apm10_changeCount_[i] < 65000) apm10_changeCount_[i]++;
      apm10_lastWord_[i] = w[i];
    }
  }
}

static inline int apm10_choosePM10Group(uint16_t pm25, const uint16_t w[10], const bool ok[10], int lastChosen) {
  int best = -1;
  long bestScore = LONG_MIN;

  for (int i = 0; i < 10; i++) {
    if (i == APM10_PM1_GROUP || i == APM10_PM25_GROUP) continue;
    if (!ok[i]) continue;
    if (apm10_isInvalidValue(w[i])) continue;

    long score = 0;
    if (w[i] >= pm25) score += 1000; else score -= 5000;
    score += (long)apm10_changeCount_[i] * 20;
    if (w[i] == pm25) score -= 30;
    if (apm10_changeCount_[i] == 0) score -= 200;
    if (i == lastChosen) score += 120;

    if (score > bestScore) {
      bestScore = score;
      best = i;
    }
  }
  return best;
}

static inline void apm10_restartMeasurement() {
  (void)apm10_stop();
  delay(50);
  (void)apm10_start();
  delay(2000);
  apm10_resetTracking();
}

// Read PM1.0, PM2.5, PM10 (PM10 auto-pick + lock)
static inline bool apm10_readPMs(uint16_t &pm1, uint16_t &pm25, uint16_t &pm10, uint8_t &pm10GroupOut) {
  uint8_t buf[APM10_FRAME_LEN];
  if (!apm10_readFrame(buf)) return false;

  uint16_t w[10] = {0};
  bool ok[10] = {0};
  for (int i = 0; i < 10; i++) ok[i] = apm10_readTripletU16(buf, i * 3, w[i]);

  if (!ok[APM10_PM1_GROUP]  || apm10_isInvalidValue(w[APM10_PM1_GROUP]))  return false;
  if (!ok[APM10_PM25_GROUP] || apm10_isInvalidValue(w[APM10_PM25_GROUP])) return false;

  pm1  = w[APM10_PM1_GROUP];
  pm25 = w[APM10_PM25_GROUP];

  apm10_updateVariability(w, ok);

  auto slotValid = [&](int g)->bool {
    if (g < 0 || g >= 10) return false;
    if (!ok[g]) return false;
    if (apm10_isInvalidValue(w[g])) return false;
    if (w[g] < pm25) return false;
    return true;
  };

  if (apm10_lockedPM10Group_ < 0 || !slotValid(apm10_lockedPM10Group_)) {
    apm10_lockedPM10Group_ = apm10_choosePM10Group(pm25, w, ok, apm10_lockedPM10Group_);
    apm10_lockConfidence_ = 0;
    if (apm10_lockedPM10Group_ < 0) return false;
  } else {
    if (apm10_lockConfidence_ < 65000) apm10_lockConfidence_++;
  }

  // early-stage improvement
  if (apm10_lockConfidence_ < 10) {
    int cand = apm10_choosePM10Group(pm25, w, ok, apm10_lockedPM10Group_);
    if (cand >= 0 && cand != apm10_lockedPM10Group_) {
      if (apm10_changeCount_[cand] > apm10_changeCount_[apm10_lockedPM10Group_] + 3) {
        apm10_lockedPM10Group_ = cand;
        apm10_lockConfidence_ = 0;
      }
    }
  }

  pm10 = w[apm10_lockedPM10Group_];
  pm10GroupOut = (uint8_t)apm10_lockedPM10Group_;
  return true;
}

static inline int apm10_aqiFromConc(int c, const int *bp) {
  if (c <= bp[0]) return 0;
  if (c >= bp[7]) return 500;

  for (int i = 0; i < 7; i++) {
    int loBP = bp[i];
    int hiBP = bp[i + 1];
    if (c >= loBP && c <= hiBP) {
      int loI = VN_AQI_I[i];
      int hiI = VN_AQI_I[i + 1];
      float aqi = (float)(hiI - loI) * (float)(c - loBP) / (float)(hiBP - loBP) + (float)loI;
      int out = (int)(aqi + 0.5f);
      if (out < 0) out = 0;
      if (out > 500) out = 500;
      return out;
    }
  }
  return 500;
}

// Label <= 5 ký tự để vừa LCD 20 cột
static inline const char* apm10_vnAqiLabel5(int aqi) {
  if (aqi <= 50)  return "TOT";
  if (aqi <= 100) return "TBINH";
  if (aqi <= 150) return "KEM";
  if (aqi <= 200) return "XAU";
  if (aqi <= 300) return "RXAU";
  return "NGHAI";
}

// Advice <= 6 ký tự để vừa LCD 20 cột
static inline const char* apm10_vnAqiAdvice6(int aqi) {
  if (aqi <= 50)  return "OK";
  if (aqi <= 100) return "GIAMCU";
  if (aqi <= 150) return "KTRANG";
  if (aqi <= 200) return "HANCHE";
  if (aqi <= 300) return "TRANH";
  return "OTRONG";
}

// ===================== API cho HS03_UI =====================
static inline void startBuiASAIRMode() {
  // Tắt header để tránh updateCountdown() ghi đè dòng 0
  apm10_savedHeaderEnabled_ = headerEnabled;
  headerEnabled = false;

  // Đảm bảo bus I2C cảm biến chạy trên GPIO1/2 và 10kHz
  I2CScanBus.begin(I2C_SCAN_SDA_PIN, I2C_SCAN_SCL_PIN, APM10_I2C_FREQ);
  I2CScanBus.setClock(APM10_I2C_FREQ);
  I2CScanBus.setTimeOut(150);

  apm10_resetTracking();
  apm10_lastReadMs_ = 0;
  apm10_badStreak_  = 0;

  apm10ShowScreen(
    "ESP32-S3 + APM10",
    "BUI ASAIR (I2C1/2)",
    "Init sensor...",
    "Please wait..."
  );

  if (!apm10_ping(APM10_ADDR)) {
    apm10ShowScreen(
      "APM10 NOT FOUND!",
      "Check SDA=GPIO1",
      "SCL=GPIO2  Addr08",
      "Click to Back"
    );
    return;
  }

  if (!apm10_start()) {
    apm10ShowScreen(
      "APM10 START FAIL!",
      "Power-cycle sensor",
      "Check wiring/I2C",
      "Click to Back"
    );
    return;
  }

  delay(2500);

  apm10ShowScreen(
    "PM1/PM2.5/PM10",
    "Update: 1s  I2C10k",
    "VN_AQI on line 4",
    "Click: Back I2C"
  );
}

static inline void updateBuiASAIRMode(unsigned long now) {
  if (apm10_lastReadMs_ != 0 && (now - apm10_lastReadMs_) < APM10_READ_PERIOD_MS) return;
  apm10_lastReadMs_ = now;

  // Sensor disconnect
  if (!apm10_ping(APM10_ADDR)) {
    lcdPrintLine(0, "PM1.0: ---- ug/m3");
    lcdPrintLine(1, "PM2.5: ---- ug/m3");
    lcdPrintLine(2, "PM10 : ---- ug/m3");
    lcdPrintLine(3, "Sensor DISCONNECTED");
    delay(200);
    return;
  }

  uint16_t pm1 = 0, pm25 = 0, pm10 = 0;
  uint8_t g10 = 0;

  if (apm10_readPMs(pm1, pm25, pm10, g10)) {
    apm10_badStreak_ = 0;

    char l0[21], l1[21], l2[21], l3[21];
    snprintf(l0, sizeof(l0), "PM1.0: %5u ug/m3", (unsigned)pm1);
    snprintf(l1, sizeof(l1), "PM2.5: %5u ug/m3", (unsigned)pm25);
    snprintf(l2, sizeof(l2), "PM10 : %5u ug/m3", (unsigned)pm10);

    int aqi25 = apm10_aqiFromConc((int)pm25, VN_BP_PM25);
    int aqi10 = apm10_aqiFromConc((int)pm10, VN_BP_PM10);
    int aqi   = (aqi25 > aqi10) ? aqi25 : aqi10;

    const char *label  = apm10_vnAqiLabel5(aqi);
    const char *advice = apm10_vnAqiAdvice6(aqi);

    // 20 ký tự vừa khít: "AQI~123 TBINH GIAMCU"
    snprintf(l3, sizeof(l3), "AQI~%3d %-5s %-6s", aqi, label, advice);

    lcdPrintLine(0, l0);
    lcdPrintLine(1, l1);
    lcdPrintLine(2, l2);
    lcdPrintLine(3, l3);
  } else {
    apm10_badStreak_++;

    lcdPrintLine(0, "PM1.0: ---- ug/m3");
    lcdPrintLine(1, "PM2.5: ---- ug/m3");
    lcdPrintLine(2, "PM10 : ---- ug/m3");

    char l3[21];
    snprintf(l3, sizeof(l3), "INVALID... (%u/%u)",
             (unsigned)apm10_badStreak_, (unsigned)APM10_MAX_BAD_STREAK);
    lcdPrintLine(3, l3);

    if (apm10_badStreak_ >= APM10_MAX_BAD_STREAK) {
      lcdPrintLine(3, "Restart measurement..");
      apm10_restartMeasurement();
      apm10_badStreak_ = 0;
    }
  }
}

static inline void stopBuiASAIRMode() {
  (void)apm10_stop();
  delay(20);

  // Trả I2C về 100kHz (mặc định)
  I2CScanBus.setClock(100000UL);

  // Bật lại header cho các mode khác
  headerEnabled = apm10_savedHeaderEnabled_;
}

#endif // BUI_ASAIR_MODE_H
