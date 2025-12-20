#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// ===================== PIN MAP =====================
// LCD2004 I2C bus (Wire)
static const int LCD_SDA = 6;
static const int LCD_SCL = 7;

// Sensor I2C bus (Wire1): APM10 (+ ADS1115 chung bus nếu có)
static const int SENS_SDA = 1;
static const int SENS_SCL = 2;

// ===================== I2C CONFIG =====================
static const uint32_t LCD_I2C_FREQ  = 100000; // LCD
static const uint32_t SENS_I2C_FREQ = 10000;  // Sensor 10kHz

// ===================== APM10 CONFIG =====================
static const uint8_t APM10_ADDR = 0x08;
static const uint8_t FRAME_LEN  = 30;      // 10 groups * (Hi,Lo,CRC)
static const uint32_t READ_PERIOD_MS = 1000;
static const uint8_t MAX_BAD_STREAK = 5;

// PM groups in frame (each group = 3 bytes)
static const int PM1_GROUP  = 0;   // bytes 0..2
static const int PM25_GROUP = 1;   // bytes 3..5
// PM10 group varies by module; we keep the “auto-pick + lock” method

// ===================== LCD =====================
static LiquidCrystal_I2C *lcd = nullptr;

// Detect common LCD I2C addresses without Serial
static uint8_t detectLcdAddress() {
  const uint8_t candidates[] = {0x27, 0x3F, 0x26, 0x20, 0x21, 0x22, 0x23};
  for (uint8_t a : candidates) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission(true) == 0) return a;
  }
  return 0x27; // fallback
}

static void lcdPrintLine(uint8_t row, const char *text) {
  char buf[21];
  memset(buf, ' ', 20);
  buf[20] = '\0';
  size_t n = strlen(text);
  if (n > 20) n = 20;
  memcpy(buf, text, n);

  lcd->setCursor(0, row);
  lcd->print(buf);
}

static void showScreen(const char *l0, const char *l1, const char *l2, const char *l3) {
  lcd->clear();
  lcdPrintLine(0, l0);
  lcdPrintLine(1, l1);
  lcdPrintLine(2, l2);
  lcdPrintLine(3, l3);
}

// ===================== APM10 CRC =====================
// CRC8: init 0xFF, poly 0x31
static uint8_t crc8_apm10(const uint8_t* data, uint8_t len) {
  uint8_t crc = 0xFF;
  for (uint8_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t b = 0; b < 8; b++) {
      crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x31) : (uint8_t)(crc << 1);
    }
  }
  return crc;
}

static bool isInvalidValue(uint16_t v) {
  return (v == 0xFFFF) || (v == 0x0000);
}

static bool sensPing(uint8_t addr7) {
  Wire1.beginTransmission(addr7);
  return (Wire1.endTransmission(true) == 0);
}

// Start measurement: payload 00 10 05 00 F6
static bool apm10Start() {
  const uint8_t cmd[] = {0x00, 0x10, 0x05, 0x00, 0xF6};
  Wire1.beginTransmission(APM10_ADDR);
  Wire1.write(cmd, sizeof(cmd));
  return (Wire1.endTransmission(true) == 0);
}

// Stop measurement: payload 01 04
static bool apm10Stop() {
  const uint8_t cmd[] = {0x01, 0x04};
  Wire1.beginTransmission(APM10_ADDR);
  Wire1.write(cmd, sizeof(cmd));
  return (Wire1.endTransmission(true) == 0);
}

// Read frame: write 03 00 (STOP) then read 30 bytes
static bool apm10ReadFrame(uint8_t buf[FRAME_LEN]) {
  Wire1.beginTransmission(APM10_ADDR);
  Wire1.write((uint8_t)0x03);
  Wire1.write((uint8_t)0x00);
  if (Wire1.endTransmission(true) != 0) return false;

  delayMicroseconds(300);

  int got = Wire1.requestFrom((int)APM10_ADDR, (int)FRAME_LEN, (int)true);
  if (got != FRAME_LEN) {
    while (Wire1.available()) (void)Wire1.read();
    return false;
  }
  for (int i = 0; i < FRAME_LEN; i++) {
    if (!Wire1.available()) return false;
    buf[i] = (uint8_t)Wire1.read();
  }
  return true;
}

// Triplet = [Hi][Lo][CRC]
static bool readTripletU16(const uint8_t* buf, int off, uint16_t& out) {
  uint8_t pair[2] = { buf[off], buf[off + 1] };
  uint8_t crc     = buf[off + 2];
  if (crc8_apm10(pair, 2) != crc) return false;
  out = (uint16_t)((pair[0] << 8) | pair[1]);
  return true;
}

// Track variability to avoid “reserved” slots
static uint16_t lastWord[10] = {0};
static uint16_t changeCount[10] = {0};

static void updateVariability(const uint16_t w[10], const bool ok[10]) {
  for (int i = 0; i < 10; i++) {
    if (!ok[i] || isInvalidValue(w[i])) continue;
    if (w[i] != lastWord[i]) {
      if (changeCount[i] < 65000) changeCount[i]++;
      lastWord[i] = w[i];
    }
  }
}

static int choosePM10Group(uint16_t pm25, const uint16_t w[10], const bool ok[10], int lastChosen) {
  int best = -1;
  long bestScore = LONG_MIN;

  for (int i = 0; i < 10; i++) {
    if (i == PM1_GROUP || i == PM25_GROUP) continue;
    if (!ok[i]) continue;
    if (isInvalidValue(w[i])) continue;

    long score = 0;
    if (w[i] >= pm25) score += 1000; else score -= 5000;
    score += (long)changeCount[i] * 20;
    if (w[i] == pm25) score -= 30;
    if (changeCount[i] == 0) score -= 200;
    if (i == lastChosen) score += 120;

    if (score > bestScore) {
      bestScore = score;
      best = i;
    }
  }
  return best;
}

static void restartMeasurement() {
  (void)apm10Stop();
  delay(50);
  (void)apm10Start();
  delay(2000);
}

// Read PM1.0, PM2.5, PM10 (PM10 auto-pick + lock)
static bool readPMs(uint16_t &pm1, uint16_t &pm25, uint16_t &pm10, uint8_t &pm10GroupOut) {
  uint8_t buf[FRAME_LEN];
  if (!apm10ReadFrame(buf)) return false;

  uint16_t w[10] = {0};
  bool ok[10] = {0};
  for (int i = 0; i < 10; i++) ok[i] = readTripletU16(buf, i * 3, w[i]);

  // PM1.0 + PM2.5 must be valid to display stable
  if (!ok[PM1_GROUP]  || isInvalidValue(w[PM1_GROUP]))  return false;
  if (!ok[PM25_GROUP] || isInvalidValue(w[PM25_GROUP])) return false;

  pm1  = w[PM1_GROUP];
  pm25 = w[PM25_GROUP];

  updateVariability(w, ok);

  static int lockedPM10Group = -1;
  static uint16_t lockConfidence = 0;

  auto slotValid = [&](int g)->bool {
    if (g < 0 || g >= 10) return false;
    if (!ok[g]) return false;
    if (isInvalidValue(w[g])) return false;
    if (w[g] < pm25) return false;
    return true;
  };

  if (lockedPM10Group < 0 || !slotValid(lockedPM10Group)) {
    lockedPM10Group = choosePM10Group(pm25, w, ok, lockedPM10Group);
    lockConfidence = 0;
    if (lockedPM10Group < 0) return false;
  } else {
    if (lockConfidence < 65000) lockConfidence++;
  }

  // early-stage improvement
  if (lockConfidence < 10) {
    int cand = choosePM10Group(pm25, w, ok, lockedPM10Group);
    if (cand >= 0 && cand != lockedPM10Group) {
      if (changeCount[cand] > changeCount[lockedPM10Group] + 3) {
        lockedPM10Group = cand;
        lockConfidence = 0;
      }
    }
  }

  pm10 = w[lockedPM10Group];
  pm10GroupOut = (uint8_t)lockedPM10Group;
  return true;
}

// ===================== VN_AQI (based on Decision 1459/QD-TCMT) =====================
// Breakpoints BPi for PM10 & PM2.5 (µg/m3) at Ii = 0,50,100,150,200,300,400,500 :contentReference[oaicite:1]{index=1}
static const int AQI_I[8]      = {0, 50, 100, 150, 200, 300, 400, 500};
static const int BP_PM10[8]    = {0, 50, 150, 250, 350, 420, 500, 600}; // last is ">=600" :contentReference[oaicite:2]{index=2}
static const int BP_PM25[8]    = {0, 25, 50, 80, 150, 250, 350, 500};   // last is ">=500" :contentReference[oaicite:3]{index=3}

static int aqiFromConc(int c, const int *bp) {
  if (c <= bp[0]) return 0;
  if (c >= bp[7]) return 500;

  for (int i = 0; i < 7; i++) {
    int loBP = bp[i];
    int hiBP = bp[i + 1];
    if (c >= loBP && c <= hiBP) {
      int loI = AQI_I[i];
      int hiI = AQI_I[i + 1];
      // linear interpolation
      float aqi = (float)(hiI - loI) * (float)(c - loBP) / (float)(hiBP - loBP) + (float)loI;
      int out = (int)(aqi + 0.5f);
      if (out < 0) out = 0;
      if (out > 500) out = 500;
      return out;
    }
  }
  return 500;
}

// AQI categories in VN_AQI: 0-50 Tot, 51-100 Trung binh, 101-150 Kem, ... :contentReference[oaicite:4]{index=4}
static const char* vnAqiLabel(int aqi) {
  if (aqi <= 50)  return "TOT";
  if (aqi <= 100) return "TBINH";
  if (aqi <= 150) return "KEM";
  if (aqi <= 200) return "XAU";
  if (aqi <= 300) return "RATXAU";
  return "NGUYHAI";
}

static const char* vnAqiAdviceShort(int aqi) {
  if (aqi <= 50)  return "OK";
  if (aqi <= 100) return "Giam mo cua";
  if (aqi <= 150) return "Deo khau trang";
  if (aqi <= 200) return "Han che ra ngoai";
  if (aqi <= 300) return "Tranh hoat dong";
  return "O trong nha + loc";
}

// ===================== MAIN =====================
void setup() {
  // LCD bus init
  Wire.begin(LCD_SDA, LCD_SCL);
  Wire.setClock(LCD_I2C_FREQ);

  uint8_t lcdAddr = detectLcdAddress();
  lcd = new LiquidCrystal_I2C(lcdAddr, 20, 4);
  lcd->init();
  lcd->backlight();

  showScreen(
    "ESP32-S3 + APM10",
    "LCD2004 I2C OK",
    "Init sensor I2C..",
    "Please wait..."
  );

  // Sensor bus init (Wire1)
  Wire1.begin(SENS_SDA, SENS_SCL);
  Wire1.setClock(SENS_I2C_FREQ);
  Wire1.setTimeOut(150);

  if (!sensPing(APM10_ADDR)) {
    showScreen(
      "APM10 NOT FOUND!",
      "Check: SDA1=GPIO1",
      "SCL1=GPIO2 SET=G",
      "Addr 0x08"
    );
    return;
  }

  if (!apm10Start()) {
    showScreen(
      "APM10 START FAIL!",
      "Power-cycle sensor",
      "Check I2C level",
      "and wiring"
    );
    return;
  }

  delay(3000);

  showScreen(
    "PM1/PM2.5/PM10",
    "I2C Sensor: 10kHz",
    "Updating each 1s",
    "VN_AQI on line 4"
  );
}

void loop() {
  static uint32_t lastMs = 0;
  static uint8_t badStreak = 0;

  if (millis() - lastMs < READ_PERIOD_MS) return;
  lastMs = millis();

  if (!sensPing(APM10_ADDR)) {
    lcdPrintLine(0, "PM1.0: ---- ug/m3");
    lcdPrintLine(1, "PM2.5: ---- ug/m3");
    lcdPrintLine(2, "PM10 : ---- ug/m3");
    lcdPrintLine(3, "Sensor DISCONNECTED");
    delay(200);
    return;
  }

  uint16_t pm1 = 0, pm25 = 0, pm10 = 0;
  uint8_t g10 = 0;

  if (readPMs(pm1, pm25, pm10, g10)) {
    badStreak = 0;

    char l0[21], l1[21], l2[21], l3[21];
    snprintf(l0, sizeof(l0), "PM1.0: %5u ug/m3",  (unsigned)pm1);
    snprintf(l1, sizeof(l1), "PM2.5: %5u ug/m3",  (unsigned)pm25);
    snprintf(l2, sizeof(l2), "PM10 : %5u ug/m3",  (unsigned)pm10);

    // VN_AQI~ based on PM2.5 & PM10 breakpoints (instant reading as approximation)
    int aqi25 = aqiFromConc((int)pm25, BP_PM25);
    int aqi10 = aqiFromConc((int)pm10, BP_PM10);
    int aqi   = (aqi25 > aqi10) ? aqi25 : aqi10;

    const char* label  = vnAqiLabel(aqi);
    const char* advice = vnAqiAdviceShort(aqi);

    // Line 4: "AQI~123 KEM | Deo..."
    // (20 chars max) -> keep it compact
    snprintf(l3, sizeof(l3), "AQI~%3d %-6s %-6s", aqi, label, advice);

    lcdPrintLine(0, l0);
    lcdPrintLine(1, l1);
    lcdPrintLine(2, l2);
    lcdPrintLine(3, l3);

  } else {
    badStreak++;

    lcdPrintLine(0, "PM1.0: ---- ug/m3");
    lcdPrintLine(1, "PM2.5: ---- ug/m3");
    lcdPrintLine(2, "PM10 : ---- ug/m3");

    char l3[21];
    snprintf(l3, sizeof(l3), "INVALID... (%u/%u)", (unsigned)badStreak, (unsigned)MAX_BAD_STREAK);
    lcdPrintLine(3, l3);

    if (badStreak >= MAX_BAD_STREAK) {
      lcdPrintLine(3, "Restart measurement..");
      restartMeasurement();
      badStreak = 0;
    }
  }
}
