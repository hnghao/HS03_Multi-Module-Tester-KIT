#ifndef MPU6050_MODE_H
#define MPU6050_MODE_H

#include <Arduino.h>
#include <Wire.h>

// Dùng lại I2C bus cảm biến (GPIO1/2) và LCD của chương trình chính
extern TwoWire I2CScanBus;
extern void lcdPrintLine(uint8_t row, const char* text);
extern bool headerEnabled;

// PIN của I2CScanBus (đã có trong HS03_UI.ino)
#ifndef I2C_SCAN_SDA_PIN
#define I2C_SCAN_SDA_PIN 1
#endif
#ifndef I2C_SCAN_SCL_PIN
#define I2C_SCAN_SCL_PIN 2
#endif

// ================== I2C SPEED (GIỮ NGUYÊN HIỂN THỊ, không đổi text) ==================
static const uint32_t MPU_I2C_FREQ = 400000UL;

// ================== MPU REG (GIỮ NGUYÊN) ==================
static const uint8_t REG_SMPLRT_DIV   = 0x19;
static const uint8_t REG_CONFIG       = 0x1A;
static const uint8_t REG_GYRO_CONFIG  = 0x1B;
static const uint8_t REG_ACCEL_CONFIG = 0x1C;
static const uint8_t REG_PWR_MGMT_1   = 0x6B;
static const uint8_t REG_WHO_AM_I     = 0x75;
static const uint8_t REG_ACCEL_XOUT_H = 0x3B;

// ================== MPU STATE ==================
static uint8_t mpuAddr = 0;   // 0 = chưa kết nối
static uint8_t whoami  = 0;

// Không chặn loop HS03: dùng timer thay cho delay()
static uint32_t lastRetryMs = 0;
static uint32_t lastReadMs  = 0;
static uint32_t showOkUntilMs = 0;

// Giữ nguyên “tốc độ hiển thị” như sketch bạn gửi
static const uint32_t RETRY_INTERVAL_MS = 1000;
static const uint32_t READ_INTERVAL_MS  = 300;
static const uint32_t OK_SCREEN_MS      = 500;

// Giữ header row không ghi đè LCD trong mode này (để đúng hiển thị bạn gửi)
static bool savedHeaderEnabled = true;

// ================== HELPERS (adapt sang I2CScanBus) ==================
static bool i2cWriteByte(TwoWire &w, uint8_t addr, uint8_t reg, uint8_t data) {
  w.beginTransmission(addr);
  w.write(reg);
  w.write(data);
  return (w.endTransmission(true) == 0);
}

static bool i2cReadBytes(TwoWire &w, uint8_t addr, uint8_t reg, uint8_t *buf, size_t len) {
  w.beginTransmission(addr);
  w.write(reg);
  if (w.endTransmission(false) != 0) return false; // repeated start
  if (w.requestFrom((int)addr, (int)len, (int)true) != (int)len) return false;
  for (size_t i = 0; i < len; i++) buf[i] = (uint8_t)w.read();
  return true;
}

static bool i2cReadByte(TwoWire &w, uint8_t addr, uint8_t reg, uint8_t &val) {
  return i2cReadBytes(w, addr, reg, &val, 1);
}

// Format số có dấu +/-, độ rộng cố định (GIỮ NGUYÊN)
static void fmtSigned(char* out, size_t outsz, float v, uint8_t width, uint8_t prec) {
  char num[16];
  float av = (v < 0) ? -v : v;
  dtostrf(av, 0, prec, num);                 // "0.00"
  char sign = (v < 0) ? '-' : '+';
  char core[20];
  snprintf(core, sizeof(core), "%c%s", sign, num); // "+0.00"
  int len = (int)strlen(core);
  int pad = (width > len) ? (width - len) : 0;

  int idx = 0;
  for (int i = 0; i < pad && idx < (int)outsz - 1; i++) out[idx++] = ' ';
  for (int i = 0; i < len && idx < (int)outsz - 1; i++) out[idx++] = core[i];
  out[idx] = '\0';
}

static bool probeMPU(uint8_t addr, uint8_t &id) {
  return i2cReadByte(I2CScanBus, addr, REG_WHO_AM_I, id);
}

static bool initMPU(uint8_t addr) {
  // Wake
  if (!i2cWriteByte(I2CScanBus, addr, REG_PWR_MGMT_1, 0x00)) return false;

  // DLPF ~ 21Hz
  if (!i2cWriteByte(I2CScanBus, addr, REG_CONFIG, 0x04)) return false;

  // ~100Hz sample (1kHz/(1+9))
  if (!i2cWriteByte(I2CScanBus, addr, REG_SMPLRT_DIV, 9)) return false;

  // Gyro ±500 dps
  if (!i2cWriteByte(I2CScanBus, addr, REG_GYRO_CONFIG, 0x08)) return false;

  // Accel ±8g
  if (!i2cWriteByte(I2CScanBus, addr, REG_ACCEL_CONFIG, 0x10)) return false;

  return true;
}

static bool findAndStartMPU() {
  mpuAddr = 0;
  whoami  = 0;

  uint8_t id;
  if (probeMPU(0x68, id)) { mpuAddr = 0x68; whoami = id; }
  else if (probeMPU(0x69, id)) { mpuAddr = 0x69; whoami = id; }
  else return false;

  if (!initMPU(mpuAddr)) {
    mpuAddr = 0;
    return false;
  }
  return true;
}

// ================== MODE API ==================
inline void startMPU6050Mode() {
  // Tắt header để giữ nguyên toàn bộ hiển thị LCD trong sketch bạn gửi
  savedHeaderEnabled = headerEnabled;
  headerEnabled = false;

  // Đảm bảo I2C bus cảm biến chạy trên GPIO1/2 và 400kHz
  I2CScanBus.begin(I2C_SCAN_SDA_PIN, I2C_SCAN_SCL_PIN, MPU_I2C_FREQ);
  I2CScanBus.setClock(MPU_I2C_FREQ);

  // Màn hình khởi tạo đúng như sketch bạn gửi
  lcdPrintLine(0, "ESP32-S3  MPU6DOF");
  lcdPrintLine(1, "LCD OK  I2C6/7   ");
  lcdPrintLine(2, "MPU I2C1/2 Init  ");
  lcdPrintLine(3, "Please wait...   ");

  mpuAddr = 0;
  whoami  = 0;
  lastRetryMs = 0;
  lastReadMs  = 0;
  showOkUntilMs = 0;
}

inline void updateMPU6050Mode(uint32_t now) {
  // Nếu đang hiển thị màn “MPU OK ...” 500ms thì giữ nguyên
  if (showOkUntilMs && (int32_t)(now - showOkUntilMs) < 0) return;
  if (showOkUntilMs && (int32_t)(now - showOkUntilMs) >= 0) showOkUntilMs = 0;

  // Chưa kết nối -> retry mỗi 1 giây (GIỮ NGUYÊN HIỂN THỊ)
  if (!mpuAddr) {
    if (now - lastRetryMs >= RETRY_INTERVAL_MS) {
      lastRetryMs = now;

      if (findAndStartMPU()) {
        char l3[21];
        snprintf(l3, sizeof(l3), "MPU OK 0x%02X ADR%02X", mpuAddr, whoami);

        lcdPrintLine(0, "ESP32-S3  MPU6DOF");
        lcdPrintLine(1, "Reading data...  ");
        lcdPrintLine(2, "                  ");
        lcdPrintLine(3, l3);

        showOkUntilMs = now + OK_SCREEN_MS;
      } else {
        lcdPrintLine(0, "MPU NOT FOUND!    ");
        lcdPrintLine(1, "Check SDA=1 SCL=2 ");
        lcdPrintLine(2, "Addr 0x68/0x69    ");
        lcdPrintLine(3, "Retrying...       ");
      }
    }
    return;
  }

  // Đọc dữ liệu mỗi 300ms (GIỮ NGUYÊN)
  if (now - lastReadMs < READ_INTERVAL_MS) return;
  lastReadMs = now;

  uint8_t raw[14];
  if (!i2cReadBytes(I2CScanBus, mpuAddr, REG_ACCEL_XOUT_H, raw, 14)) {
    mpuAddr = 0;
    lcdPrintLine(0, "MPU DISCONNECTED  ");
    lcdPrintLine(1, "Reconnecting...   ");
    lcdPrintLine(2, "                  ");
    lcdPrintLine(3, "                  ");
    return;
  }

  int16_t ax = (int16_t)((raw[0] << 8) | raw[1]);
  int16_t ay = (int16_t)((raw[2] << 8) | raw[3]);
  int16_t az = (int16_t)((raw[4] << 8) | raw[5]);
  // int16_t tempRaw = (int16_t)((raw[6] << 8) | raw[7]);
  int16_t gx = (int16_t)((raw[8] << 8) | raw[9]);
  int16_t gy = (int16_t)((raw[10] << 8) | raw[11]);
  int16_t gz = (int16_t)((raw[12] << 8) | raw[13]);

  // Theo cấu hình: Accel ±8g => 4096 LSB/g ; Gyro ±500 dps => 65.5 LSB/(dps)
  const float g_per_lsb   = 1.0f / 4096.0f;
  const float dps_per_lsb = 1.0f / 65.5f;

  float ax_g = ax * g_per_lsb;
  float ay_g = ay * g_per_lsb;
  float az_g = az * g_per_lsb;

  float gx_dps = gx * dps_per_lsb;
  float gy_dps = gy * dps_per_lsb;
  float gz_dps = gz * dps_per_lsb;

  // float tempC = (tempRaw / 340.0f) + 36.53f;

  // ==== Build 4 lines exactly fitting 20 cols (GIỮ NGUYÊN FORMAT) ====
  char sax[8], say[8], saz[8];
  char sgx[7], sgy[7], sgz[7];
  char st[8];

  fmtSigned(sax, sizeof(sax), ax_g, 6, 2);
  fmtSigned(say, sizeof(say), ay_g, 6, 2);
  fmtSigned(saz, sizeof(saz), az_g, 6, 2);

  fmtSigned(sgx, sizeof(sgx), gx_dps, 5, 1);
  fmtSigned(sgy, sizeof(sgy), gy_dps, 5, 1);
  fmtSigned(sgz, sizeof(sgz), gz_dps, 5, 1);

  // dtostrf(tempC, 5, 2, st); // "25.00"

  char line0[21], line1[21], line2[21], line3[21];

  snprintf(line0, sizeof(line0), "AX:%s AY:%s", sax, say);
  // snprintf(line1, sizeof(line1), "AZ:%s T:%sC",  saz, st);
  // Không còn T -> dòng 1 chỉ hiển thị AZ
  snprintf(line1, sizeof(line1), "AZ:%s           ", saz);
  snprintf(line2, sizeof(line2), "GX:%s GY:%s",   sgx, sgy);
  snprintf(line3, sizeof(line3), "GZ:%s ADR:0x%02X", sgz, mpuAddr);

  lcdPrintLine(0, line0);
  lcdPrintLine(1, line1);
  lcdPrintLine(2, line2);
  lcdPrintLine(3, line3);
}

inline void stopMPU6050Mode() {
  // Trả I2C bus về 100kHz (mặc định)
  I2CScanBus.setClock(100000UL);

  // Bật lại header cho các mode khác
  headerEnabled = savedHeaderEnabled;
}

#endif
