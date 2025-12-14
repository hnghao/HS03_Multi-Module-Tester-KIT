#include <Wire.h>
#include <LiquidCrystal_PCF8574.h>

// ================== PIN MAP ==================
#define MPU_SDA   1
#define MPU_SCL   2

#define LCD_SDA   6
#define LCD_SCL   7

// ================== I2C SPEED ==================
#define MPU_I2C_FREQ  400000UL
#define LCD_I2C_FREQ  100000UL   // LCD thường ổn định hơn ở 100k

// ================== MPU REG ==================
static const uint8_t REG_SMPLRT_DIV   = 0x19;
static const uint8_t REG_CONFIG       = 0x1A;
static const uint8_t REG_GYRO_CONFIG  = 0x1B;
static const uint8_t REG_ACCEL_CONFIG = 0x1C;
static const uint8_t REG_PWR_MGMT_1   = 0x6B;
static const uint8_t REG_WHO_AM_I     = 0x75;
static const uint8_t REG_ACCEL_XOUT_H = 0x3B;

// ================== LCD OBJECTS (0x27 / 0x3F) ==================
LiquidCrystal_PCF8574 lcd27(0x27);
LiquidCrystal_PCF8574 lcd3F(0x3F);
LiquidCrystal_PCF8574* lcd = &lcd27;

// ================== MPU STATE ==================
uint8_t mpuAddr = 0;   // 0 = chưa kết nối
uint8_t whoami  = 0;

// ================== HELPERS ==================
bool i2cWriteByte(TwoWire &w, uint8_t addr, uint8_t reg, uint8_t data) {
  w.beginTransmission(addr);
  w.write(reg);
  w.write(data);
  return (w.endTransmission(true) == 0);
}

bool i2cReadBytes(TwoWire &w, uint8_t addr, uint8_t reg, uint8_t *buf, size_t len) {
  w.beginTransmission(addr);
  w.write(reg);
  if (w.endTransmission(false) != 0) return false; // repeated start
  if (w.requestFrom((int)addr, (int)len, (int)true) != (int)len) return false;
  for (size_t i = 0; i < len; i++) buf[i] = w.read();
  return true;
}

bool i2cReadByte(TwoWire &w, uint8_t addr, uint8_t reg, uint8_t &val) {
  return i2cReadBytes(w, addr, reg, &val, 1);
}

// In đúng 20 ký tự (đệm space để không dính ký tự cũ)
void lcdPrintLine(uint8_t row, const char* text) {
  char buf[21];
  size_t n = strlen(text);
  if (n > 20) n = 20;
  memcpy(buf, text, n);
  for (size_t i = n; i < 20; i++) buf[i] = ' ';
  buf[20] = '\0';
  lcd->setCursor(0, row);
  lcd->print(buf);
}

// Format số có dấu +/-, độ rộng cố định
void fmtSigned(char* out, size_t outsz, float v, uint8_t width, uint8_t prec) {
  char num[16];
  float av = (v < 0) ? -v : v;
  dtostrf(av, 0, prec, num);               // "0.00"
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

uint8_t detectLCDAddr() {
  for (uint8_t a : { (uint8_t)0x27, (uint8_t)0x3F }) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission(true) == 0) return a;
  }
  return 0;
}

bool probeMPU(uint8_t addr, uint8_t &id) {
  return i2cReadByte(Wire1, addr, REG_WHO_AM_I, id);
}

bool initMPU(uint8_t addr) {
  // Wake
  if (!i2cWriteByte(Wire1, addr, REG_PWR_MGMT_1, 0x00)) return false;
  delay(50);

  // DLPF ~ 21Hz
  if (!i2cWriteByte(Wire1, addr, REG_CONFIG, 0x04)) return false;

  // ~100Hz sample (1kHz/(1+9))
  if (!i2cWriteByte(Wire1, addr, REG_SMPLRT_DIV, 9)) return false;

  // Gyro ±500 dps
  if (!i2cWriteByte(Wire1, addr, REG_GYRO_CONFIG, 0x08)) return false;

  // Accel ±8g
  if (!i2cWriteByte(Wire1, addr, REG_ACCEL_CONFIG, 0x10)) return false;

  return true;
}

bool findAndStartMPU() {
  mpuAddr = 0;
  whoami = 0;

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

// ================== SETUP ==================
void setup() {
  // LCD bus (Wire) on GPIO6/7
  Wire.begin(LCD_SDA, LCD_SCL);
  Wire.setClock(LCD_I2C_FREQ);

  uint8_t lcdAddr = detectLCDAddr();
  if (lcdAddr == 0x3F) lcd = &lcd3F;
  else lcd = &lcd27; // mặc định

  lcd->begin(20, 4);
  lcd->setBacklight(255);

  lcdPrintLine(0, "ESP32-S3  MPU6DOF");
  lcdPrintLine(1, "LCD OK  I2C6/7   ");
  lcdPrintLine(2, "MPU I2C1/2 Init  ");
  lcdPrintLine(3, "Please wait...   ");

  // MPU bus (Wire1) on GPIO1/2
  Wire1.begin(MPU_SDA, MPU_SCL);
  Wire1.setClock(MPU_I2C_FREQ);

  delay(300);
}

// ================== LOOP ==================
void loop() {
  static uint32_t lastRetry = 0;

  if (!mpuAddr) {
    // Thử kết nối lại mỗi 1 giây
    if (millis() - lastRetry >= 1000) {
      lastRetry = millis();

      if (findAndStartMPU()) {
        char l3[21];
        snprintf(l3, sizeof(l3), "MPU OK 0x%02X WHO%02X", mpuAddr, whoami);
        lcdPrintLine(0, "ESP32-S3  MPU6DOF");
        lcdPrintLine(1, "Reading data...  ");
        lcdPrintLine(2, "                  ");
        lcdPrintLine(3, l3);
        delay(500);
      } else {
        lcdPrintLine(0, "MPU NOT FOUND!    ");
        lcdPrintLine(1, "Check SDA=1 SCL=2 ");
        lcdPrintLine(2, "Addr 0x68/0x69    ");
        lcdPrintLine(3, "Retrying...       ");
      }
    }
    return;
  }

  uint8_t raw[14];
  if (!i2cReadBytes(Wire1, mpuAddr, REG_ACCEL_XOUT_H, raw, 14)) {
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
  int16_t tempRaw = (int16_t)((raw[6] << 8) | raw[7]);
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

  float tempC = (tempRaw / 340.0f) + 36.53f;

  // ==== Build 4 lines exactly fitting 20 cols ====
  char sax[8], say[8], saz[8];
  char sgx[7], sgy[7], sgz[7];
  char st[8];

  fmtSigned(sax, sizeof(sax), ax_g, 6, 2);
  fmtSigned(say, sizeof(say), ay_g, 6, 2);
  fmtSigned(saz, sizeof(saz), az_g, 6, 2);

  fmtSigned(sgx, sizeof(sgx), gx_dps, 5, 1);
  fmtSigned(sgy, sizeof(sgy), gy_dps, 5, 1);
  fmtSigned(sgz, sizeof(sgz), gz_dps, 5, 1);

  dtostrf(tempC, 5, 2, st); // "25.00"

  char line0[21], line1[21], line2[21], line3[21];

  // 20x4 format:
  // L0: "AX:+0.00 AY:+0.00"
  // L1: "AZ:+0.00 T:25.00C"
  // L2: "GX:+0.0 GY:+0.0   "
  // L3: "GZ:+0.0 WHO:0x68   "
  snprintf(line0, sizeof(line0), "AX:%s AY:%s", sax, say);
  snprintf(line1, sizeof(line1), "AZ:%s T:%sC",  saz, st);
  snprintf(line2, sizeof(line2), "GX:%s GY:%s",   sgx, sgy);
  snprintf(line3, sizeof(line3), "GZ:%s WHO:0x%02X", sgz, mpuAddr);

  lcdPrintLine(0, line0);
  lcdPrintLine(1, line1);
  lcdPrintLine(2, line2);
  lcdPrintLine(3, line3);

  delay(300);
}
