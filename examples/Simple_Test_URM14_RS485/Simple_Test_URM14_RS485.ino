/*
  URM14 (RS485) + TTL-RS485 module  -> ESP32-S3
  Hiển thị lên LCD2004 I2C (SDA=GPIO6, SCL=GPIO7)
  - Giữ nguyên logic đọc URM14 như bản đang chạy tốt: trigger mode + đọc eDistance
  - Bỏ hoàn toàn Serial Monitor
*/

#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "DFRobot_RTU.h"

// ===================== USER CONFIG =====================
// UART RS485 (giữ như bản bạn đang dùng chạy OK)
static const int URM14_RX_PIN = 9;   // ESP32-S3 RX  (nhận từ TXD của module)
static const int URM14_TX_PIN = 3;   // ESP32-S3 TX  (gửi ra RXD của module)
static const uint32_t URM14_BAUD = 19200;

// LCD2004 I2C
static const int LCD_SDA_PIN = 6;
static const int LCD_SCL_PIN = 7;
// =======================================================

HardwareSerial RS485Serial(1);
DFRobot_RTU modbus(&RS485Serial);

// ================ URM14_RS485 define (GIỮ NGUYÊN) =================
#define SLAVE_ADDR                ((uint16_t)0x0C)

#define TEMP_CPT_SEL_BIT          ((uint16_t)0x01)
#define TEMP_CPT_ENABLE_BIT       ((uint16_t)0x01 << 1)
#define MEASURE_MODE_BIT          ((uint16_t)0x01 << 2)
#define MEASURE_TRIG_BIT          ((uint16_t)0x01 << 3)

typedef enum {
  ePid,
  eVid,
  eAddr,
  eComBaudrate,
  eComParityStop,
  eDistance,
  eInternalTempreture,
  eExternTempreture,
  eControl,
  eNoise
} eRegIndex_t;

static volatile uint16_t cr = 0;

// ===================== LCD helper =====================
static bool i2cProbe(uint8_t addr) {
  Wire.beginTransmission(addr);
  return (Wire.endTransmission() == 0);
}

static uint8_t detectLCDAddress() {
  // Ưu tiên 2 địa chỉ phổ biến
  if (i2cProbe(0x27)) return 0x27;
  if (i2cProbe(0x3F)) return 0x3F;

  // Nếu không thấy, quét nhanh vùng thường dùng
  for (uint8_t a = 0x20; a <= 0x3F; a++) {
    if (i2cProbe(a)) return a;
  }
  // fallback
  return 0x27;
}

LiquidCrystal_I2C *lcd = nullptr;

static void lcdPrintLine(uint8_t row, const char *text) {
  // In 20 ký tự, tự padding bằng khoảng trắng để không còn ký tự cũ
  char buf[21];
  size_t n = strlen(text);
  if (n > 20) n = 20;
  memset(buf, ' ', 20);
  memcpy(buf, text, n);
  buf[20] = '\0';

  lcd->setCursor(0, row);
  lcd->print(buf);
}

// ===================== Modbus helpers (GIỮ NGUYÊN) =====================
static uint16_t readData(uint16_t addr, eRegIndex_t reg) {
  return modbus.readHoldingRegister(addr, reg);
}

static void writeData(uint16_t addr, eRegIndex_t reg, uint16_t data) {
  modbus.writeHoldingRegister(addr, reg, data);
}

// ===================== URM14 logic (GIỮ NGUYÊN) =====================
static void URM14_init() {
  cr |= MEASURE_MODE_BIT;                 // Trigger mode
  cr &= ~(uint16_t)TEMP_CPT_SEL_BIT;      // Internal temperature compensation
  cr &= ~(uint16_t)TEMP_CPT_ENABLE_BIT;   // (giữ y như code mẫu)
  writeData(SLAVE_ADDR, eControl, cr);
}

static float readDistanceMM() {
  cr |= MEASURE_TRIG_BIT;                 // Trigger ranging
  writeData(SLAVE_ADDR, eControl, cr);

  delay(100);                             // >= 30ms theo tài liệu mẫu
  uint16_t raw = readData(SLAVE_ADDR, eDistance);

  // one LSB = 0.1 mm  => mm = raw / 10.0
  return (float)raw / 10.0f;
}

// ===================== Arduino =====================
void setup() {
  // I2C cho LCD2004
  Wire.begin(LCD_SDA_PIN, LCD_SCL_PIN);
  Wire.setClock(100000);
  delay(50);

  uint8_t lcdAddr = detectLCDAddress();
  lcd = new LiquidCrystal_I2C(lcdAddr, 20, 4);
  lcd->init();
  lcd->backlight();

  // UART RS485
  RS485Serial.begin(URM14_BAUD, SERIAL_8N1, URM14_RX_PIN, URM14_TX_PIN);
  delay(200);

  URM14_init();

  // Splash
  lcdPrintLine(0, "URM14 RS485 - ESP32S3");
  lcdPrintLine(1, "Init OK");
  {
    char line2[21];
    snprintf(line2, sizeof(line2), "LCD:0x%02X SDA%d SCL%d", lcdAddr, LCD_SDA_PIN, LCD_SCL_PIN);
    lcdPrintLine(2, line2);

    char line3[21];
    snprintf(line3, sizeof(line3), "UART RX%d TX%d %lu", URM14_RX_PIN, URM14_TX_PIN, (unsigned long)URM14_BAUD);
    lcdPrintLine(3, line3);
  }

  delay(800);
}

void loop() {
  static uint32_t last = 0;
  const uint32_t periodMs = 200; // GIỮ NGUYÊN chu ky doc

  if (millis() - last >= periodMs) {
    last = millis();

    float mm = readDistanceMM();
    float cm = mm / 10.0f;

    char l1[21], l2[21], l3[21];

    // Dòng 0: tiêu đề cố định (không đổi)
    lcdPrintLine(0, "URM14 Distance (Trig)");

    // Dòng 1: mm
    snprintf(l1, sizeof(l1), "Dist: %7.1f mm", mm);
    lcdPrintLine(1, l1);

    // Dòng 2: cm
    snprintf(l2, sizeof(l2), "      %7.2f cm", cm);
    lcdPrintLine(2, l2);

    // Dòng 3: thông tin UART cố định
    snprintf(l3, sizeof(l3), "RX%d TX%d  %lu", URM14_RX_PIN, URM14_TX_PIN, (unsigned long)URM14_BAUD);
    lcdPrintLine(3, l3);
  }
}
