#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// ===================== LCD2004 I2C (ESP32-S3) =====================
static const uint8_t LCD_ADDR = 0x27;   // change to 0x3F if needed
static const int LCD_SDA_PIN = 6;
static const int LCD_SCL_PIN = 7;

LiquidCrystal_I2C lcd(LCD_ADDR, 20, 4);

static void lcdPrintLine(uint8_t row, const String &s) {
  lcd.setCursor(0, row);
  String out = s;
  if (out.length() > 20) out.remove(20);
  lcd.print(out);
  // pad spaces to clear old chars
  for (int i = out.length(); i < 20; i++) lcd.print(' ');
}

// ===================== USER CONFIG (URM12 RS485) =====================
static const uint8_t  URM12_ADDR = 0x0B;   // default slave address
static const uint32_t URM12_BAUD = 19200;  // default baudrate

static const int RS485_RX_PIN = 9;         // ESP32-S3 RX  <- RO of RS485 module
static const int RS485_TX_PIN = 3;         // ESP32-S3 TX  -> DI of RS485 module

// If your RS485 module needs direction control (DE/RE):
// - Connect DE and /RE together to a GPIO
// - Set RS485_DE_PIN to that GPIO number
// - If your module is auto-direction, keep -1.
static const int RS485_DE_PIN = -1;

// ===================== URM12 REGISTERS (Holding Registers) =====================
static const uint16_t REG_DISTANCE = 0x0005; // distance, 1cm/LSB
static const uint16_t REG_CONTROL  = 0x0008; // control register (CR)

// Control bits
static const uint16_t TEMP_CONFIG_MASK  = 0x0003;      // bit1..0
static const uint16_t MEASURE_MODE_BIT  = (1u << 2);   // bit2: 0 auto, 1 passive
static const uint16_t MEASURE_TRIG_BIT  = (1u << 3);   // bit3: trigger once in passive

// UART instance (UART1)
HardwareSerial RS485Serial(1);

// ===================== MODBUS CRC16 =====================
static uint16_t modbusCRC16(const uint8_t *buf, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= buf[i];
    for (int b = 0; b < 8; b++) {
      if (crc & 0x0001) crc = (crc >> 1) ^ 0xA001;
      else             crc = (crc >> 1);
    }
  }
  return crc;
}

// ===================== RS485 SEND/RECV HELPERS =====================
static void rs485SetTX(bool enableTX) {
  if (RS485_DE_PIN < 0) return; // auto-direction module
  digitalWrite(RS485_DE_PIN, enableTX ? HIGH : LOW);
}

static void rs485WriteFrame(const uint8_t *frame, size_t len) {
  rs485SetTX(true);
  delayMicroseconds(50);

  RS485Serial.write(frame, len);
  RS485Serial.flush();

  delayMicroseconds(50);
  rs485SetTX(false);
}

static bool rs485ReadExact(uint8_t *out, size_t len, uint32_t timeoutMs) {
  uint32_t t0 = millis();
  size_t got = 0;
  while (got < len && (millis() - t0) < timeoutMs) {
    int c = RS485Serial.read();
    if (c >= 0) out[got++] = (uint8_t)c;
    else delay(1);
  }
  return (got == len);
}

// ===================== MODBUS RTU (0x03, 0x06) =====================
static bool modbusReadHoldingRegister(uint8_t addr, uint16_t reg, uint16_t &valueOut) {
  uint8_t req[8];
  req[0] = addr;
  req[1] = 0x03;
  req[2] = (uint8_t)(reg >> 8);
  req[3] = (uint8_t)(reg & 0xFF);
  req[4] = 0x00;
  req[5] = 0x01;
  uint16_t crc = modbusCRC16(req, 6);
  req[6] = (uint8_t)(crc & 0xFF);
  req[7] = (uint8_t)(crc >> 8);

  while (RS485Serial.available()) RS485Serial.read();
  rs485WriteFrame(req, sizeof(req));

  uint8_t resp[7];
  if (!rs485ReadExact(resp, sizeof(resp), 200)) return false;

  if (resp[0] != addr) return false;
  if (resp[1] != 0x03) return false;
  if (resp[2] != 0x02) return false;

  uint16_t crcR = (uint16_t)resp[5] | ((uint16_t)resp[6] << 8);
  uint16_t crcC = modbusCRC16(resp, 5);
  if (crcR != crcC) return false;

  valueOut = ((uint16_t)resp[3] << 8) | resp[4];
  return true;
}

static bool modbusWriteSingleRegister(uint8_t addr, uint16_t reg, uint16_t value) {
  uint8_t req[8];
  req[0] = addr;
  req[1] = 0x06;
  req[2] = (uint8_t)(reg >> 8);
  req[3] = (uint8_t)(reg & 0xFF);
  req[4] = (uint8_t)(value >> 8);
  req[5] = (uint8_t)(value & 0xFF);
  uint16_t crc = modbusCRC16(req, 6);
  req[6] = (uint8_t)(crc & 0xFF);
  req[7] = (uint8_t)(crc >> 8);

  while (RS485Serial.available()) RS485Serial.read();
  rs485WriteFrame(req, sizeof(req));

  uint8_t resp[8];
  if (!rs485ReadExact(resp, sizeof(resp), 200)) return false;

  uint16_t crcR = (uint16_t)resp[6] | ((uint16_t)resp[7] << 8);
  uint16_t crcC = modbusCRC16(resp, 6);
  if (crcR != crcC) return false;

  for (int i = 0; i < 6; i++) {
    if (resp[i] != req[i]) return false;
  }
  return true;
}

// ===================== URM12 HIGH-LEVEL =====================
static uint16_t cr_base = 0;

static bool urm12InitPassiveInternalTemp() {
  uint16_t cr = 0;

  if (!modbusReadHoldingRegister(URM12_ADDR, REG_CONTROL, cr)) {
    // if cannot read, still push a sane default
    cr = 0x0004; // passive + internal temp (common default)
  }

  cr |= MEASURE_MODE_BIT;            // passive
  cr &= ~(uint16_t)TEMP_CONFIG_MASK; // internal temp (00)
  cr &= ~(uint16_t)MEASURE_TRIG_BIT; // ensure trigger bit = 0

  if (!modbusWriteSingleRegister(URM12_ADDR, REG_CONTROL, cr)) return false;
  cr_base = cr;
  delay(50);
  return true;
}

static bool urm12TriggerOnce() {
  if (!modbusWriteSingleRegister(URM12_ADDR, REG_CONTROL, (uint16_t)(cr_base | MEASURE_TRIG_BIT))) return false;
  delay(10);
  if (!modbusWriteSingleRegister(URM12_ADDR, REG_CONTROL, cr_base)) return false;
  return true;
}

static bool urm12ReadDistanceCm(uint16_t &distCm) {
  return modbusReadHoldingRegister(URM12_ADDR, REG_DISTANCE, distCm);
}

// ===================== ARDUINO =====================
void setup() {
  // LCD init
  Wire.begin(LCD_SDA_PIN, LCD_SCL_PIN);
  lcd.init();
  lcd.backlight();
  lcd.clear();

  lcdPrintLine(0, "URM12 RS485 Modbus");
  lcdPrintLine(1, "Init...");
  lcdPrintLine(2, "                ");
  lcdPrintLine(3, "RX=9 TX=3 I2C6/7");

  // RS485 direction pin (if used)
  if (RS485_DE_PIN >= 0) {
    pinMode(RS485_DE_PIN, OUTPUT);
    rs485SetTX(false);
  }

  // RS485 UART init
  RS485Serial.begin(URM12_BAUD, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);
  delay(200);

  // URM12 init (logic unchanged)
  if (urm12InitPassiveInternalTemp()) {
    lcdPrintLine(1, "Init: OK");
  } else {
    lcdPrintLine(1, "Init: FAIL");
    lcdPrintLine(2, "Check A/B,addr,baud");
  }
}

void loop() {
  // Trigger one measurement (passive mode), then read after ~300ms (logic unchanged)
  if (!urm12TriggerOnce()) {
    lcdPrintLine(2, "Trigger: FAIL");
    delay(500);
    return;
  }

  delay(300);

  uint16_t distCm = 0;
  if (urm12ReadDistanceCm(distCm)) {
    lcdPrintLine(2, "Trigger: OK");
    lcdPrintLine(1, "Distance: " + String(distCm) + " cm");
  } else {
    lcdPrintLine(2, "Read: FAIL (CRC/TO)");
  }

  delay(300);
}
