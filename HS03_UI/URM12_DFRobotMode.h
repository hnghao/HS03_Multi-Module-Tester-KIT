#ifndef URM12_DFROBOT_MODE_H
#define URM12_DFROBOT_MODE_H

#include <Arduino.h>
#include <HardwareSerial.h>
#include <LiquidCrystal_I2C.h>

// ===================== LCD DÙNG CHUNG (đã khai báo sẵn trong HS03_UI) =====================
extern LiquidCrystal_I2C lcd;
extern bool headerEnabled;

// In HS03 bạn đang dùng RX=GPIO9, TX=GPIO3 cho RS485 URM12
#ifndef URM12_ADDR
  #define URM12_ADDR 0x0B
#endif

#ifndef URM12_BAUD
  #define URM12_BAUD 19200UL
#endif

#ifndef URM12_RS485_RX_PIN
  #define URM12_RS485_RX_PIN 9
#endif

#ifndef URM12_RS485_TX_PIN
  #define URM12_RS485_TX_PIN 3
#endif

// Nếu module RS485 cần DE/RE -> set GPIO tại đây. Auto-direction thì để -1.
#ifndef URM12_RS485_DE_PIN
  #define URM12_RS485_DE_PIN (-1)
#endif

// UART instance (UART1) - GIỐNG code bạn test
static HardwareSerial URM12_RS485Serial(1);

// ===================== LCD PRINT (GIỐNG HỆT CODE BẠN) =====================
static inline void urm12LcdPrintLine(uint8_t row, const String &s) {
  lcd.setCursor(0, row);
  String out = s;
  if (out.length() > 20) out.remove(20);
  lcd.print(out);
  for (int i = out.length(); i < 20; i++) lcd.print(' ');
}

// ===================== URM12 REGISTERS (Holding Registers) =====================
static const uint16_t URM12_REG_DISTANCE = 0x0005; // distance, 1cm/LSB
static const uint16_t URM12_REG_CONTROL  = 0x0008; // control register (CR)

// Control bits
static const uint16_t URM12_TEMP_CONFIG_MASK  = 0x0003;      // bit1..0
static const uint16_t URM12_MEASURE_MODE_BIT  = (1u << 2);   // bit2: 0 auto, 1 passive
static const uint16_t URM12_MEASURE_TRIG_BIT  = (1u << 3);   // bit3: trigger once in passive

// ===================== MODBUS CRC16 =====================
static inline uint16_t urm12ModbusCRC16(const uint8_t *buf, size_t len) {
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
static inline void urm12Rs485SetTX(bool enableTX) {
  if (URM12_RS485_DE_PIN < 0) return; // auto-direction module
  digitalWrite(URM12_RS485_DE_PIN, enableTX ? HIGH : LOW);
}

static inline void urm12Rs485WriteFrame(const uint8_t *frame, size_t len) {
  urm12Rs485SetTX(true);
  delayMicroseconds(50);

  URM12_RS485Serial.write(frame, len);
  URM12_RS485Serial.flush();

  delayMicroseconds(50);
  urm12Rs485SetTX(false);
}

static inline bool urm12Rs485ReadExact(uint8_t *out, size_t len, uint32_t timeoutMs) {
  uint32_t t0 = millis();
  size_t got = 0;
  while (got < len && (millis() - t0) < timeoutMs) {
    int c = URM12_RS485Serial.read();
    if (c >= 0) out[got++] = (uint8_t)c;
    else delay(1);
  }
  return (got == len);
}

// ===================== MODBUS RTU (0x03, 0x06) =====================
static inline bool urm12ModbusReadHoldingRegister(uint8_t addr, uint16_t reg, uint16_t &valueOut) {
  uint8_t req[8];
  req[0] = addr;
  req[1] = 0x03;
  req[2] = (uint8_t)(reg >> 8);
  req[3] = (uint8_t)(reg & 0xFF);
  req[4] = 0x00;
  req[5] = 0x01;
  uint16_t crc = urm12ModbusCRC16(req, 6);
  req[6] = (uint8_t)(crc & 0xFF);
  req[7] = (uint8_t)(crc >> 8);

  while (URM12_RS485Serial.available()) URM12_RS485Serial.read();
  urm12Rs485WriteFrame(req, sizeof(req));

  uint8_t resp[7];
  if (!urm12Rs485ReadExact(resp, sizeof(resp), 200)) return false;

  if (resp[0] != addr) return false;
  if (resp[1] != 0x03) return false;
  if (resp[2] != 0x02) return false;

  uint16_t crcR = (uint16_t)resp[5] | ((uint16_t)resp[6] << 8);
  uint16_t crcC = urm12ModbusCRC16(resp, 5);
  if (crcR != crcC) return false;

  valueOut = ((uint16_t)resp[3] << 8) | resp[4];
  return true;
}

static inline bool urm12ModbusWriteSingleRegister(uint8_t addr, uint16_t reg, uint16_t value) {
  uint8_t req[8];
  req[0] = addr;
  req[1] = 0x06;
  req[2] = (uint8_t)(reg >> 8);
  req[3] = (uint8_t)(reg & 0xFF);
  req[4] = (uint8_t)(value >> 8);
  req[5] = (uint8_t)(value & 0xFF);
  uint16_t crc = urm12ModbusCRC16(req, 6);
  req[6] = (uint8_t)(crc & 0xFF);
  req[7] = (uint8_t)(crc >> 8);

  while (URM12_RS485Serial.available()) URM12_RS485Serial.read();
  urm12Rs485WriteFrame(req, sizeof(req));

  uint8_t resp[8];
  if (!urm12Rs485ReadExact(resp, sizeof(resp), 200)) return false;

  uint16_t crcR = (uint16_t)resp[6] | ((uint16_t)resp[7] << 8);
  uint16_t crcC = urm12ModbusCRC16(resp, 6);
  if (crcR != crcC) return false;

  for (int i = 0; i < 6; i++) {
    if (resp[i] != req[i]) return false;
  }
  return true;
}

// ===================== URM12 HIGH-LEVEL (GIỐNG HỆT LOGIC BẠN) =====================
static uint16_t urm12_cr_base = 0;

static inline bool urm12InitPassiveInternalTemp() {
  uint16_t cr = 0;

  if (!urm12ModbusReadHoldingRegister(URM12_ADDR, URM12_REG_CONTROL, cr)) {
    // if cannot read, still push a sane default
    cr = 0x0004; // passive + internal temp (common default)
  }

  cr |= URM12_MEASURE_MODE_BIT;             // passive
  cr &= ~(uint16_t)URM12_TEMP_CONFIG_MASK;  // internal temp (00)
  cr &= ~(uint16_t)URM12_MEASURE_TRIG_BIT;  // ensure trigger bit = 0

  if (!urm12ModbusWriteSingleRegister(URM12_ADDR, URM12_REG_CONTROL, cr)) return false;
  urm12_cr_base = cr;
  delay(50);
  return true;
}

static inline bool urm12TriggerOnce() {
  if (!urm12ModbusWriteSingleRegister(URM12_ADDR, URM12_REG_CONTROL, (uint16_t)(urm12_cr_base | URM12_MEASURE_TRIG_BIT))) return false;
  delay(10);
  if (!urm12ModbusWriteSingleRegister(URM12_ADDR, URM12_REG_CONTROL, urm12_cr_base)) return false;
  return true;
}

static inline bool urm12ReadDistanceCm(uint16_t &distCm) {
  return urm12ModbusReadHoldingRegister(URM12_ADDR, URM12_REG_DISTANCE, distCm);
}

// ===================== MODE API CHO HS03_UI =====================
// Để không bị header/coutdown ghi đè dòng 0 (vì code bạn dùng dòng 0 làm title)
static bool urm12_savedHeaderEnabled = true;

// state machine để giữ y hệt trình tự delay(300)/delay(300) nhưng KHÔNG block toàn loop
enum URM12_RunStage : uint8_t { URM12_STAGE_IDLE=0, URM12_STAGE_WAIT_READ, URM12_STAGE_POST_WAIT, URM12_STAGE_WAIT_RETRY };
static URM12_RunStage urm12_stage = URM12_STAGE_IDLE;
static uint32_t urm12_nextMs = 0;

static inline void startURM12DFRobotMode() {
  // tắt header để giữ nguyên hiển thị dòng 0 giống code bạn
  urm12_savedHeaderEnabled = headerEnabled;
  headerEnabled = false;

  // LCD init phần hiển thị giống setup() của bạn
  lcd.clear();
  urm12LcdPrintLine(0, "URM12 RS485 Modbus");
  urm12LcdPrintLine(1, "Init...");
  urm12LcdPrintLine(2, "                ");
  urm12LcdPrintLine(3, "RX=9 TX=3 I2C6/7");

  // RS485 direction pin (if used)
  if (URM12_RS485_DE_PIN >= 0) {
    pinMode(URM12_RS485_DE_PIN, OUTPUT);
    urm12Rs485SetTX(false);
  }

  // RS485 UART init
  URM12_RS485Serial.begin(URM12_BAUD, SERIAL_8N1, URM12_RS485_RX_PIN, URM12_RS485_TX_PIN);
  delay(200);

  // URM12 init (logic unchanged)
  if (urm12InitPassiveInternalTemp()) {
    urm12LcdPrintLine(1, "Init: OK");
  } else {
    urm12LcdPrintLine(1, "Init: FAIL");
    urm12LcdPrintLine(2, "Check A/B,addr,baud");
  }

  urm12_stage = URM12_STAGE_IDLE;
  urm12_nextMs = millis(); // chạy ngay
}

static inline void updateURM12DFRobotMode(unsigned long now) {
  if ((int32_t)(now - urm12_nextMs) < 0) return;

  if (urm12_stage == URM12_STAGE_IDLE) {
    // Trigger one measurement
    if (!urm12TriggerOnce()) {
      urm12LcdPrintLine(2, "Trigger: FAIL");
      urm12_stage = URM12_STAGE_WAIT_RETRY;
      urm12_nextMs = now + 500;
      return;
    }
    // đợi ~300ms rồi đọc
    urm12_stage = URM12_STAGE_WAIT_READ;
    urm12_nextMs = now + 300;
    return;
  }

  if (urm12_stage == URM12_STAGE_WAIT_READ) {
    uint16_t distCm = 0;
    if (urm12ReadDistanceCm(distCm)) {
      urm12LcdPrintLine(2, "Trigger: OK");
      urm12LcdPrintLine(1, "Distance: " + String(distCm) + " cm");
    } else {
      urm12LcdPrintLine(2, "Read: FAIL (CRC/TO)");
    }
    // delay(300)
    urm12_stage = URM12_STAGE_POST_WAIT;
    urm12_nextMs = now + 300;
    return;
  }

  if (urm12_stage == URM12_STAGE_POST_WAIT) {
    // quay về chu kỳ mới
    urm12_stage = URM12_STAGE_IDLE;
    urm12_nextMs = now;
    return;
  }

  // WAIT_RETRY
  if (urm12_stage == URM12_STAGE_WAIT_RETRY) {
    urm12_stage = URM12_STAGE_IDLE;
    urm12_nextMs = now;
    return;
  }
}

static inline void stopURM12DFRobotMode() {
  // trả lại header cho các mode khác
  headerEnabled = urm12_savedHeaderEnabled;

  // nếu bạn muốn “nhả” UART1 khi thoát mode thì bật dòng dưới:
  // URM12_RS485Serial.end();
}

#endif // URM12_DFROBOT_MODE_H
