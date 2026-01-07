#ifndef URM14_DFROBOT_MODE_H
#define URM14_DFROBOT_MODE_H

#include <Arduino.h>
#include <HardwareSerial.h>

// ===================== URM14 RS485 (DFRobot) =====================
// - Giữ nguyên phần hiển thị giống sketch test bạn gửi:
//   Line0: "URM14 Distance (Trig)"
//   Line1: "Dist:  xxxxx.x mm"
//   Line2: "      xxxxx.xx cm"
//   Line3: "RX9 TX3  19200"
// - Encoder click: thoát về menu RS485 (xử lý trong HS03_UI.ino)

// Pin map theo bản test
static const int URM14_RS485_RX_PIN = 9;
static const int URM14_RS485_TX_PIN = 3;

// Nếu module RS485 của bạn cần DE/RE điều khiển hướng:
// - nối DE và /RE chung 1 chân GPIO
// - đổi URM14_RS485_DE_PIN = GPIO đó
// - nếu module auto-direction: để -1
static const int URM14_RS485_DE_PIN = -1;

static const uint8_t  URM14_SLAVE_ADDR = 0x0C;   // default
static const uint32_t URM14_BAUD       = 19200;  // default

// Holding Registers (URM14)
static const uint16_t URM14_REG_DISTANCE = 0x0005; // 0.1mm/LSB (sketch: /10 => mm)
static const uint16_t URM14_REG_CONTROL  = 0x0008; // CR

// Control bits (giữ đúng như sketch test)
static const uint16_t URM14_TEMP_CPT_SEL_BIT    = 0x0001;            // bit0
static const uint16_t URM14_TEMP_CPT_ENABLE_BIT = (uint16_t)(1u<<1); // bit1
static const uint16_t URM14_MEASURE_MODE_BIT    = (uint16_t)(1u<<2); // bit2: 1=trigger mode
static const uint16_t URM14_MEASURE_TRIG_BIT    = (uint16_t)(1u<<3); // bit3: trigger once

// UART1 (như bản test)
static HardwareSerial URM14_RS485Serial(1);

// Các hàm LCD dùng chung trong chương trình chính
extern void lcdPrintLine(uint8_t row, const char *text);
extern bool headerEnabled;

// ===================== MODBUS CRC16 =====================
static uint16_t urm14_modbusCRC16(const uint8_t *buf, size_t len) {
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
static inline void urm14_rs485SetTX(bool enableTX) {
  if (URM14_RS485_DE_PIN < 0) return; // auto-direction module
  digitalWrite(URM14_RS485_DE_PIN, enableTX ? HIGH : LOW);
}

static void urm14_rs485WriteFrame(const uint8_t *frame, size_t len) {
  urm14_rs485SetTX(true);
  delayMicroseconds(50);

  URM14_RS485Serial.write(frame, len);
  URM14_RS485Serial.flush();

  delayMicroseconds(50);
  urm14_rs485SetTX(false);
}

static bool urm14_rs485ReadExact(uint8_t *out, size_t len, uint32_t timeoutMs) {
  uint32_t t0 = millis();
  size_t got = 0;
  while (got < len && (millis() - t0) < timeoutMs) {
    int c = URM14_RS485Serial.read();
    if (c >= 0) out[got++] = (uint8_t)c;
    else delay(1);
  }
  return (got == len);
}

// ===================== MODBUS RTU (0x03, 0x06) =====================
static bool urm14_readHoldingRegister(uint8_t addr, uint16_t reg, uint16_t &valueOut) {
  uint8_t req[8];
  req[0] = addr;
  req[1] = 0x03;
  req[2] = (uint8_t)(reg >> 8);
  req[3] = (uint8_t)(reg & 0xFF);
  req[4] = 0x00;
  req[5] = 0x01;
  uint16_t crc = urm14_modbusCRC16(req, 6);
  req[6] = (uint8_t)(crc & 0xFF);
  req[7] = (uint8_t)(crc >> 8);

  while (URM14_RS485Serial.available()) URM14_RS485Serial.read();
  urm14_rs485WriteFrame(req, sizeof(req));

  uint8_t resp[7];
  if (!urm14_rs485ReadExact(resp, sizeof(resp), 200)) return false;

  if (resp[0] != addr) return false;
  if (resp[1] != 0x03) return false;
  if (resp[2] != 0x02) return false;

  uint16_t crcR = (uint16_t)resp[5] | ((uint16_t)resp[6] << 8);
  uint16_t crcC = urm14_modbusCRC16(resp, 5);
  if (crcR != crcC) return false;

  valueOut = ((uint16_t)resp[3] << 8) | resp[4];
  return true;
}

static bool urm14_writeSingleRegister(uint8_t addr, uint16_t reg, uint16_t value) {
  uint8_t req[8];
  req[0] = addr;
  req[1] = 0x06;
  req[2] = (uint8_t)(reg >> 8);
  req[3] = (uint8_t)(reg & 0xFF);
  req[4] = (uint8_t)(value >> 8);
  req[5] = (uint8_t)(value & 0xFF);
  uint16_t crc = urm14_modbusCRC16(req, 6);
  req[6] = (uint8_t)(crc & 0xFF);
  req[7] = (uint8_t)(crc >> 8);

  while (URM14_RS485Serial.available()) URM14_RS485Serial.read();
  urm14_rs485WriteFrame(req, sizeof(req));

  uint8_t resp[8];
  if (!urm14_rs485ReadExact(resp, sizeof(resp), 200)) return false;

  uint16_t crcR = (uint16_t)resp[6] | ((uint16_t)resp[7] << 8);
  uint16_t crcC = urm14_modbusCRC16(resp, 6);
  if (crcR != crcC) return false;

  for (int i = 0; i < 6; i++) {
    if (resp[i] != req[i]) return false;
  }
  return true;
}

// ===================== MODE STATE =====================
static bool urm14_savedHeaderEnabled = true;
static bool urm14_inited = false;
static uint16_t urm14_cr = 0;

// Stage machine ~ timing giống sketch test (200ms/cycle, wait 100ms sau trigger)
enum URM14Stage : uint8_t {
  URM14_STAGE_TRIGGER = 0,
  URM14_STAGE_WAIT_READ,
  URM14_STAGE_READ,
  URM14_STAGE_POST_WAIT,
  URM14_STAGE_RETRY_WAIT
};

static URM14Stage urm14_stage = URM14_STAGE_TRIGGER;
static uint32_t   urm14_nextMs = 0;

static void URM14_init() {
  // Trigger mode + internal temperature (logic giữ nguyên như sketch test)
  urm14_cr = 0x0000;
  urm14_cr |= URM14_MEASURE_MODE_BIT;   // trigger mode
  urm14_cr &= (uint16_t)(~URM14_TEMP_CPT_SEL_BIT);    // internal temperature
  urm14_cr &= (uint16_t)(~URM14_TEMP_CPT_ENABLE_BIT); // keep like test

  (void)urm14_writeSingleRegister(URM14_SLAVE_ADDR, URM14_REG_CONTROL, urm14_cr);
}

// ===================== PUBLIC API =====================
static void startURM14DFRobotMode() {
  // Tắt header để giữ đúng hiển thị URM14 ở dòng 0
  urm14_savedHeaderEnabled = headerEnabled;
  headerEnabled = false;

  if (URM14_RS485_DE_PIN >= 0) {
    pinMode(URM14_RS485_DE_PIN, OUTPUT);
    urm14_rs485SetTX(false);
  }

  URM14_RS485Serial.begin(URM14_BAUD, SERIAL_8N1, URM14_RS485_RX_PIN, URM14_RS485_TX_PIN);
  delay(50);

  URM14_init();

  // Splash nhanh
  lcdPrintLine(0, "URM14 RS485 - ESP32S3");
  lcdPrintLine(1, "Init...");
  lcdPrintLine(2, "                ");
  lcdPrintLine(3, "RX9 TX3  19200");

  urm14_inited = true;
  urm14_stage = URM14_STAGE_TRIGGER;
  urm14_nextMs = millis() + 100;
}

static void stopURM14DFRobotMode() {
  headerEnabled = urm14_savedHeaderEnabled;

  lcdPrintLine(1, "");
  lcdPrintLine(2, "");
  lcdPrintLine(3, "");
}

static void updateURM14DFRobotMode(unsigned long now) {
  if (!urm14_inited) return;
  if ((int32_t)(now - urm14_nextMs) < 0) return;

  char l3[21];
  snprintf(l3, sizeof(l3), "RX%d TX%d  %lu", URM14_RS485_RX_PIN, URM14_RS485_TX_PIN, (unsigned long)URM14_BAUD);

  switch (urm14_stage) {
    case URM14_STAGE_TRIGGER: {
      urm14_cr |= URM14_MEASURE_TRIG_BIT;
      bool ok = urm14_writeSingleRegister(URM14_SLAVE_ADDR, URM14_REG_CONTROL, urm14_cr);

      lcdPrintLine(0, "URM14 Distance (Trig)");

      if (!ok) {
        lcdPrintLine(1, "Trigger: FAIL");
        lcdPrintLine(2, "Check A/B,addr,baud");
        lcdPrintLine(3, l3);
        urm14_stage = URM14_STAGE_RETRY_WAIT;
        urm14_nextMs = now + 300;
        break;
      }

      urm14_stage = URM14_STAGE_WAIT_READ;
      urm14_nextMs = now + 100; // như delay(100)
    } break;

    case URM14_STAGE_WAIT_READ:
      urm14_stage = URM14_STAGE_READ;
      urm14_nextMs = now;
      break;

    case URM14_STAGE_READ: {
      uint16_t raw = 0;
      bool ok = urm14_readHoldingRegister(URM14_SLAVE_ADDR, URM14_REG_DISTANCE, raw);

      lcdPrintLine(0, "URM14 Distance (Trig)");

      if (ok) {
        float distanceMM = (float)raw / 10.0f;  // giữ đúng như sketch
        float distanceCM = distanceMM / 10.0f;

        char l1[21], l2[21];
        snprintf(l1, sizeof(l1), "Dist: %7.1f mm", distanceMM);
        snprintf(l2, sizeof(l2), "      %7.2f cm", distanceCM);

        lcdPrintLine(1, l1);
        lcdPrintLine(2, l2);
        lcdPrintLine(3, l3);
      } else {
        lcdPrintLine(1, "Read: FAIL (CRC/TO)");
        lcdPrintLine(2, "Retry...");
        lcdPrintLine(3, l3);
      }

      urm14_stage = URM14_STAGE_POST_WAIT;
      urm14_nextMs = now + 100; // hoàn tất 200ms/cycle
    } break;

    case URM14_STAGE_POST_WAIT:
      urm14_stage = URM14_STAGE_TRIGGER;
      urm14_nextMs = now;
      break;

    case URM14_STAGE_RETRY_WAIT:
      urm14_stage = URM14_STAGE_TRIGGER;
      urm14_nextMs = now;
      break;
  }
}

#endif // URM14_DFROBOT_MODE_H
