#ifndef RS485_AGH3485_MODE_H
#define RS485_AGH3485_MODE_H

#include <Arduino.h>
#include "Display.h"
#include <HardwareSerial.h>

// UART RS485 dùng chung với mode SHTC3.
// Trong RS485SHTC3Mode.h bạn ĐÃ có định nghĩa thực thể RS485Serial,
// ví dụ:  HardwareSerial RS485Serial(1);
extern SoftwareSerial RS485Serial;

// Nếu chưa có macro chân RX/TX, dùng mặc định như code SHTC3 (RX=2, TX=1)
#ifndef RS485_RX_PIN
#define RS485_RX_PIN 2
#endif

#ifndef RS485_TX_PIN
#define RS485_TX_PIN 1
#endif

// Lệnh đọc modbus: 01 03 00 00 00 02 C4 0B
static const uint8_t agh3485ReadCmd[] = {
  0x01, 0x03, 0x00, 0x00, 0x00, 0x02, 0xC4, 0x0B
};

static const uint8_t AGH3485_FRAME_LEN = 9;

static unsigned long aghLastQueryTime   = 0;
static unsigned long aghCmdSentTime     = 0;
static const unsigned long AGH_QUERY_INTERVAL_MS   = 1000;
static const unsigned long AGH_RESPONSE_DELAY_MS   = 30;
static const unsigned long AGH_RESPONSE_TIMEOUT_MS = 200;

enum AGH3485State {
  AGH_IDLE = 0,
  AGH_WAIT_RESPONSE
};

static AGH3485State aghState = AGH_IDLE;

// --------------------------------------------------
// Khởi động mode AGH3485
// --------------------------------------------------
inline void startRS485AGH3485Mode() {
  aghState         = AGH_IDLE;
  aghLastQueryTime = 0;
  aghCmdSentTime   = 0;

  // // Khởi động UART RS485 (nếu RS485Serial đã begin ở nơi khác thì lệnh này cũng không sao)
  // RS485Serial.begin(9600, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);
  // ĐÚNG (dùng EspSoftwareSerial):
  RS485Serial.begin(9600, SWSERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);
  // Xoá buffer cũ
  while (RS485Serial.available()) {
    RS485Serial.read();
  }

  lcd.clear();
  // Dòng 0: header "RS485 2/3 NNs" sẽ được drawRS485Header() lo
  lcdPrintLine(1, "AGH3485 RS485     ");
  lcdPrintLine(2, "Dang khoi dong...");
  lcdPrintLine(3, "Nhan nut de thoat");
}

// --------------------------------------------------
// Cập nhật đo AGH3485 (gọi liên tục trong loop)
// --------------------------------------------------
inline void updateRS485AGH3485Mode(unsigned long now) {
  switch (aghState) {
    case AGH_IDLE:
      if (now - aghLastQueryTime >= AGH_QUERY_INTERVAL_MS) {
        // Gửi lệnh đọc
        while (RS485Serial.available()) {
          RS485Serial.read();
        }

        RS485Serial.write(agh3485ReadCmd, sizeof(agh3485ReadCmd));
        RS485Serial.flush();

        aghCmdSentTime = now;
        aghState       = AGH_WAIT_RESPONSE;
      }
      break;

    case AGH_WAIT_RESPONSE:
      if (RS485Serial.available() >= AGH3485_FRAME_LEN) {
        uint8_t buf[AGH3485_FRAME_LEN];
        RS485Serial.readBytes(buf, AGH3485_FRAME_LEN);

        if (buf[0] == 0x01 && buf[1] == 0x03 && buf[2] == 0x04) {
          int      humidityRaw = (buf[3] << 8) | buf[4];
          int16_t  tempRaw     = (int16_t)((buf[5] << 8) | buf[6]);

          float humidity    = humidityRaw / 10.0f;
          float temperature = tempRaw / 10.0f;

          char lineHum[21];
          char lineTemp[21];

          // Giữ style hiển thị gần giống chương trình mẫu
          snprintf(lineHum,  sizeof(lineHum),  "Hum: %5.1f %%RH", humidity);
          snprintf(lineTemp, sizeof(lineTemp), "Temp:%5.1f C   ", temperature);

          lcdPrintLine(1, lineHum);
          lcdPrintLine(2, lineTemp);
          // Xoá dòng báo lỗi cũ (nếu có)
          lcdPrintLine(3, "                    ");
        } else {
          // Frame sai format
          lcdPrintLine(3, "Frame loi / lech!  ");
        }

        aghLastQueryTime = now;
        aghState         = AGH_IDLE;
      } else if (now - aghCmdSentTime > AGH_RESPONSE_TIMEOUT_MS) {
        // Timeout không đủ dữ liệu
        lcdPrintLine(3, "Khong du du lieu  ");
        aghLastQueryTime = now;
        aghState         = AGH_IDLE;

        while (RS485Serial.available()) {
          RS485Serial.read();
        }
      }
      break;
  }
}

// --------------------------------------------------
// Thoát mode AGH3485
// --------------------------------------------------
inline void stopRS485AGH3485Mode() {
  aghState = AGH_IDLE;
  lcdPrintLine(1, " ");
  lcdPrintLine(2, " ");
  lcdPrintLine(3, " ");
}

#endif // RS485_AGH3485_MODE_H
