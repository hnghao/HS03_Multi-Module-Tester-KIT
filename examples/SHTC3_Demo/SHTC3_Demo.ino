/**
 * Demo đọc nhiệt độ & độ ẩm từ "SHTC3 RS485 Modbus RTU"
 * Dùng ESP32-S3, giao tiếp qua UART TTL (RS485 converter).
 *
 * Cấu hình cảm biến mặc định:
 *  - Baudrate: 4800 bit/s
 *  - Address : 0x01
 *
 * Có 2 cách đọc dữ liệu:
 *  1. Dùng struct "dataSHTC3"
 *  2. Đọc từng giá trị riêng lẻ (readTemperature, readHumidity)
 */

#include <Arduino.h>
#include "SHTC3Modbus.h"

/* ------------------------------------------------------------------------- */
/* Chân UART nối với mạch RS485 <-> SHTC3                                  */
/* Lưu ý: đây là GPIO của ESP32-S3, không phải chân 2/3 của UNO            */
/*  - GPIO2  : RX từ SHTC3 (RO của module RS485)                            */
/*  - GPIO1  : TX đến SHTC3 (DI của module RS485)                           */
/* ------------------------------------------------------------------------- */

#define RX_SHT 2   // ESP32-S3 GPIO2  - nhận dữ liệu từ cảm biến
#define TX_SHT 1   // ESP32-S3 GPIO1  - gửi dữ liệu tới cảm biến

/* Dùng SoftwareSerial (giống ví dụ gốc trên UNO) */
SHTC3 demo(RX_SHT, TX_SHT, BAUD_4800, 0x01);

/* Nếu muốn dùng HardwareSerial (Serial/Serial1) thì cần chỉnh lại thư viện
 * để port = &Serial1 và set pin cho Serial1, mình có thể giúp ở bước sau.
 */

/* ------------------------------------------------------------------------- */

void setup()
{
  // Serial debug qua USB
  Serial.begin(115200);
  // Đợi Serial sẵn sàng (trên một số board S3 có thể bỏ while nếu không cần)
  while (!Serial) {
    delay(10);
  }

  Serial.println(F("Start reading SHTC3 Modbus (ESP32-S3)"));
  Serial.println(F("Baudrate: 4800, Address: 0x01"));
  Serial.println(F("UART SHTC3 --> RX=GPIO2, TX=GPIO1"));
  Serial.println(F("--------------------------------------------------"));

  // Khởi động cổng UART dùng trong thư viện với baud 4800
  demo.begin(4800);

  // Nếu cần có thể chỉnh timeout (ms) khi chờ trả lời:
  // demo.setTimeout(1000);  // ví dụ 1 giây
}

/* ------------------------------------------------------------------------- */

void loop()
{
  /* --------------------------- Cách 1: dùng struct ----------------------- */
  dataSHTC3 data = demo.getData();

  Serial.print(F("T = "));
  Serial.print(data.temperatureC);
  Serial.print(F(" °C\t"));

  Serial.print(F("T = "));
  Serial.print(data.temperatureF);
  Serial.print(F(" °F\t"));

  Serial.print(F("RH = "));
  Serial.print(data.humidity);
  Serial.println(F(" %"));

  delay(1000);

  /* ---------------------- Cách 2: đọc từng giá trị -----------------------
   * Nếu muốn test riêng lẻ, bỏ comment các dòng dưới và comment phần struct.
   */

  // Serial.print(F("T (°C) = "));
  // Serial.print(demo.readTemperature()); // °C
  // Serial.println(F(" °C"));

  // Serial.print(F("T (°F) = "));
  // Serial.print(demo.readTemperature(false)); // °F
  // Serial.println(F(" °F"));

  // Serial.print(F("RH (%) = "));
  // Serial.print(demo.readHumidity());
  // Serial.println(F(" %"));

  // Serial.println(F("--- --- ---"));
  // delay(1000);
}
