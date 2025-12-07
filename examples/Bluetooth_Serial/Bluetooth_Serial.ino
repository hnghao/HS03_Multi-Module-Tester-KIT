#include <Arduino.h>

// ================= CẤU HÌNH PHẦN CỨNG =================
// UART dùng để giao tiếp với Bluetooth
#define BT_RX_PIN   2    // ESP32-S3 nhận dữ liệu (RX)  <-- TX của JDY-33 / HC-05
#define BT_TX_PIN   1    // ESP32-S3 gửi dữ liệu (TX)  --> RX của JDY-33 / HC-05

// Baudrate AT (thử 9600 trước, nếu không được thì đổi thành 38400)
#define BT_BAUD     9600

HardwareSerial BTSerial(1);  // Dùng UART1

// Danh sách lệnh AT tự động gửi
const char* atCommands[] = {
  "AT",           // Kiểm tra module còn sống không
  "AT+VERSION?",  // Xem version firmware
  "AT+NAME?",     // Xem tên hiện tại
  "AT+UART?",     // Xem cấu hình UART hiện tại
  "AT+ROLE?"      // Xem vai trò (0: slave, 1: master - tùy module)
};
const uint8_t NUM_AT_COMMANDS = sizeof(atCommands) / sizeof(atCommands[0]);

// =============== HÀM GỬI & ĐỌC PHẢN HỒI AT ===============

void sendAT(const char* cmd) {
  Serial.print(F("\n>> Gửi lệnh: "));
  Serial.println(cmd);

  // Gửi lệnh đến module Bluetooth (kèm CRLF)
  BTSerial.print(cmd);
  BTSerial.print("\r\n");
}

// Đọc phản hồi từ module trong khoảng timeoutMs (ms)
void readBTResponse(uint16_t timeoutMs) {
  uint32_t start = millis();

  Serial.println(F("<< Phản hồi:"));

  while (millis() - start < timeoutMs) {
    while (BTSerial.available()) {
      char c = BTSerial.read();
      Serial.write(c);  // In thẳng ra Serial Monitor
    }
  }

  Serial.println();
  Serial.println(F("---- Hết phản hồi ----"));
}

// Gửi lần lượt các lệnh AT để test
void autoTestBluetooth() {
  Serial.println(F("\n===== BẮT ĐẦU KIỂM TRA AT VỚI MODULE BLUETOOTH ====="));

  for (uint8_t i = 0; i < NUM_AT_COMMANDS; i++) {
    sendAT(atCommands[i]);     // Gửi lệnh
    readBTResponse(800);       // Đọc phản hồi ~800ms
    delay(400);                // Nghỉ một chút giữa các lệnh
  }

  Serial.println(F("===== KẾT THÚC KIỂM TRA TỰ ĐỘNG =====\n"));
  Serial.println(F("Bây giờ bạn có thể gõ thêm lệnh AT trong Serial Monitor nếu muốn."));
  Serial.println(F("Ví dụ: AT, AT+NAME=TEST_BT, AT+ROLE=0, ..."));
}

// ================= SETUP & LOOP =================

void setup() {
  // Serial USB để debug
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }

  Serial.println();
  Serial.println(F("ESP32-S3 <-> JDY-33 / HC-05 - Tự động gửi lệnh AT kiểm tra"));
  Serial.println(F("Mở Serial Monitor 115200, chọn Both NL & CR."));

  // Khởi động UART1 cho Bluetooth
  BTSerial.begin(BT_BAUD, SERIAL_8N1, BT_RX_PIN, BT_TX_PIN);
  Serial.print(F("Đang dùng UART1 với baud: "));
  Serial.println(BT_BAUD);

  delay(500);  // Đợi module ổn định 1 chút

  // Gửi một loạt lệnh AT tự động để kiểm tra
  autoTestBluetooth();
}

void loop() {
  // Phần này tùy chọn: tạo "cầu nối" để có thể gõ lệnh AT thủ công

  // Dữ liệu từ Bluetooth -> PC
  if (BTSerial.available()) {
    char c = BTSerial.read();
    Serial.write(c);
  }

  // Dữ liệu từ PC (Serial Monitor) -> Bluetooth
  if (Serial.available()) {
    char c = Serial.read();
    BTSerial.write(c);
  }
}
