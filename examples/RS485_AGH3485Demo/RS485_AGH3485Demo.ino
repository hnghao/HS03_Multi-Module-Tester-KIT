#include <SoftwareSerial.h>

SoftwareSerial rs485(9, 3);  // RX, TX

// Lệnh đọc modbus: 01 03 00 00 00 02 C4 0B
byte readCmd[] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x02, 0xC4, 0x0B};

void setup() {
  Serial.begin(115200);
  rs485.begin(9600);

  Serial.println("AGH3485 Temperature & Humidity Reader Ready!");
}

void loop() {

  // ===== Gửi lệnh truy vấn =====
  rs485.write(readCmd, sizeof(readCmd));
  rs485.flush();   // Đảm bảo gửi xong

  delay(30);       // Đợi cảm biến trả dữ liệu (rất quan trọng)

  // ===== Nhận dữ liệu =====
  if (rs485.available() >= 9) {

    byte buf[9];
    rs485.readBytes(buf, 9);

    // Kiểm tra frame 01 03 04 ...
    if (buf[0] == 0x01 && buf[1] == 0x03 && buf[2] == 0x04) {

      int humidity_raw = (buf[3] << 8) | buf[4];
      int16_t temp_raw = (buf[5] << 8) | buf[6];

      float humidity = humidity_raw / 10.0;
      float temperature = temp_raw / 10.0;

      Serial.print("Humidity: ");
      Serial.print(humidity);
      Serial.print(" %RH   |   Temperature: ");
      Serial.print(temperature);
      Serial.println(" °C");
    }
    else {
      Serial.println("Frame lỗi hoặc lệch dữ liệu!");
    }
  }

  delay(1000);
}
