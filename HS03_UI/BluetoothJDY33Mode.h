#pragma once
#include <Arduino.h>
#include <LiquidCrystal_I2C.h>

// Dùng chung lcd khai báo trong HS03_UI.ino
extern LiquidCrystal_I2C lcd;

// -------------------- JDY-33 UART --------------------
#define JDY_TX_PIN   1   // TX ESP32-S3 -> RX JDY-33
#define JDY_RX_PIN   2   // RX ESP32-S3 <- TX JDY-33

#define JDY_BAUD     9600
#define AT_RESPONSE_TIMEOUT 1000  // thời gian chờ phản hồi (ms)

// -------------------- Hàm nội bộ gửi AT -------------------
static bool jdySendAT(const char *label, const char *cmd) {
  // Xoá buffer cũ (nếu có)
  while (Serial2.available()) {
    Serial2.read();
  }

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("JDY-33 AT TEST   ");

  lcd.setCursor(0, 1);
  lcd.print("CMD: ");
  lcd.print(label);

  lcd.setCursor(0, 2);
  lcd.print("Sending...       ");

  // Gửi lệnh AT tới JDY-33 (kết thúc \r\n)
  Serial2.print(cmd);
  Serial2.print("\r\n");

  String resp;
  unsigned long start = millis();

  while (millis() - start < AT_RESPONSE_TIMEOUT) {
    while (Serial2.available()) {
      char c = (char)Serial2.read();
      resp += c;
    }
  }

  resp.trim();             // bỏ \r \n và khoảng trắng 2 đầu
  bool ok = resp.length() > 0;

  // Hiển thị trạng thái
  lcd.setCursor(0, 2);
  lcd.print("Status: ");
  if (ok) {
    lcd.print("OK   ");
  } else {
    lcd.print("FAIL ");
  }

  // Hiển thị một phần nội dung phản hồi (nếu có)
  lcd.setCursor(0, 3);
  lcd.print("Resp: ");

  if (resp.length() == 0) {
    lcd.print("No data      ");
  } else {
    resp.replace('\r', ' ');
    resp.replace('\n', ' ');
    // cắt ngắn để vừa 20 ký tự (từ cột 0)
    if (resp.length() > 14) {
      resp = resp.substring(0, 14);
    }
    lcd.setCursor(6, 3);       // "Resp: " là 6 ký tự
    lcd.print("              "); // xoá phần cũ
    lcd.setCursor(6, 3);
    lcd.print(resp);
  }

  delay(1500); // cho bạn thời gian đọc mỗi lệnh
  return ok;
}

// -------------------- Mode JDY-33 (blocking) -------------------
void startBluetoothJDY33Mode() {
  // Khởi tạo UART cho JDY-33
  Serial2.begin(JDY_BAUD, SERIAL_8N1, JDY_RX_PIN, JDY_TX_PIN);
  delay(200);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("JDY-33 STARTING");
  lcd.setCursor(0, 1);
  lcd.print("Init UART...      ");
  delay(500);

  lcd.setCursor(0, 1);
  lcd.print("Init UART: OK     ");
  delay(500);

  // Gửi các lệnh AT và hiển thị kết quả trên LCD
  bool allOK = true;

  allOK &= jdySendAT("AT",           "AT");
  allOK &= jdySendAT("Set NAME1",    "AT+NAMEJDY_Hshopvn");
  allOK &= jdySendAT("Set NAME2",    "AT+NAMEJDY_Hshopvn_BLE");
  allOK &= jdySendAT("Read NAME",    "AT+NAME");
  allOK &= jdySendAT("Set BAUD4",    "AT+BAUD4");

  // Màn hình tổng kết
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("JDY-33 DONE");

  lcd.setCursor(0, 1);
  if (allOK) {
    lcd.print("Tat ca lenh OK   ");
  } else {
    lcd.print("Co lenh bi FAIL  ");
  }

  lcd.setCursor(0, 2);
  lcd.print("Module san sang   ");

  lcd.setCursor(0, 3);
  lcd.print("Nhan nut de Back ");

  // Đợi nhấn Encoder để thoát
  bool last = digitalRead(ENCODER_SW_PIN);
  while (true) {
    bool cur = digitalRead(ENCODER_SW_PIN);
    if (last == HIGH && cur == LOW) {
      delay(50);
      while (digitalRead(ENCODER_SW_PIN) == LOW) {
        delay(10);
      }
      break;
    }
    last = cur;
    delay(10);
  }

  Serial2.end();
}
