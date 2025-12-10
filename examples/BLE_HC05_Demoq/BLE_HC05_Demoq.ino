#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Preferences.h>

// ------------------- LCD2004 I2C -------------------
#define I2C_SDA_PIN   6
#define I2C_SCL_PIN   7
#define LCD_ADDR      0x27

LiquidCrystal_I2C lcd(LCD_ADDR, 20, 4);

// ------------------- HC-05 UART --------------------
#define HC05_RX_PIN   2   // ESP32-S3 RX (nhận từ TX bluetooth)
#define HC05_TX_PIN   1   // ESP32-S3 TX (gửi tới RX bluetooth)

HardwareSerial HC05(1);

// ------------------- ENCODER -----------------------
#define ENC_SW_PIN    15  // Nút nhấn encoder (SW)
#define ENC_DT_PIN    16
#define ENC_CLK_PIN   17

volatile long encPosition = 0;
int lastClkState = 0;     // dùng đọc encoder mượt

// ------------------- NVS / Preferences -------------
Preferences prefs;
uint8_t nameIndex = 1;     // Số đếm đặt tên: 01..99

// ------------------- HẰNG SỐ -----------------------
const unsigned long STARTUP_DELAY_MS = 5000;   // chờ module ổn định
const unsigned long LONG_PRESS_MS    = 5000;   // (hiện tại không dùng nữa, nhưng để sẵn)

// ===================================================
// TIỆN ÍCH LCD
// ===================================================

void lcdPrintLine(uint8_t row, const String &text) {
  lcd.setCursor(0, row);
  for (uint8_t i = 0; i < 20; i++) lcd.print(' ');
  lcd.setCursor(0, row);
  if (text.length() > 20) {
    lcd.print(text.substring(0, 20));
  } else {
    lcd.print(text);
  }
}

// Cuộn text nếu dài hơn 20 ký tự, cuộn từ phải sang trái
void lcdScrollText(uint8_t row, const String &text,
                   uint8_t repeat = 2, uint16_t stepDelay = 300) {
  if (text.length() <= 20) {
    lcdPrintLine(row, text);
    return;
  }

  String padded = text + "   "; // thêm khoảng trắng để cuộn mềm hơn
  uint8_t width = 20;
  int maxOffset = padded.length() - width;

  for (uint8_t r = 0; r < repeat; r++) {
    for (int offset = 0; offset <= maxOffset; offset++) {
      lcd.setCursor(0, row);
      for (uint8_t i = 0; i < width; i++) {
        char c = padded[offset + i];
        lcd.print(c);
      }
      delay(stepDelay);
    }
  }
}

// ===================================================
// ENCODER - ĐỌC MƯỢT
// ===================================================

void encoderInit() {
  lastClkState = digitalRead(ENC_CLK_PIN);
  encPosition = 0;
}

void encoderUpdate() {
  int clkState = digitalRead(ENC_CLK_PIN);

  if (clkState != lastClkState) {
    int dtState = digitalRead(ENC_DT_PIN);

    // Chỉ tính khi cạnh lên để mượt
    if (clkState == HIGH) {
      if (dtState != clkState) {
        encPosition++;   // quay 1 chiều
      } else {
        encPosition--;   // quay chiều ngược lại
      }
    }

    lastClkState = clkState;
  }
}

// ===================================================
// HÀM AT CƠ BẢN
// ===================================================

String sendATRaw(const char* cmd, unsigned long timeoutMs) {
  // Xóa buffer cũ nếu còn
  while (HC05.available()) {
    HC05.read();
  }

  HC05.print(cmd);
  HC05.print("\r\n");

  unsigned long start = millis();
  String response = "";

  while (millis() - start < timeoutMs) {
    while (HC05.available()) {
      char c = HC05.read();
      response += c;
    }
  }

  return response;
}

// Parse tên từ phản hồi AT+NAME?
String parseNameFromResponse(String resp) {
  resp.replace('\r', ' ');
  resp.replace('\n', ' ');
  resp.trim();
  if (resp.length() == 0) return "";

  int idxColon = resp.indexOf(':');
  int idxEq    = resp.indexOf('=');
  int idx      = -1;

  if (idxColon >= 0) idx = idxColon;
  else if (idxEq >= 0) idx = idxEq;

  int start = 0;
  if (idx >= 0 && idx + 1 < (int)resp.length()) {
    start = idx + 1;
  }

  int end = resp.length();
  int idxOK    = resp.indexOf("OK", start);
  int idxSpace = resp.indexOf(' ', start);

  if (idxOK >= 0 && idxOK < end)       end = idxOK;
  if (idxSpace >= 0 && idxSpace < end) end = idxSpace;

  String name = resp.substring(start, end);
  name.trim();
  return name;
}

// ===================================================
// 1) KIỂM TRA LỆNH AT
// ===================================================

bool checkAT_OK() {
  lcd.clear();
  lcdPrintLine(0, "Kiem tra lenh AT");
  lcdPrintLine(1, "Gui: AT");

  String resp = sendATRaw("AT", 1500);

  String clean = resp;
  clean.replace('\r', ' ');
  clean.replace('\n', ' ');
  clean.trim();

  lcdPrintLine(2, "Phan hoi:");
  if (clean.length() == 0) {
    lcdPrintLine(3, "(khong co phan hoi)");
  } else {
    lcdScrollText(3, clean, 1, 300);
  }

  delay(1000);

  if (resp.indexOf("OK") != -1 || resp.indexOf("ok") != -1) {
    lcd.clear();
    lcdPrintLine(0, "AT: OK");
    lcdPrintLine(1, "Tiep tuc...");
    delay(1000);
    return true;
  } else {
    lcd.clear();
    lcdPrintLine(0, "LOI: AT khong OK");
    lcdPrintLine(1, "Kiem tra ket noi");
    while (true) {
      delay(1000); // Đứng luôn
    }
  }
}

// ===================================================
// 2) ĐỌC BAUDRATE BẰNG AT+UART?
// ===================================================

void readBaudAndShow() {
  lcd.clear();
  lcdPrintLine(0, "Doc baud UART...");
  lcdPrintLine(1, "Gui: AT+UART?");

  String resp = sendATRaw("AT+UART?", 1500);

  String clean = resp;
  clean.replace('\r', ' ');
  clean.replace('\n', ' ');
  clean.trim();

  String baud = "N/A";

  int idx = clean.indexOf(':');
  if (idx >= 0 && idx + 1 < (int)clean.length()) {
    int start = idx + 1;
    int end = start;
    while (end < (int)clean.length() &&
           clean[end] >= '0' && clean[end] <= '9') {
      end++;
    }
    if (end > start) {
      baud = clean.substring(start, end);
    }
  }

  lcdPrintLine(2, "Baud hien tai:");
  lcdPrintLine(3, baud);  // ví dụ: 38400
  delay(2000);
}

// ===================================================
// 3) ĐỌC TÊN HIỆN TẠI BẰNG AT+NAME? (HIỂN THỊ CUỘN)
// ===================================================

String readNameAndShow(const char* title) {
  lcd.clear();
  lcdPrintLine(0, title);
  lcdPrintLine(1, "Gui: AT+NAME?");

  String resp = sendATRaw("AT+NAME?", 1500);
  String name = parseNameFromResponse(resp);

  lcdPrintLine(2, "Ten hien tai:");

  if (name.length() == 0) {
    lcdPrintLine(3, "(khong co ten)");
  } else {
    lcdScrollText(3, name, 2, 300);
  }

  delay(1000);
  return name;
}

// ===================================================
// 4) ĐỔI TÊN VỚI SỐ ĐẾM 01..99, TỰ QUAY VỀ 01
// ===================================================

String renameModuleWithCounter() {
  lcd.clear();
  lcdPrintLine(0, "Doi ten Bluetooth");

  // Giới hạn nameIndex 1..99
  if (nameIndex < 1 || nameIndex > 99) nameIndex = 1;

  char numStr[4];
  snprintf(numStr, sizeof(numStr), "%02u", nameIndex);

  String newName = "HC05_HShopvn_BLE_";
  newName += numStr;

  String cmd = "AT+NAME=";
  cmd += newName;

  lcdPrintLine(1, "Ten moi:");
  lcdPrintLine(2, newName);

  String resp = sendATRaw(cmd.c_str(), 3000);

  String clean = resp;
  clean.replace('\r', ' ');
  clean.replace('\n', ' ');
  clean.trim();

  if (clean.length() == 0) {
    lcdPrintLine(3, "Khong co phan hoi");
  } else {
    if (clean.length() <= 20) {
      lcdPrintLine(3, clean);
    } else {
      lcdScrollText(3, clean, 1, 300);
    }
  }

  delay(1500);

  // Tăng và lưu số đếm cho lần sau, giới hạn 99 -> quay về 01
  uint8_t nextIndex = nameIndex + 1;
  if (nextIndex > 99) nextIndex = 1;
  nameIndex = nextIndex;
  prefs.putUChar("nameIndex", nameIndex);

  // Reset module để áp dụng tên mới cho quảng cáo Bluetooth
  lcd.clear();
  lcdPrintLine(0, "Reset module BT");
  lcdPrintLine(1, "Gui: AT+RESET");

  resp = sendATRaw("AT+RESET", 2000);
  clean = resp;
  clean.replace('\r', ' ');
  clean.replace('\n', ' ');
  clean.trim();

  if (clean.length() == 0) {
    lcdPrintLine(2, "(khong co phan hoi)");
  } else {
    if (clean.length() <= 20) {
      lcdPrintLine(2, clean);
    } else {
      lcdScrollText(2, clean, 1, 300);
    }
  }

  lcdPrintLine(3, "Doi khoi dong...");
  delay(STARTUP_DELAY_MS);  // chờ HC-05 khởi động lại

  // Thử đọc lại tên mới từ module (nếu vẫn đang AT mode)
  readNameAndShow("Ten sau khi doi");

  return newName;
}

// ===================================================
// 5) FACTORY RESET: RESET SỐ ĐẾM VỀ 01
// ===================================================

void factoryResetCounter() {
  lcd.clear();
  lcdPrintLine(0, "Factory Reset");
  lcdPrintLine(1, "Reset so dem...");

  nameIndex = 1;
  prefs.putUChar("nameIndex", nameIndex);

  lcdPrintLine(2, "So dem ve: 01");
  lcdPrintLine(3, "San sang doi ten");
  delay(1500);
}

// ===================================================
// 6) MENU CHÍNH DÙNG ENCODER: RENAME / FACTORY RESET
// ===================================================

void drawMenu(uint8_t item) {
  lcd.clear();
  lcdPrintLine(0, "Menu Bluetooth");

  String line1 = (item == 0 ? "> " : "  ");
  line1 += "Rename module";
  lcdPrintLine(1, line1);

  String line2 = (item == 1 ? "> " : "  ");
  line2 += "Factory Reset";
  lcdPrintLine(2, line2);

  char buf[21];
  snprintf(buf, sizeof(buf), "Index: %02u", nameIndex);
  lcdPrintLine(3, String(buf));
}

void runMainMenu() {
  uint8_t menuItem = 0; // 0 = Rename, 1 = Factory Reset
  long lastEncPos = 0;

  encoderInit();
  drawMenu(menuItem);

  bool lastSw = digitalRead(ENC_SW_PIN);

  while (true) {
    // Cập nhật encoder mượt
    encoderUpdate();
    long pos = encPosition;

    if (pos > lastEncPos) {
      // quay 1 nấc -> chuyển mục xuống
      menuItem = (menuItem + 1) % 2;
      lastEncPos = pos;
      drawMenu(menuItem);
    } else if (pos < lastEncPos) {
      // quay ngược lại
      menuItem = (menuItem + 1 + 2) % 2;
      lastEncPos = pos;
      drawMenu(menuItem);
    }

    // Đọc nút nhấn encoder (SW) với debounce nhẹ
    bool sw = digitalRead(ENC_SW_PIN);
    if (sw != lastSw) {
      delay(5);
      sw = digitalRead(ENC_SW_PIN);
      if (sw != lastSw) {
        lastSw = sw;
        if (sw == LOW) {
          // Nhấn nút -> thực hiện mục đang chọn
          if (menuItem == 0) {
            // Rename
            renameModuleWithCounter();
          } else {
            // Factory Reset
            factoryResetCounter();
          }
          // Sau khi thực hiện xong, vẽ lại menu với index mới
          drawMenu(menuItem);
        }
      }
    }

    delay(2); // tránh vòng lặp quá gắt, vẫn rất mượt
  }
}

// ===================================================
// SETUP
// ===================================================

void setup() {
  // LCD
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  lcd.init();
  lcd.backlight();

  lcd.clear();
  lcdPrintLine(0, "ESP32-S3 <-> BT");
  lcdPrintLine(1, "HC-05 AT Config");
  lcdPrintLine(2, "Khoi dong...");
  delay(1000);

  // Encoder
  pinMode(ENC_SW_PIN, INPUT_PULLUP);
  pinMode(ENC_DT_PIN, INPUT_PULLUP);
  pinMode(ENC_CLK_PIN, INPUT_PULLUP);

  // Preferences: lưu số đếm tên
  prefs.begin("BT_CFG", false);
  nameIndex = prefs.getUChar("nameIndex", 1);
  if (nameIndex < 1 || nameIndex > 99) nameIndex = 1;

  // UART bluetooth (AT mode thường 38400)
  HC05.begin(38400, SERIAL_8N1, HC05_RX_PIN, HC05_TX_PIN);

  lcdPrintLine(2, "Doi on dinh 5s...");
  unsigned long startWait = millis();
  while (millis() - startWait < STARTUP_DELAY_MS) {
    delay(100);
  }

  // 1) Kiểm tra AT
  checkAT_OK();

  // 2) Đọc baud
  readBaudAndShow();

  // 3) Đọc tên hiện tại & cuộn nếu dài
  readNameAndShow("Doc ten hien tai");

  // 4) Vào menu chính dùng encoder
  runMainMenu();
}

// ===================================================
// LOOP
// ===================================================

void loop() {
  // Không dùng, vì runMainMenu() đã loop vô hạn
}
