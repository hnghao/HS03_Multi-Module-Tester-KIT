#pragma once
#include <Preferences.h>

// Dùng Serial1 cho HC-05 (UART1 của ESP32-S3)
#define HC05_RX_PIN   2   // ESP32-S3 RX (nhận từ TX bluetooth)
#define HC05_TX_PIN   1   // ESP32-S3 TX (gửi tới RX bluetooth)
#define HC05          Serial1

// Thời gian chờ ổn định module
const unsigned long HC05_STARTUP_DELAY_MS = 5000;

// ----- Biến toàn cục riêng cho HC-05 -----
static Preferences hc05Prefs;
static uint8_t hc05NameIndex = 1;   // Số đếm đặt tên: 01..99

// Encoder riêng cho menu HC-05
static long hc05EncPosition = 0;
static int  hc05LastClkState = 0;

// =========================
// TIỆN ÍCH LCD (cuộn text)
// =========================
static void hc05LcdScrollText(uint8_t row, const String &text,
                              uint8_t repeat = 2, uint16_t stepDelay = 300) {
  if (text.length() <= 20) {
    lcdPrintLine(row, text.c_str());
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


// =========================
// ENCODER HC-05 MENU
// =========================
static void hc05EncoderInit() {
  hc05LastClkState = digitalRead(ENCODER_CLK_PIN);
  hc05EncPosition  = 0;
}

static void hc05EncoderUpdate() {
  int clkState = digitalRead(ENCODER_CLK_PIN);

  if (clkState != hc05LastClkState) {
    int dtState = digitalRead(ENCODER_DT_PIN);

    // Chỉ tính khi cạnh lên để mượt
    if (clkState == HIGH) {
      if (dtState != clkState) {
        hc05EncPosition++;   // quay 1 chiều
      } else {
        hc05EncPosition--;   // quay chiều ngược lại
      }
    }

    hc05LastClkState = clkState;
  }
}

// =========================
// HÀM AT CƠ BẢN
// =========================
static String hc05SendATRaw(const char* cmd, unsigned long timeoutMs) {
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
static String hc05ParseNameFromResponse(String resp) {
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

// =========================
// 1) KIỂM TRA LỆNH AT
// =========================
static bool hc05CheckAT_OK() {
  lcd.clear();
  lcdPrintLine(0, "Kiem tra lenh AT");
  lcdPrintLine(1, "Gui: AT");

  String resp = hc05SendATRaw("AT", 1500);

  String clean = resp;
  clean.replace('\r', ' ');
  clean.replace('\n', ' ');
  clean.trim();

  lcdPrintLine(2, "Phan hoi:");
  if (clean.length() == 0) {
    lcdPrintLine(3, "(khong co phan hoi)");
  } else {
    hc05LcdScrollText(3, clean, 1, 300);
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
      delay(1000); // Đứng luôn nếu lỗi AT (giữ đúng logic gốc)
    }
  }
}

// =========================
// 2) ĐỌC BAUDRATE AT+UART?
// =========================
static void hc05ReadBaudAndShow() {
  lcd.clear();
  lcdPrintLine(0, "Doc baud UART...");
  lcdPrintLine(1, "Gui: AT+UART?");

  String resp = hc05SendATRaw("AT+UART?", 1500);

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
  lcdPrintLine(3, baud.c_str());  // ví dụ: 38400
  delay(2000);
}

// =========================
// 3) ĐỌC TÊN HIỆN TẠI AT+NAME?
// =========================
static String hc05ReadNameAndShow(const char* title) {
  lcd.clear();
  lcdPrintLine(0, title);
  lcdPrintLine(1, "Gui: AT+NAME?");

  String resp = hc05SendATRaw("AT+NAME?", 1500);
  String name = hc05ParseNameFromResponse(resp);

  lcdPrintLine(2, "Ten hien tai:");

  if (name.length() == 0) {
    lcdPrintLine(3, "(khong co ten)");
  } else {
    hc05LcdScrollText(3, name, 2, 300);
  }

  delay(1000);
  return name;
}

// =========================
// 4) ĐỔI TÊN MODULE HC-05
// =========================
static String hc05RenameModuleWithCounter() {
  lcd.clear();
  lcdPrintLine(0, "Doi ten Bluetooth");

  // Giới hạn nameIndex 1..99
  if (hc05NameIndex < 1 || hc05NameIndex > 99) hc05NameIndex = 1;

  char numStr[4];
  snprintf(numStr, sizeof(numStr), "%02u", hc05NameIndex);

  String newName = "HC05_HShopvn_BLE_";
  newName += numStr;

  String cmd = "AT+NAME=";
  cmd += newName;

  lcdPrintLine(1, "Ten moi:");
  lcdPrintLine(2, newName.c_str());

  String resp = hc05SendATRaw(cmd.c_str(), 3000);

  String clean = resp;
  clean.replace('\r', ' ');
  clean.replace('\n', ' ');
  clean.trim();

  if (clean.length() == 0) {
    lcdPrintLine(3, "Khong co phan hoi");
  } else {
    if (clean.length() <= 20) {
      lcdPrintLine(3, clean.c_str());
    } else {
      hc05LcdScrollText(3, clean, 1, 300);
    }
  }

  delay(1500);

  // Tăng và lưu số đếm cho lần sau, giới hạn 99 -> quay về 01
  uint8_t nextIndex = hc05NameIndex + 1;
  if (nextIndex > 99) nextIndex = 1;
  hc05NameIndex = nextIndex;
  hc05Prefs.putUChar("nameIndex", hc05NameIndex);

  // Reset module để áp dụng tên mới cho quảng cáo Bluetooth
  lcd.clear();
  lcdPrintLine(0, "Reset module BT");
  lcdPrintLine(1, "Gui: AT+RESET");

  resp = hc05SendATRaw("AT+RESET", 2000);
  clean = resp;
  clean.replace('\r', ' ');
  clean.replace('\n', ' ');
  clean.trim();

  if (clean.length() == 0) {
    lcdPrintLine(2, "(khong co phan hoi)");
  } else {
    if (clean.length() <= 20) {
      lcdPrintLine(2, clean.c_str());
    } else {
      hc05LcdScrollText(2, clean, 1, 300);
    }
  }

  lcdPrintLine(3, "Doi khoi dong...");
  delay(HC05_STARTUP_DELAY_MS);  // chờ HC-05 khởi động lại

  // Thử đọc lại tên mới từ module (nếu vẫn đang AT mode)
  hc05ReadNameAndShow("Ten sau khi doi");

  return newName;
}

// =========================
// 5) FACTORY RESET SỐ ĐẾM
// =========================
static void hc05FactoryResetCounter() {
  lcd.clear();
  lcdPrintLine(0, "Factory Reset");
  lcdPrintLine(1, "Reset so dem...");

  hc05NameIndex = 1;
  hc05Prefs.putUChar("nameIndex", hc05NameIndex);

  lcdPrintLine(2, "So dem ve: 01");
  lcdPrintLine(3, "San sang doi ten");
  delay(1500);
}

// =========================
// 6) MENU CHÍNH DÙNG ENCODER
// =========================
static void hc05DrawMenu(uint8_t item) {
  lcd.clear();
  lcdPrintLine(0, "Menu Bluetooth");

  String line1 = (item == 0 ? "> " : "  ");
  line1 += "Rename module";
  lcdPrintLine(1, line1.c_str());

  String line2 = (item == 1 ? "> " : "  ");
  line2 += "Factory Reset";
  lcdPrintLine(2, line2.c_str());

  char buf[21];
  snprintf(buf, sizeof(buf), "Index: %02u", hc05NameIndex);
  lcdPrintLine(3, buf);   // buf là char[], truyền trực tiếp được
}


static void hc05RunMainMenu() {
  uint8_t menuItem = 0; // 0 = Rename, 1 = Factory Reset
  long lastEncPos = 0;

  hc05EncoderInit();
  hc05DrawMenu(menuItem);

  bool lastSw = digitalRead(ENCODER_SW_PIN);

  while (true) {
    // Cập nhật encoder mượt
    hc05EncoderUpdate();
    long pos = hc05EncPosition;

    if (pos > lastEncPos) {
      // quay 1 nấc -> chuyển mục xuống
      menuItem = (menuItem + 1) % 2;
      lastEncPos = pos;
      hc05DrawMenu(menuItem);
    } else if (pos < lastEncPos) {
      // quay ngược lại
      menuItem = (menuItem + 1 + 2) % 2;
      lastEncPos = pos;
      hc05DrawMenu(menuItem);
    }

    // Đọc nút nhấn encoder (SW) với debounce nhẹ
    bool sw = digitalRead(ENCODER_SW_PIN);
    if (sw != lastSw) {
      delay(5);
      sw = digitalRead(ENCODER_SW_PIN);
      if (sw != lastSw) {
        lastSw = sw;
        if (sw == LOW) {
          // Nhấn nút -> thực hiện mục đang chọn
          if (menuItem == 0) {
            // Rename
            hc05RenameModuleWithCounter();
          } else {
            // Factory Reset
            hc05FactoryResetCounter();
          }
          // Sau khi thực hiện xong, vẽ lại menu với index mới
          hc05DrawMenu(menuItem);
        }
      }
    }

    delay(2); // tránh vòng lặp quá gắt, vẫn rất mượt
  }
}

// =========================
// HÀM PUBLIC: BẮT ĐẦU MODE HC-05
// =========================
static void startBluetoothHC05Mode() {
  // Màn hình chào giống chương trình gốc
  lcd.clear();
  lcdPrintLine(0, "ESP32-S3 <-> BT");
  lcdPrintLine(1, "HC-05 AT Config");
  lcdPrintLine(2, "Khoi dong...");
  lcdPrintLine(3, " ");
  delay(1000);

  // Preferences: lưu số đếm tên
  hc05Prefs.begin("BT_CFG", false);
  hc05NameIndex = hc05Prefs.getUChar("nameIndex", 1);
  if (hc05NameIndex < 1 || hc05NameIndex > 99) hc05NameIndex = 1;

  // UART bluetooth (AT mode thường 38400)
  HC05.begin(38400, SERIAL_8N1, HC05_RX_PIN, HC05_TX_PIN);

  lcdPrintLine(2, "Doi on dinh 5s...");
  unsigned long startWait = millis();
  while (millis() - startWait < HC05_STARTUP_DELAY_MS) {
    delay(100);
  }

  // 1) Kiểm tra AT
  hc05CheckAT_OK();

  // 2) Đọc baud
  hc05ReadBaudAndShow();

  // 3) Đọc tên hiện tại & cuộn nếu dài
  hc05ReadNameAndShow("Doc ten hien tai");

  // 4) Vào menu chính dùng encoder
  hc05RunMainMenu();

  // (hàm này thực tế sẽ không thoát, đúng kiểu logic chương trình gốc)
}
