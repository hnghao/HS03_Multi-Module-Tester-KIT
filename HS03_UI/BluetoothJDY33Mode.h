#ifndef BLUETOOTH_JDY33_MODE_H
#define BLUETOOTH_JDY33_MODE_H

#include <Arduino.h>
#include <Preferences.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// Dùng lại LCD & I2C bus từ chương trình chính
extern LiquidCrystal_I2C lcd;
extern TwoWire I2CScanBus;

// Dùng lại chân encoder của HS03
// ENCODER_SW_PIN, ENCODER_DT_PIN, ENCODER_CLK_PIN đã được define trong HS03_UI.ino

// Dùng lại chân I2C Scan (1,2) cho UART JDY-33
// (đã dùng ổn trong chương trình trước)
#define JDY_TX_PIN    I2C_SCAN_SDA_PIN   // TX ESP32-S3 -> RX JDY-33 (GPIO1)
#define JDY_RX_PIN    I2C_SCAN_SCL_PIN   // RX ESP32-S3 <- TX JDY-33 (GPIO2)

#define JDY_BAUD      9600
#define AT_TIMEOUT    1000  // ms

// LCD 20 cột, 4 dòng (LCD2004)
#define LCD_COLS      20

// Vị trí cuộn tên
static const uint8_t JDY_NAME_ROW    = 1;               // dòng "Name:"
static const uint8_t JDY_NAME_COL    = 6;               // sau chữ "Name:"
static const uint8_t JDY_NAME_WINDOW = LCD_COLS - JDY_NAME_COL; // 14 ký tự

// ================== Trạng thái riêng cho JDY ==================
enum JDYAppState {
  JDY_APP_ERROR,
  JDY_APP_MENU,
  JDY_APP_RENAMING,
  JDY_APP_FACTORY_RESET
};

static JDYAppState jdy_appState = JDY_APP_ERROR;

// NVS (Preferences) để lưu bộ đếm tên
static Preferences jdy_prefs;
static uint16_t jdy_nameIndex = 0;
static long     jdy_baudVal   = 0;
static String   jdy_currentName;

// Menu Rename / Factory reset
static int  jdy_menuIndex     = 0;
static const int JDY_MENU_ITEMS = 2;

// Nút nhấn encoder
static bool         jdy_lastSwState   = HIGH;
static unsigned long jdy_swPressStart = 0;

// Giải mã encoder quadrature mượt
static uint8_t jdy_encPrevAB = 0;
static int8_t  jdy_encAccum  = 0;
static const int8_t jdy_encTable[16] = {
  0, -1, +1, 0,
  +1, 0,  0, -1,
  -1, 0,  0, +1,
  0, +1, -1, 0
};

// Cuộn tên
static String  jdy_scrollName;
static uint8_t jdy_scrollPos   = 0;
static bool    jdy_needScroll  = false;
static unsigned long jdy_lastScroll = 0;
static const unsigned long JDY_SCROLL_INTERVAL = 300; // ms

// ================== Khai báo hàm nội bộ ==================
static String jdy_sendATGetResponse(const char *cmd, uint16_t timeoutMs = AT_TIMEOUT);
static long   jdy_baudCodeToValue(int code);
static int    jdy_parseBaudCode(const String &resp);
static String jdy_extractNameFromResp(const String &resp);

static void   jdy_setupNameScroll(const String &name);
static void   jdy_updateNameDisplay();
static void   jdy_handleNameScroll();
static void   jdy_showMenuScreen();
static void   jdy_startRenameSequence();
static void   jdy_factoryResetCounter();
static void   jdy_handleEncoderRotation();
static void   jdy_handleEncoderButton(bool &exitRequested);

// ================== Gửi AT & đọc phản hồi ==================
static String jdy_sendATGetResponse(const char *cmd, uint16_t timeoutMs) {
  // Xoá buffer cũ
  while (Serial2.available()) Serial2.read();

  Serial2.print(cmd);
  Serial2.print("\r\n");

  String resp;
  unsigned long start = millis();
  while (millis() - start < timeoutMs) {
    while (Serial2.available()) {
      char c = (char)Serial2.read();
      resp += c;
    }
  }
  return resp;
}

// ================== Xử lý BAUD ==================
static long jdy_baudCodeToValue(int code) {
  switch (code) {
    case 2: return 2400;
    case 3: return 4800;
    case 4: return 9600;
    case 5: return 19200;
    case 6: return 38400;
    case 7: return 57600;
    case 8: return 115200;
    case 9: return 128000;
    default: return 0;
  }
}

static int jdy_parseBaudCode(const String &resp) {
  int idx = resp.indexOf('=');
  if (idx >= 0 && idx + 1 < (int)resp.length()) {
    char c = resp.charAt(idx + 1);
    if (c >= '0' && c <= '9') return c - '0';
  }

  for (int i = resp.length() - 1; i >= 0; i--) {
    char c = resp.charAt(i);
    if (c >= '0' && c <= '9') return c - '0';
  }
  return -1;
}

// ================== Tách NAME từ phản hồi ==================
static String jdy_extractNameFromResp(const String &respIn) {
  String s = respIn;
  s.trim();
  s.replace('\r', ' ');
  s.replace('\n', ' ');

  int eqPos = s.indexOf('=');
  if (eqPos >= 0 && eqPos + 1 < (int)s.length()) {
    s = s.substring(eqPos + 1);
  }
  s.trim();
  return s;
}

// ================== Cuộn tên ==================
static void jdy_setupNameScroll(const String &name) {
  jdy_scrollName = name;
  jdy_scrollPos  = 0;
  jdy_needScroll = (jdy_scrollName.length() > JDY_NAME_WINDOW);
  jdy_lastScroll = millis();
}

static void jdy_updateNameDisplay() {
  lcd.setCursor(0, JDY_NAME_ROW);
  lcd.print("Name:");

  String window;
  if (!jdy_needScroll) {
    window = jdy_scrollName;
  } else {
    if (jdy_scrollPos + JDY_NAME_WINDOW <= jdy_scrollName.length()) {
      window = jdy_scrollName.substring(jdy_scrollPos, jdy_scrollPos + JDY_NAME_WINDOW);
    } else {
      window = jdy_scrollName.substring(jdy_scrollPos);
      while (window.length() < JDY_NAME_WINDOW) window += " ";
    }
  }

  while (window.length() < JDY_NAME_WINDOW) window += " ";

  lcd.setCursor(JDY_NAME_COL, JDY_NAME_ROW);
  lcd.print(window);
}

static void jdy_handleNameScroll() {
  if (!jdy_needScroll) return;

  unsigned long now = millis();
  if (now - jdy_lastScroll >= JDY_SCROLL_INTERVAL) {
    jdy_lastScroll = now;
    jdy_scrollPos++;
    if (jdy_scrollPos > jdy_scrollName.length()) jdy_scrollPos = 0;
    jdy_updateNameDisplay();
  }
}

// ================== Hiển thị menu JDY ==================
static void jdy_showMenuScreen() {
  lcd.clear();

  // Dòng 0: Baud
  lcd.setCursor(0, 0);
  if (jdy_baudVal != 0) {
    lcd.print("Baud: ");
    lcd.print(jdy_baudVal);
    lcd.print("    ");
  } else {
    lcd.print("Baud: ?           ");
  }

  // Dòng 1: Name (cuộn)
  jdy_updateNameDisplay();

  // Dòng 2 & 3: Menu
  lcd.setCursor(0, 2);
  lcd.print(jdy_menuIndex == 0 ? ">" : " ");
  lcd.print(" Rename          ");

  lcd.setCursor(0, 3);
  lcd.print(jdy_menuIndex == 1 ? ">" : " ");
  lcd.print(" Factory reset   ");
}

// ================== Encoder xoay (mượt từng nấc) ==================
static void jdy_handleEncoderRotation() {
  // Chỉ xử lý khi đang ở menu JDY
  if (jdy_appState != JDY_APP_MENU) return;

  uint8_t A = digitalRead(ENCODER_CLK_PIN);
  uint8_t B = digitalRead(ENCODER_DT_PIN);
  uint8_t AB = (A << 1) | B;

  uint8_t index = (jdy_encPrevAB << 2) | AB;
  jdy_encPrevAB = AB;

  int8_t step = jdy_encTable[index & 0x0F];
  if (step == 0) return;

  jdy_encAccum += step;
  if (jdy_encAccum >= 4) {
    jdy_encAccum = 0;
    jdy_menuIndex++;
  } else if (jdy_encAccum <= -4) {
    jdy_encAccum = 0;
    jdy_menuIndex--;
  } else {
    return;
  }

  if (jdy_menuIndex < 0)                jdy_menuIndex = JDY_MENU_ITEMS - 1;
  if (jdy_menuIndex >= JDY_MENU_ITEMS)  jdy_menuIndex = 0;

  lcd.setCursor(0, 2);
  lcd.print(jdy_menuIndex == 0 ? ">" : " ");
  lcd.print(" Rename          ");

  lcd.setCursor(0, 3);
  lcd.print(jdy_menuIndex == 1 ? ">" : " ");
  lcd.print(" Factory reset   ");
}

// ================== Encoder nhấn (short = chọn, long = THOÁT) ==================
static void jdy_handleEncoderButton(bool &exitRequested) {
  bool sw  = digitalRead(ENCODER_SW_PIN);
  unsigned long now = millis();

  // Falling edge: bắt đầu đo thời gian nhấn
  if (sw == LOW && jdy_lastSwState == HIGH) {
    jdy_swPressStart = now;
  }

  // Rising edge: kết thúc nhấn
  if (sw == HIGH && jdy_lastSwState == LOW) {
    unsigned long dt = 0;

    // Chỉ tính thời gian nếu đã có falling edge hợp lệ
    if (jdy_swPressStart > 0) {
      dt = now - jdy_swPressStart;
    }

    // Reset để lần nhấn sau không bị dính thời gian cũ
    jdy_swPressStart = 0;

    // Nếu dt == 0 nghĩa là nút đã bị giữ từ trước khi vào mode JDY,
    // lần nhả này sẽ bị bỏ qua để tránh tự thoát.
    if (dt > 0) {
      // Nhấn NGẮN: 50ms–1500ms -> xử lý menu Rename / Factory reset
      if (dt > 50 && dt < 1500) {
        if (jdy_appState == JDY_APP_MENU) {
          if (jdy_menuIndex == 0) {
            jdy_startRenameSequence();
          } else if (jdy_menuIndex == 1) {
            jdy_factoryResetCounter();
          }
        }
      }
      // Nhấn DÀI: >=1500ms -> yêu cầu thoát JDY-33 về menu Bluetooth
      else if (dt >= 1500) {
        exitRequested = true;
      }
    }
  }

  jdy_lastSwState = sw;
}


// ================== Đổi tên JDY-33 ==================
static void jdy_startRenameSequence() {
  jdy_appState = JDY_APP_RENAMING;

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Renaming JDY...   ");

  uint16_t newIndex = jdy_nameIndex + 1;
  if (newIndex > 99 || newIndex == 0) {
    newIndex = 1;
  }

  char suffix[3];
  snprintf(suffix, sizeof(suffix), "%02u", newIndex);

  String newName = String("JDY_Hshopvn_BLE_") + suffix;

  lcd.setCursor(0, 1);
  lcd.print("Set: ");
  String shortName = newName;
  if (shortName.length() > 15) shortName = shortName.substring(0, 15);
  lcd.print(shortName);

  // Đổi tên SPP
  String cmdName  = String("AT+NAME") + newName;
  String respName = jdy_sendATGetResponse(cmdName.c_str());
  respName.trim();
  bool okName = (respName.indexOf("OK") >= 0);

  // Đổi tên BLE
  String cmdNamb  = String("AT+NAMB") + newName;
  String respNamb = jdy_sendATGetResponse(cmdNamb.c_str());
  respNamb.trim();
  bool okNamb = (respNamb.indexOf("OK") >= 0);

  bool ok = okName || okNamb;

  lcd.setCursor(0, 2);
  if (ok) {
    lcd.print("Result: OK        ");

    jdy_nameIndex = newIndex;
    jdy_prefs.putUShort("name_idx", jdy_nameIndex);

    jdy_sendATGetResponse("AT+RESET", 300);
    delay(1500);

    String nameResp = jdy_sendATGetResponse("AT+NAME");
    jdy_currentName = jdy_extractNameFromResp(nameResp);
    jdy_setupNameScroll(jdy_currentName);

    lcd.setCursor(0, 3);
    lcd.print("Rescan BT on phone");
  } else {
    lcd.print("Result: FAIL      ");
    lcd.setCursor(0, 3);
    lcd.print("Check module      ");
  }

  delay(1500);

  jdy_showMenuScreen();
  jdy_appState = JDY_APP_MENU;
}

// ================== Factory reset bộ đếm tên ==================
static void jdy_factoryResetCounter() {
  jdy_appState = JDY_APP_FACTORY_RESET;

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Factory reset cnt ");

  jdy_nameIndex = 0;
  jdy_prefs.putUShort("name_idx", jdy_nameIndex);

  lcd.setCursor(0, 2);
  lcd.print("Idx->0 (next 01)  ");
  lcd.setCursor(0, 3);
  lcd.print("OK                ");

  delay(1500);

  jdy_showMenuScreen();
  jdy_appState = JDY_APP_MENU;
}

// ================== HÀM GỌI TỪ HS03: CHẠY JDY-33 (BLOCKING) ==================
inline void startBluetoothJDY33Mode() {
  // Tạm tắt I2CScanBus vì dùng chung GPIO1/2 cho UART2
  I2CScanBus.end();

  // Init encoder state riêng cho JDY
  uint8_t initA = digitalRead(ENCODER_CLK_PIN);
  uint8_t initB = digitalRead(ENCODER_DT_PIN);
  jdy_encPrevAB = (initA << 1) | initB;
  jdy_encAccum  = 0;

  jdy_lastSwState   = digitalRead(ENCODER_SW_PIN);
  jdy_swPressStart  = 0;
  jdy_menuIndex     = 0;
  jdy_appState      = JDY_APP_ERROR;
  jdy_scrollName    = "";
  jdy_scrollPos     = 0;
  jdy_needScroll    = false;

  // Màn hình khởi động JDY-33
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("JDY-33 START...   ");

  // NVS
  jdy_prefs.begin("jdycfg", false);
  jdy_nameIndex = jdy_prefs.getUShort("name_idx", 0);

  // UART2 cho JDY-33
  Serial2.begin(JDY_BAUD, SERIAL_8N1, JDY_RX_PIN, JDY_TX_PIN);
  delay(200);

  // --------- B1: TEST AT ---------
  lcd.setCursor(0, 1);
  lcd.print("Test AT command   ");

  String atResp = jdy_sendATGetResponse("AT");
  atResp.trim();
  bool atOK = atResp.length() > 0 && atResp.indexOf("OK") >= 0;

  lcd.setCursor(0, 2);
  if (atOK) {
    lcd.print("AT: OK            ");
  } else {
    lcd.print("AT: FAIL          ");
    lcd.setCursor(0, 3);
    lcd.print("Check wiring/mode ");

    // Nếu AT FAIL -> cho phép nhấn giữ để thoát
    while (true) {
      bool exitReq = false;
      jdy_handleEncoderButton(exitReq);
      if (exitReq) break;
      delay(10);
    }

    Serial2.end();
    jdy_prefs.end();
    I2CScanBus.begin(I2C_SCAN_SDA_PIN, I2C_SCAN_SCL_PIN, 100000);
    while (digitalRead(ENCODER_SW_PIN) == LOW) delay(10);
    return;
  }
  delay(800);

  // --------- B2: ĐỌC BAUD ---------
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Read BAUD...      ");

  String baudResp = jdy_sendATGetResponse("AT+BAUD");
  baudResp.trim();
  baudResp.replace('\r', ' ');
  baudResp.replace('\n', ' ');

  int baudCode = jdy_parseBaudCode(baudResp);
  jdy_baudVal  = jdy_baudCodeToValue(baudCode);

  lcd.setCursor(0, 1);
  if (jdy_baudVal != 0) {
    lcd.print("Baud: ");
    lcd.print(jdy_baudVal);
    lcd.print("    ");
  } else {
    lcd.print("Baud: unknown     ");
  }

  lcd.setCursor(0, 2);
  lcd.print("Resp:");
  if (baudResp.length() == 0) {
    lcd.print(" No data   ");
  } else {
    String tmp = baudResp;
    if (tmp.length() > 14) tmp = tmp.substring(0, 14);
    lcd.setCursor(6, 2);
    lcd.print("              ");
    lcd.setCursor(6, 2);
    lcd.print(tmp);
  }
  delay(1200);

  // --------- B3: ĐỌC NAME ---------
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Read NAME...      ");

  String nameResp = jdy_sendATGetResponse("AT+NAME");
  nameResp.trim();
  nameResp.replace('\r', ' ');
  nameResp.replace('\n', ' ');

  jdy_currentName = jdy_extractNameFromResp(nameResp);

  lcd.setCursor(0, 1);
  lcd.print("Name current:     ");
  lcd.setCursor(0, 2);
  if (jdy_currentName.length() == 0) {
    lcd.print("(none)           ");
  } else {
    String tmp = jdy_currentName;
    if (tmp.length() > 20) tmp = tmp.substring(0, 20);
    lcd.print(tmp);
  }
  delay(1200);

  // Chuẩn bị cuộn tên & hiển thị menu
  jdy_setupNameScroll(jdy_currentName);
  jdy_showMenuScreen();
  jdy_appState = JDY_APP_MENU;

  // --------- Vòng lặp riêng cho JDY-33 ---------
  while (true) {
    bool exitReq = false;

    jdy_handleEncoderRotation();
    jdy_handleEncoderButton(exitReq);
    jdy_handleNameScroll();

    if (exitReq) break;

    delay(5);  // tránh chiếm CPU 100%
  }

  // Dọn dẹp & trả lại tài nguyên
  Serial2.end();
  jdy_prefs.end();

  // Khởi động lại I2CScanBus cho ADS1115 / I2C Scan
  I2CScanBus.begin(I2C_SCAN_SDA_PIN, I2C_SCAN_SCL_PIN, 100000);

  // Đảm bảo nút đã nhả trước khi quay lại HS03
  while (digitalRead(ENCODER_SW_PIN) == LOW) {
    delay(10);
  }
}

#endif // BLUETOOTH_JDY33_MODE_H
