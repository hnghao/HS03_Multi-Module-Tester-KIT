#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Preferences.h>

// ================== LCD2004 I2C ==================
#define I2C_SDA_PIN   6
#define I2C_SCL_PIN   7
#define LCD_ADDR      0x27
#define LCD_COLS      20

LiquidCrystal_I2C lcd(LCD_ADDR, LCD_COLS, 4);

// ================== JDY-33 UART ==================
// CHỈNH LẠI CHO ĐÚNG VỚI MẠCH CỦA BẠN
#define JDY_TX_PIN    1     // TX ESP32-S3 -> RX JDY-33
#define JDY_RX_PIN    2     // RX ESP32-S3 <- TX JDY-33

#define JDY_BAUD      9600
#define AT_TIMEOUT    1000  // ms

// ================== ENCODER ==================
#define ENC_SW_PIN    15
#define ENC_DT_PIN    16
#define ENC_CLK_PIN   17

int  menuIndex     = 0;
const int MENU_ITEMS = 2;   // 0: Rename, 1: Factory reset

bool lastSwState   = HIGH;
unsigned long swPressStart = 0;

// Giải mã quadrature (CLK, DT) cho encoder
static uint8_t encPrevAB = 0;
static int8_t  encAccum  = 0;
// Bảng tra quadrature: 00->01->11->10->00 (CW), ngược lại là CCW
const int8_t encTable[16] = {
  0, -1, +1, 0,
  +1, 0,  0, -1,
  -1, 0,  0, +1,
  0, +1, -1, 0
};

// ================== STATE & EEPROM ==================
Preferences prefs;
uint16_t g_nameIndex = 0;   // bộ đếm tên (01..99)
long     g_baudVal   = 0;
String   currentName = "";

enum AppState {
  APP_ERROR,
  APP_MENU,
  APP_RENAMING,
  APP_FACTORY_RESET
};

AppState appState = APP_ERROR;

// ================== CUỘN TÊN ==================
const uint8_t NAME_ROW    = 1;       // dòng hiển thị tên
const uint8_t NAME_COL    = 6;       // sau "Name: "
const uint8_t NAME_WINDOW = LCD_COLS - NAME_COL;  // 14 ký tự

String  scrollName    = "";
uint8_t scrollPos     = 0;
bool    needScroll    = false;
unsigned long lastScroll = 0;
const unsigned long SCROLL_INTERVAL = 300; // ms

// ================== KHAI BÁO HÀM ==================
String sendATGetResponse(const char *cmd, uint16_t timeoutMs = AT_TIMEOUT);
long   baudCodeToValue(int code);
int    parseBaudCode(const String &resp);
String extractNameFromResp(const String &resp);

void   setupNameScroll(const String &name);
void   updateNameDisplay();
void   handleNameScroll();
void   handleEncoderRotation();
void   handleEncoderButton();
void   showMenuScreen();
void   startRenameSequence();
void   factoryResetCounter();

// ================== SETUP ==================
void setup() {
  // LCD
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("JDY-33 START...   ");

  // Encoder
  pinMode(ENC_SW_PIN,  INPUT_PULLUP);
  pinMode(ENC_DT_PIN,  INPUT_PULLUP);
  pinMode(ENC_CLK_PIN, INPUT_PULLUP);
  lastSwState = digitalRead(ENC_SW_PIN);

  // Khởi tạo trạng thái ban đầu của encoder
  uint8_t initA = digitalRead(ENC_CLK_PIN);
  uint8_t initB = digitalRead(ENC_DT_PIN);
  encPrevAB = (initA << 1) | initB;
  encAccum  = 0;

  // EEPROM (Preferences)
  prefs.begin("jdycfg", false);
  g_nameIndex = prefs.getUShort("name_idx", 0); // nếu chưa có thì =0

  // UART2 cho JDY-33
  Serial2.begin(JDY_BAUD, SERIAL_8N1, JDY_RX_PIN, JDY_TX_PIN);
  delay(200);

  // --------- B1: TEST AT ---------
  lcd.setCursor(0, 1);
  lcd.print("Test AT command   ");

  String atResp = sendATGetResponse("AT");
  atResp.trim();
  bool atOK = atResp.length() > 0 && atResp.indexOf("OK") >= 0;

  lcd.setCursor(0, 2);
  if (atOK) {
    lcd.print("AT: OK            ");
  } else {
    lcd.print("AT: FAIL          ");
    lcd.setCursor(0, 3);
    lcd.print("Check wiring/mode ");
    appState = APP_ERROR;
    while (1) {
      delay(1000);
    }
  }
  delay(800);

  // --------- B2: ĐỌC BAUD ---------
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Read BAUD...      ");

  String baudResp = sendATGetResponse("AT+BAUD");
  baudResp.trim();
  baudResp.replace('\r', ' ');
  baudResp.replace('\n', ' ');

  int baudCode = parseBaudCode(baudResp);
  g_baudVal    = baudCodeToValue(baudCode);

  lcd.setCursor(0, 1);
  if (g_baudVal != 0) {
    lcd.print("Baud: ");
    lcd.print(g_baudVal);
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

  String nameResp = sendATGetResponse("AT+NAME");
  nameResp.trim();
  nameResp.replace('\r', ' ');
  nameResp.replace('\n', ' ');

  currentName = extractNameFromResp(nameResp);

  lcd.setCursor(0, 1);
  lcd.print("Name current:     ");
  lcd.setCursor(0, 2);
  if (currentName.length() == 0) {
    lcd.print("(none)           ");
  } else {
    String tmp = currentName;
    if (tmp.length() > 20) tmp = tmp.substring(0, 20);
    lcd.print(tmp);
  }
  delay(1200);

  // Chuẩn bị cuộn tên & hiển thị menu
  setupNameScroll(currentName);
  showMenuScreen();
  appState = APP_MENU;
}

// ================== LOOP ==================
void loop() {
  if (appState == APP_ERROR) return;

  if (appState == APP_MENU) {
    handleEncoderRotation();
    handleEncoderButton();
  }

  handleNameScroll();
}

// ================== GỬI AT & NHẬN PHẢN HỒI ==================
String sendATGetResponse(const char *cmd, uint16_t timeoutMs) {
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

// ================== XỬ LÝ BAUD ==================
long baudCodeToValue(int code) {
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

int parseBaudCode(const String &resp) {
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

// ================== TÁCH NAME TỪ RESP ==================
String extractNameFromResp(const String &respIn) {
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

// ================== CUỘN TÊN ==================
void setupNameScroll(const String &name) {
  scrollName = name;
  scrollPos  = 0;
  needScroll = (scrollName.length() > NAME_WINDOW);
  lastScroll = millis();
}

void updateNameDisplay() {
  lcd.setCursor(0, NAME_ROW);
  lcd.print("Name:");

  String window;
  if (!needScroll) {
    window = scrollName;
  } else {
    if (scrollPos + NAME_WINDOW <= scrollName.length()) {
      window = scrollName.substring(scrollPos, scrollPos + NAME_WINDOW);
    } else {
      window = scrollName.substring(scrollPos);
      while (window.length() < NAME_WINDOW) window += " ";
    }
  }

  while (window.length() < NAME_WINDOW) window += " ";

  lcd.setCursor(NAME_COL, NAME_ROW);
  lcd.print(window);
}

void handleNameScroll() {
  if (!needScroll) return;

  unsigned long now = millis();
  if (now - lastScroll >= SCROLL_INTERVAL) {
    lastScroll = now;
    scrollPos++;
    if (scrollPos > scrollName.length()) scrollPos = 0;
    updateNameDisplay();
  }
}

// ================== HIỂN THỊ MENU ==================
void showMenuScreen() {
  lcd.clear();

  // Dòng 0: Baud
  lcd.setCursor(0, 0);
  if (g_baudVal != 0) {
    lcd.print("Baud: ");
    lcd.print(g_baudVal);
    lcd.print("    ");
  } else {
    lcd.print("Baud: ?           ");
  }

  // Dòng 1: Name (cuộn)
  updateNameDisplay();

  // Dòng 2 & 3: Menu
  lcd.setCursor(0, 2);
  lcd.print(menuIndex == 0 ? ">" : " ");
  lcd.print(" Rename          ");

  lcd.setCursor(0, 3);
  lcd.print(menuIndex == 1 ? ">" : " ");
  lcd.print(" Factory reset   ");
}

// ================== ENCODER: XOAY MƯỢT TỪNG NẤC ==================
void handleEncoderRotation() {
  // Đọc trạng thái hiện tại của encoder (CLK=A, DT=B)
  uint8_t A = digitalRead(ENC_CLK_PIN);
  uint8_t B = digitalRead(ENC_DT_PIN);
  uint8_t AB = (A << 1) | B;

  // Ghép 2 bit cũ + 2 bit mới thành 4 bit index cho encTable
  uint8_t index = (encPrevAB << 2) | AB;
  encPrevAB = AB;

  int8_t step = encTable[index & 0x0F];
  if (step == 0) return;

  // Tích luỹ, đủ ±4 thì coi là 1 "click" encoder
  encAccum += step;
  if (encAccum >= 4) {
    encAccum = 0;
    // quay theo 1 chiều
    menuIndex++;
  } else if (encAccum <= -4) {
    encAccum = 0;
    // quay chiều ngược lại
    menuIndex--;
  } else {
    return; // chưa đủ 4 bước, chưa đổi menu
  }

  // Giới hạn & vòng lại
  if (menuIndex < 0)           menuIndex = MENU_ITEMS - 1;
  if (menuIndex >= MENU_ITEMS) menuIndex = 0;

  // Cập nhật menu
  lcd.setCursor(0, 2);
  lcd.print(menuIndex == 0 ? ">" : " ");
  lcd.print(" Rename          ");

  lcd.setCursor(0, 3);
  lcd.print(menuIndex == 1 ? ">" : " ");
  lcd.print(" Factory reset   ");
}

// ================== ENCODER: NHẤN NÚT ==================
void handleEncoderButton() {
  bool sw = digitalRead(ENC_SW_PIN);
  unsigned long now = millis();

  // Bấm xuống
  if (sw == LOW && lastSwState == HIGH) {
    swPressStart = now;
  }

  // Nhả ra -> nhấn ngắn
  if (sw == HIGH && lastSwState == LOW) {
    unsigned long dt = now - swPressStart;
    if (dt > 50 && dt < 1500) {
      if (appState == APP_MENU) {
        if (menuIndex == 0) {
          startRenameSequence();
        } else if (menuIndex == 1) {
          factoryResetCounter();
        }
      }
    }
  }

  lastSwState = sw;
}

// ================== ĐỔI TÊN JDY-33 ==================
void startRenameSequence() {
  appState = APP_RENAMING;

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Renaming JDY...   ");

  // Tính index mới: nếu >99 hoặc =0 thì quay lại 01
  uint16_t newIndex = g_nameIndex + 1;
  if (newIndex > 99 || newIndex == 0) {
    newIndex = 1;
  }

  char suffix[3];  // "01" + '\0'
  snprintf(suffix, sizeof(suffix), "%02u", newIndex);

  String newName = String("JDY_Hshopvn_BLE_") + suffix;

  lcd.setCursor(0, 1);
  lcd.print("Set: ");
  String shortName = newName;
  if (shortName.length() > 15) shortName = shortName.substring(0, 15);
  lcd.print(shortName);

  // Đổi tên SPP
  String cmdName  = String("AT+NAME") + newName;
  String respName = sendATGetResponse(cmdName.c_str());
  respName.trim();
  bool okName = (respName.indexOf("OK") >= 0);

  // Đổi tên BLE
  String cmdNamb  = String("AT+NAMB") + newName;
  String respNamb = sendATGetResponse(cmdNamb.c_str());
  respNamb.trim();
  bool okNamb = (respNamb.indexOf("OK") >= 0);

  bool ok = okName || okNamb;

  lcd.setCursor(0, 2);
  if (ok) {
    lcd.print("Result: OK        ");

    // Lưu index mới
    g_nameIndex = newIndex;
    prefs.putUShort("name_idx", g_nameIndex);

    // Reset JDY để áp dụng tên mới
    sendATGetResponse("AT+RESET", 300);
    delay(1500);

    // Đọc lại tên
    String nameResp = sendATGetResponse("AT+NAME");
    currentName     = extractNameFromResp(nameResp);
    setupNameScroll(currentName);

    lcd.setCursor(0, 3);
    lcd.print("Rescan BT on phone");
  } else {
    lcd.print("Result: FAIL      ");
    lcd.setCursor(0, 3);
    lcd.print("Check module      ");
  }

  delay(1500);

  showMenuScreen();
  appState = APP_MENU;
}

// ================== FACTORY RESET BỘ ĐẾM ==================
void factoryResetCounter() {
  appState = APP_FACTORY_RESET;

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Factory reset cnt ");

  g_nameIndex = 0;
  prefs.putUShort("name_idx", g_nameIndex);

  lcd.setCursor(0, 2);
  lcd.print("Idx->0 (next 01)  ");
  lcd.setCursor(0, 3);
  lcd.print("OK                ");

  delay(1500);

  showMenuScreen();
  appState = APP_MENU;
}
