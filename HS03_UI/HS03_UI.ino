#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_ADS1X15.h>
#include <string.h>
#include <Adafruit_NeoPixel.h>   // NeoPixel WS2812
#include <LedControl.h>          // MAX7219
#include <SoftwareSerial.h>      // <-- THÊM DÒNG NÀY
#include <Preferences.h>

// ======================
// Cấu hình chân
// ======================
#define ENCODER_CLK_PIN  17   // HW-040 CLK
#define ENCODER_DT_PIN   16   // HW-040 DT
#define ENCODER_SW_PIN   15   // HW-040 SW (nút nhấn)

#define LCD_SDA_PIN      6    // LCD2004 SDA
#define LCD_SCL_PIN      7    // LCD2004 SCL

// Bus I2C #1 cho Scan + ADS1115
#define I2C_SCAN_SDA_PIN 1    // SDA: GPIO1
#define I2C_SCAN_SCL_PIN 2    // SCL: GPIO2

// Buzzer
#define BUZZER_PIN       8

// Đèn giao thông
#define LED_RED_PIN      1
#define LED_YELLOW_PIN   2
#define LED_GREEN_PIN    4

#define NEOPIXEL_PIN       5      // WS2812 tại GPIO5
#define NEOPIXEL_LED_COUNT 144    // Tối đa 150 led

#define RS485_RX_PIN  9   // RX ESP32-S3 nhận từ TX của module RS485
#define RS485_TX_PIN  3   // TX ESP32-S3 gửi tới RX của module RS485

SoftwareSerial RS485Serial(RS485_RX_PIN, RS485_TX_PIN);

// LCD
#define LCD_I2C_ADDR     0x27

// ADS1115
#define ADS1115_I2C_ADDR 0x48   // ADDR nối GND -> 0x48

// MAX7219 Matrix Led (8x32)
#define MATRIX_DIN_PIN   1   // DIN -> GPIO1
#define MATRIX_CS_PIN    2   // CS  -> GPIO2
#define MATRIX_CLK_PIN   4   // CLK -> GPIO4
#define MATRIX_DEVICE_COUNT 4   // hỗ trợ tối đa 4 module (8x32 dùng 4 cái 8x8)

// Tạo đối tượng LedControl duy nhất
LedControl matrixLc = LedControl(MATRIX_DIN_PIN, MATRIX_CLK_PIN, MATRIX_CS_PIN, MATRIX_DEVICE_COUNT);

// ======================
// Enum trạng thái
// ======================
enum MenuLevel {
  LEVEL_MAIN = 0,
  LEVEL_I2C_SUB,
  LEVEL_BT_SUB,
  LEVEL_MATRIX_SUB,
  LEVEL_I2C_OLED_SUB,
  LEVEL_RS485_SUB,        // submenu cho Sensor RS485
  LEVEL_SETTINGS_MENU,       // Menu Settings (ẩn)
  LEVEL_SETTINGS_BUZZER_EDIT, // chỉnh thời gian buzzer
  LEVEL_SETTINGS_ABOUT,
  LEVEL_DFROBOT_SUB
};

enum AppState {
  STATE_SPLASH,
  STATE_MENU,
  STATE_I2C_SCAN,
  STATE_I2C_AM2315C,
  STATE_ANALOG,
  STATE_DFROBOT_ANALOG,
  STATE_TRAFFIC_LED,
  STATE_NEOPIXEL,
  STATE_SIMPLE_SCREEN,
  STATE_MATRIX_8X32,     // Matrix 8x32
  STATE_RS485_SHTC3,     // Sensor RS485: SHTC3
  STATE_RS485_AGH3485,     // Sensor RS485: AGH3485 (ASAIR)
  STATE_SEGMENT_4X7_HC595,
  STATE_TM1637,           // 4x7 Segment TM1637
  STATE_PCA9685_TEST,
  STATE_MAX6675,
  STATE_I2C_DHT20_AHT20,
  STATE_I2C_MPU6050,
  STATE_DFROBOT_URM37,
  STATE_I2C_AGR12
};

// ======================
// Cấu hình menu
// ======================
// Menu chính
const char* mainMenuItems[] = {
  "I2C",                 //  0
  "Analog",              //  1
  "Bluetooth",           //  2  (BLE)
  "PS2",                 //  3
  "Sensor RS485",        //  4
  "Ultrasonic JSN",      //  5
  "Traffic Led",         //  6
  "Max6675",             //  7
  "2x7 Seg HC595",       //  8
  "4/8x7 Seg HC595",     //  9
  "4x7 Seg TM1637",      // 10
  "Matrix",              // 11
  "Neopixel",            // 12
  "Grove",               // 13
  "Maker",               // 14
  "DFRobot"              // 15
};

const int MAIN_MENU_COUNT = sizeof(mainMenuItems) / sizeof(mainMenuItems[0]);

// Sub-menu I2C
const char* const i2cSubMenuItems[] = {
  "Scan",
  "Test PCA9685",
  "OLED IIC",
  "AM2315C",
  "DHT20_AHT20",
  "MPU6050",
  "AGR12",
  "<-- Back"
};

const int I2C_MENU_COUNT = sizeof(i2cSubMenuItems) / sizeof(i2cSubMenuItems[0]);
// ======================
// SETTINGS (Menu ẩn)
// ======================
const char* const settingsMenuItems[] = {
  "Set Buzzer",
  "Back to Menu Test",
  "About"
};

const int SETTINGS_MENU_COUNT = sizeof(settingsMenuItems) / sizeof(settingsMenuItems[0]);

int currentSettingsIndex = 0;

// Lưu/đọc thời gian countdown (buzzer kêu khi về 0)
// Range: 3..10 phút, step 30s
Preferences prefs;
static const char* PREF_NS       = "hs03";
static const char* PREF_KEY_CD_S = "cdSec";

uint16_t settingsCountdownSec = 180;     // giá trị đang chỉnh trong Settings
bool settingsSavedHeaderEnabled = true;  // để restore headerEnabled

const char* const btSubMenuItems[] = {
  "1. JDY-33",
  "2. HC-05",
  "3. BT Slot 3",
  "<-- Back"
};

const int BT_MENU_COUNT = sizeof(btSubMenuItems) / sizeof(btSubMenuItems[0]);

// Submenu cho OLED IIC
const char* const i2cOledSubMenuItems[] = {
  "OLED 0.91\"",
  "OLED 0.96\"",
  "OLED 1.3\"",
  "<-- Back"
};
const int I2C_OLED_MENU_COUNT = sizeof(i2cOledSubMenuItems) / sizeof(i2cOledSubMenuItems[0]);

// Sub-menu Led Matrix (chỉ còn Matrix 8x32 + Back)
const char* const matrixSubMenuItems[] = {
  "Matrix 8x32",
  "<-- Back"
};
const int MATRIX_MENU_COUNT = sizeof(matrixSubMenuItems) / sizeof(matrixSubMenuItems[0]);

// Sub-menu Sensor RS485
const char* const rs485SubMenuItems[] = {
  "1. SHTC3",
  "2. AGH3485 (ASAIR)",
  "3. Sensor 3",
  "<-- Back"
};

const int RS485_MENU_COUNT = sizeof(rs485SubMenuItems) / sizeof(rs485SubMenuItems[0]);

// ===== DFRobot feature (menu con cho mục 16) =====
const char* const dfrobotSubMenuItems[] = {
  "DFRobotAnalog",
  "URM37 Ultrasonic",
  "<-- Back"
};
const int DFROBOT_MENU_COUNT = sizeof(dfrobotSubMenuItems) / sizeof(dfrobotSubMenuItems[0]);


// ======================
// Biến toàn cục
// ======================
LiquidCrystal_I2C lcd(LCD_I2C_ADDR, 20, 4);

Adafruit_NeoPixel neoStrip(NEOPIXEL_LED_COUNT, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);
unsigned long lastNeoUpdate = 0;
const unsigned long NEOPIXEL_UPDATE_INTERVAL = 50;  // ms, tốc độ chạy led

// I2C bus 1 cho Scan + ADS1115
TwoWire I2CScanBus(1);
Adafruit_ADS1115 ads1115;
bool ads1115_ok = false;

MenuLevel currentLevel = LEVEL_MAIN;
AppState  appState     = STATE_SPLASH;

int currentMainIndex   = 0;
int currentI2CIndex    = 0;
int currentI2COLEDIndex = 0;   // index cho submenu OLED IIC
int currentMatrixIndex = 0;    // index cho sub-menu Led Matrix
int currentRS485Index  = 0;    // index cho sub-menu RS485
int currentBTIndex     = 0;    // index cho submenu Bluetooth
int currentDFRobotIndex = 0;

int  lastClkState     = HIGH;
bool lastBtnState     = HIGH;
unsigned long lastBtnTime     = 0;
unsigned long lastEncoderTime = 0;
const unsigned long BTN_DEBOUNCE      = 200; // ms
const unsigned long ENCODER_DEBOUNCE  = 1;   // ms

char headerLabel[16] = "";

// Cờ bật/tắt vẽ header dòng 0
bool headerEnabled = true;

// Countdown 120s + buzzer 10s
uint16_t             COUNTDOWN_SECONDS     = 180;   // sẽ nạp từ NVS
const unsigned long  BUZZER_DURATION       = 10000UL;
unsigned long        countdownStartMillis  = 0;
int                  countdownRemaining    = 180;   // setup() sẽ set lại đúng
bool                 countdownFinished     = false;

bool                 buzzerActive          = false;
unsigned long        buzzerStartMillis     = 0;
bool                 buzzerState           = false;
unsigned long        lastBuzzerToggleMillis = 0;
const unsigned long  BUZZER_TOGGLE_INTERVAL = 200;

// Thời gian cập nhật cho Analog & DFRobot
const unsigned long  ANALOG_UPDATE_INTERVAL   = 500;
unsigned long        lastAnalogUpdate         = 0;

const unsigned long  DFROBOT_UPDATE_INTERVAL  = 500;
unsigned long        lastDFRobotUpdate        = 0;

// ======================
// Prototype
// ======================
void onEncoderTurn(int direction);
void onButtonClick();
void exitPCA9685ToI2CMenu();
void printMatrixSubMenuItem();
void printRS485SubMenuItem();
void printI2COLEDSubMenuItem();
void drawMatrixHeader(uint8_t funcIndex);
void drawRS485Header(uint8_t funcIndex);
void drawI2COLEDHeader(uint8_t funcIndex);
void printBTSubMenuItem();
void drawBTHeader(uint8_t funcIndex);    // <-- THÊM DÒNG NÀY

void enterSettingsMenu();
void exitSettingsMenuToMain();
void printSettingsMenuItem();
void printSettingsBuzzerEdit();
void printSettingsAbout();

uint16_t normalizeCountdownSec(uint16_t sec);
void loadCountdownSetting();
void saveCountdownSetting(uint16_t sec);
void restartCountdownNow(unsigned long now);

// Kéo các file header
#include "Display.h"
#include "CountdownBuzzer.h"
#include "I2CScanMode.h"
#include "PCA9685TestMode.h"
#include "AnalogMode.h"
#include "DFRobotAnalog.h"
#include "SimpleScreens.h"
#include "TrafficLedMode.h"
#include "NeoPixelMode.h"
#include "MatrixLedMode.h"
#include "RS485SHTC3Mode.h"   // mode đọc SHTC3 RS485
#include "TM1637Mode.h"       // mode 4x7 Segment TM1637 (mới)
#include "Segment4x7HC595Mode.h" // mode 4x7 Segment HC595 (blocking)
#include "Segment2x7HC595Mode.h"   // mode 2x7 Segment HC595 (blocking)
#include "UltrasonicJSNMode.h"   // <--- THÊM DÒNG NÀY
#include "PS2Mode.h"
#include "MAX6675Mode.h"
#include "OLED091Mode.h"
#include "OLED096Mode.h"
#include "OLED13Mode.h"
#include "BluetoothJDY33Mode.h"   // mode Bluetooth JDY-33
#include "BluetoothHC05Mode.h"    // mode Bluetooth HC-05
#include "RS485AGH3485Mode.h" // mode đọc AGH3485 (ASAIR) RS485
#include "AM2315CMode.h"
#include "DHT20AHT20Mode.h"
#include "MPU6050Mode.h"
#include "URM37Mode.h"
#include "AGR12Mode.h"

// Định nghĩa object MAX6675 dùng chung cho mode
MAX6675 max6675(MAX6675_CLK_PIN, MAX6675_CS_PIN, MAX6675_DO_PIN);

// ======================
// Hàm vẽ header cho Led Matrix
// Hiển thị dạng: "Matrix 1/1  NNs"
// ======================
void drawMatrixHeader(uint8_t funcIndex) {
  // Hiện tại chỉ còn 1 chức năng Matrix 8x32 -> luôn hiển thị 1/1
  const uint8_t cur   = 1;
  const uint8_t total = 1;

  int cd = countdownRemaining;
  if (cd < 0) cd = 0;

  char buf[21];
  snprintf(buf, sizeof(buf), "Matrix %u/%u %3ds", cur, total, cd);

  uint8_t len = strlen(buf);
  while (len < 20) {
    buf[len++] = ' ';
  }
  buf[20] = '\0';

  lcd.setCursor(0, 0);
  lcd.print(buf);
}

// ======================
// Hàm vẽ header cho Sensor RS485
// Hiển thị dạng: "RS485 x/3  NNs"
// ======================
void drawRS485Header(uint8_t funcIndex) {
  // funcIndex: 0..3 (0=SHTC3, 1=Sensor2, 2=Sensor3, 3=Back)
  uint8_t sensorPos;
  if (funcIndex <= 2) sensorPos = funcIndex + 1; // 1..3
  else sensorPos = 3;                            // Back xem như 3/3

  const uint8_t total = 3;

  int cd = countdownRemaining;
  if (cd < 0) cd = 0;

  char buf[21];
  snprintf(buf, sizeof(buf), "RS485 %u/%u %3ds", sensorPos, total, cd);

  uint8_t len = strlen(buf);
  while (len < 20) {
    buf[len++] = ' ';
  }
  buf[20] = '\0';

  lcd.setCursor(0, 0);
  lcd.print(buf);
}

// ======================
// SETUP
// ======================
void setup() {
  // I2C cho LCD: bus 0 (Wire)
  Wire.begin(LCD_SDA_PIN, LCD_SCL_PIN);
  lcd.init();
  lcd.backlight();

  // I2C bus 1 cho Scan + ADS1115
  I2CScanBus.begin(I2C_SCAN_SDA_PIN, I2C_SCAN_SCL_PIN, 100000);

  // Khởi tạo NeoPixel (WS2812)
  neoStrip.begin();
  neoStrip.show();        // Tắt hết
  neoStrip.setBrightness(64);   // Độ sáng vừa phải (0–255)

  // Khởi tạo MAX7219 Matrix rồi tắt hết để tiết kiệm năng lượng
  matrixInitDevices();    // setIntensity, clear, bật chip
  matrixShutdownAll();    // sau đó shutdown toàn bộ, chỉ bật khi vào Led Matrix

  // Màn hình chào 1.5s
  lcd.clear();
  lcdPrintLine(0, "********************");
  lcdPrintLine(1, "*  HSHOP TEST KIT  *");
  lcdPrintLine(2, "*XIN CHAO QUY KHACH*");
  lcdPrintLine(3, "********************");
  delay(500);

  // Cấu hình IO
  pinMode(ENCODER_CLK_PIN, INPUT_PULLUP);
  pinMode(ENCODER_DT_PIN,  INPUT_PULLUP);
  pinMode(ENCODER_SW_PIN,  INPUT_PULLUP);

  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  pinMode(LED_RED_PIN, OUTPUT);
  pinMode(LED_YELLOW_PIN, OUTPUT);
  pinMode(LED_GREEN_PIN, OUTPUT);
  digitalWrite(LED_RED_PIN, LOW);
  digitalWrite(LED_YELLOW_PIN, LOW);
  digitalWrite(LED_GREEN_PIN, LOW);

  lastClkState = digitalRead(ENCODER_CLK_PIN);
  lastBtnState = digitalRead(ENCODER_SW_PIN);

  // ADS1115 trên I2CScanBus (SDA=1, SCL=2)
  ads1115_ok = ads1115.begin(ADS1115_I2C_ADDR, &I2CScanBus);
  if (ads1115_ok) {
    ads1115.setGain(GAIN_TWOTHIRDS); // ±6.144V
  }

  // Khởi động countdown
  // Nạp thời gian countdown từ NVS (3..10 phút, step 30s)
  loadCountdownSetting();
  // Khởi động countdown theo giá trị đã nạp
  restartCountdownNow(millis());


  // Về menu chính
  lcd.clear();
  appState     = STATE_MENU;
  currentLevel = LEVEL_MAIN;
  strncpy(headerLabel, "Menu", sizeof(headerLabel));
  headerLabel[sizeof(headerLabel) - 1] = '\0';
  printMainMenuItem();
}

void exitPCA9685ToI2CMenu() {
  // Tắt xung servo trước khi thoát
  stopPCA9685TestMode();

  // Về lại menu I2C
  appState    = STATE_MENU;
  currentLevel = LEVEL_I2C_SUB;
  currentI2CIndex = 1;   // mục "Test PCA9685"

  strncpy(headerLabel, "I2C Menu", sizeof(headerLabel));
  headerLabel[sizeof(headerLabel) - 1] = '\0';

  printI2CSubMenuItem();   // hàm này sẽ cập nhật LCD (gồm cả dòng header)
}

void printI2COLEDSubMenuItem() {
  lcd.clear();

  // Header riêng cho menu OLED: "OLED x/3  NNs"
  drawI2COLEDHeader(currentI2COLEDIndex);

  // Dòng 1: tên mục con đang chọn
  lcdPrintLine(1, i2cOledSubMenuItems[currentI2COLEDIndex]);

  // Dòng 2-3: gợi ý thao tác (giữ nguyên)
  lcdPrintLine(2, "Xoay encoder de chon");
  lcdPrintLine(3, "Nhan nut de OK");
}

// ======================
// LOOP
// ======================
void loop() {
  unsigned long now = millis();

  // Đếm ngược 120s + buzzer 10s
  updateCountdown(now);
  handleBuzzer(now);

  // Cập nhật theo trạng thái
  switch (appState) {
    case STATE_ANALOG:
      updateAnalogMode(now);
      break;

    case STATE_I2C_AGR12:
      updateAGR12Mode((uint32_t)now);
      break;

    case STATE_DFROBOT_ANALOG:
      updateDFRobotAnalogMode(now);
      break;

    case STATE_DFROBOT_URM37:
      updateURM37Mode(now);
      break;

    case STATE_TRAFFIC_LED:
      updateTrafficLedMode(now);
      break;

    case STATE_I2C_SCAN:
      updateI2CScanMode(now);
      break;

    case STATE_I2C_AM2315C:       
      updateAM2315CMode(now);
      break;

    case STATE_I2C_DHT20_AHT20:
      updateDHT20AHT20Mode(now);
      break;

    case STATE_NEOPIXEL:
      updateNeoPixelMode(now);
      break;

    case STATE_MATRIX_8X32:
      updateMatrix8x32Mode();    // mưa rơi 8x32
      break;

    case STATE_RS485_SHTC3:
      updateRS485SHTC3Mode(now); // đọc SHTC3 RS485 & hiển thị LCD
      break;

    case STATE_RS485_AGH3485:
      updateRS485AGH3485Mode(now); // đọc AGH3485 RS485 & hiển thị LCD
      break;

    case STATE_SEGMENT_4X7_HC595:
        update4x7HC595Mode(now);   // hàm này đã alias sang header mới
      break;

    case STATE_TM1637:
      updateTM1637Mode(now);     // cập nhật TM1637 (12:34 + nháy colon)
      break;

    case STATE_MAX6675:
      updateMAX6675Mode(now);
      break;

    case STATE_PCA9685_TEST:
      updatePCA9685TestMode(now);
      break;

    case STATE_I2C_MPU6050:
      updateMPU6050Mode(now);
      break;

    default:
      break;
  }

  // Đọc xoay encoder (CLK)
  int clkState = digitalRead(ENCODER_CLK_PIN);
  if (clkState != lastClkState) {
    if (clkState == LOW && (now - lastEncoderTime) > ENCODER_DEBOUNCE) {
      int dtState   = digitalRead(ENCODER_DT_PIN);
      int direction = (dtState == HIGH) ? +1 : -1;
      onEncoderTurn(direction);
      lastEncoderTime = now;
    }
    lastClkState = clkState;
  }

  // Đọc nút nhấn
// =====================================================
// Đọc nút nhấn Encoder (click + long-press 10s vào Settings)
// - Long press 10s CHỈ áp dụng ở MENU CHÍNH (STATE_MENU + LEVEL_MAIN)
// - Ở mọi tính năng khác: click xử lý NGAY khi nhấn xuống (giữ nguyên hành vi cũ)
// =====================================================
int btnState = digitalRead(ENCODER_SW_PIN);

// biến phục vụ long-press (chỉ dùng ở menu chính)
static bool          holdActive  = false;
static bool          holdFired   = false;
static unsigned long holdStartMs = 0;

// Nếu đang hold mà rời khỏi menu chính thì hủy hold (tránh kẹt trạng thái)
if (holdActive && !(appState == STATE_MENU && currentLevel == LEVEL_MAIN)) {
  holdActive = false;
  holdFired  = false;
}

// Debounce theo cạnh
if (btnState != lastBtnState) {
  if ((now - lastBtnTime) > BTN_DEBOUNCE) {

    // Cạnh nhấn xuống
    if (lastBtnState == HIGH && btnState == LOW) {
      if (appState == STATE_MENU && currentLevel == LEVEL_MAIN) {
        // Ở menu chính: không click ngay, chờ thả để click / hoặc đủ 10s để vào Settings
        holdActive  = true;
        holdFired   = false;
        holdStartMs = now;
      } else {
        // Ở mọi tính năng khác: click NGAY khi nhấn xuống (giữ nguyên hành vi cũ)
        onButtonClick();
      }
    }

    // Cạnh nhả ra
    if (lastBtnState == LOW && btnState == HIGH) {
      // Nếu đang hold ở menu chính và chưa kích hoạt Settings -> coi là 1 click bình thường
      if (holdActive) {
        if (!holdFired) {
          onButtonClick();
        }
        holdActive = false;
        holdFired  = false;
      }
    }

    lastBtnTime  = now;
    lastBtnState = btnState;
  }
}

// Long press check (chỉ ở menu chính)
if (holdActive && !holdFired &&
    appState == STATE_MENU && currentLevel == LEVEL_MAIN &&
    btnState == LOW &&
    (now - holdStartMs) >= 10000UL) {

  holdFired = true;
  enterSettingsMenu();   // hàm bạn đã thêm cho Menu Settings
  // Không gọi onButtonClick ở đây, để tránh "click" ngay khi nhả nút
}

  // ------------------------------
  // ĐẢM BẢO HEADER LED MATRIX & RS485 LUÔN ĐÚNG
  // ------------------------------
  if (appState == STATE_MENU) {
    if (currentLevel == LEVEL_MATRIX_SUB) {
      drawMatrixHeader(currentMatrixIndex);
    } else if (currentLevel == LEVEL_RS485_SUB) {
      drawRS485Header(currentRS485Index);
    } else if (currentLevel == LEVEL_I2C_OLED_SUB) {
    // Đảm bảo khi đang ở menu OLED thì header luôn là "OLED x/3 NNs"
    drawI2COLEDHeader(currentI2COLEDIndex);
    } else if (currentLevel == LEVEL_BT_SUB) {
      // Đảm bảo khi đang ở menu Bluetooth thì header luôn là "BLE x/3 NNs"
      drawBTHeader(currentBTIndex);
    }
  } else if (appState == STATE_MATRIX_8X32) {
    drawMatrixHeader(0);
  } else if (appState == STATE_RS485_SHTC3) {
    drawRS485Header(0);   // SHTC3 là sensor 1/3
  } else if (appState == STATE_RS485_AGH3485) {
    drawRS485Header(1);   // AGH3485 là sensor 2/3
  }
}

// ======================
// Hàm vẽ header cho OLED IIC
// Hiển thị dạng: "OLED x/3  NNs"
// ======================
void drawI2COLEDHeader(uint8_t funcIndex) {
  // funcIndex: 0..3 (0=0.91, 1=0.96, 2=1.3, 3=Back)
  uint8_t pos;
  if (funcIndex <= 2) {
    pos = funcIndex + 1;   // 1..3 cho 3 loại OLED
  } else {
    pos = 3;               // "<-- Back" xem như 3/3
  }

  const uint8_t total = 3; // 3 loại OLED

  int cd = countdownRemaining;
  if (cd < 0) cd = 0;

  char buf[21];
  snprintf(buf, sizeof(buf), "OLED %u/%u %3ds", pos, total, cd);

  uint8_t len = strlen(buf);
  while (len < 20) {
    buf[len++] = ' ';
  }
  buf[20] = '\0';

  lcd.setCursor(0, 0);
  lcd.print(buf);
}

// ======================
// Hàm vẽ header cho Bluetooth
// Hiển thị dạng: "BLE x/3  NNs"
// ======================
void drawBTHeader(uint8_t funcIndex) {
  // funcIndex: 0..3 (0=JDY-33, 1=BT Slot 2, 2=BT Slot 3, 3=Back)
  uint8_t pos;
  if (funcIndex <= 2) {
    pos = funcIndex + 1;   // 1..3 cho 3 slot BT
  } else {
    pos = 3;               // "<-- Back" xem như 3/3
  }

  const uint8_t total = 3; // 3 slot Bluetooth

  int cd = countdownRemaining;
  if (cd < 0) cd = 0;

  char buf[21];
  snprintf(buf, sizeof(buf), "BLE %u/%u %3ds", pos, total, cd);

  uint8_t len = strlen(buf);
  while (len < 20) {
    buf[len++] = ' ';
  }
  buf[20] = '\0';

  lcd.setCursor(0, 0);
  lcd.print(buf);
}

// ======================
// Xử lý xoay encoder
// ======================
void onEncoderTurn(int direction) {
  // Đang ở mode Test PCA9685 -> dùng encoder để chỉnh góc servo
  if (appState == STATE_PCA9685_TEST) {
    // Không đụng tới logic trong mode PCA9685
    pca9685OnEncoderTurn(direction);
    return;
  }

  // ===== SETTINGS MENU (không dùng tăng tốc bước nhảy) =====
  if (appState == STATE_MENU && currentLevel == LEVEL_SETTINGS_MENU) {
    currentSettingsIndex += direction;
    if (currentSettingsIndex < 0) currentSettingsIndex = SETTINGS_MENU_COUNT - 1;
    if (currentSettingsIndex >= SETTINGS_MENU_COUNT) currentSettingsIndex = 0;
    printSettingsMenuItem();
    return;
  }

  if (appState == STATE_MENU && currentLevel == LEVEL_SETTINGS_BUZZER_EDIT) {
    int32_t v = (int32_t)settingsCountdownSec + (int32_t)direction * 30; // step 30s
    if (v < 0) v = 0;
    settingsCountdownSec = normalizeCountdownSec((uint16_t)v);
    printSettingsBuzzerEdit();
    return;
  }

  if (appState == STATE_MENU && currentLevel == LEVEL_DFROBOT_SUB) {
  currentDFRobotIndex += direction;
  if (currentDFRobotIndex < 0) currentDFRobotIndex = DFROBOT_MENU_COUNT - 1;
  if (currentDFRobotIndex >= DFROBOT_MENU_COUNT) currentDFRobotIndex = 0;
  printDFRobotSubMenuItem();
  return;
  }

  // Chỉ cho phép xoay khi ở MENU (con trỏ menu)
  if (appState != STATE_MENU) return;

  // ====== TĂNG TỐC THEO TỐC ĐỘ XOAY ======
  // Xoay càng nhanh thì bước nhảy càng lớn (2 hoặc 3 bước/lần)
  static unsigned long lastTurnMs = 0;
  unsigned long now = millis();
  unsigned long dt  = now - lastTurnMs;
  lastTurnMs = now;

  int step = 1;  // mặc định nhảy 1 mục

  // Nếu quay nhanh (liên tục, dt nhỏ) thì nhảy nhiều mục hơn
  if (dt < 40) {
    step = 3;    // xoay rất nhanh
  } else if (dt < 120) {
    step = 2;    // xoay nhanh vừa
  }

  int delta = direction * step;
  // =======================================

  if (currentLevel == LEVEL_MAIN) {
    currentMainIndex += delta;
    if (currentMainIndex < 0) currentMainIndex = MAIN_MENU_COUNT - 1;
    if (currentMainIndex >= MAIN_MENU_COUNT) currentMainIndex = 0;
    printMainMenuItem();

  } else if (currentLevel == LEVEL_I2C_SUB) {
    currentI2CIndex += delta;
    if (currentI2CIndex < 0) currentI2CIndex = I2C_MENU_COUNT - 1;
    if (currentI2CIndex >= I2C_MENU_COUNT) currentI2CIndex = 0;
    printI2CSubMenuItem();
    
  } else if (currentLevel == LEVEL_DFROBOT_SUB) {
  currentDFRobotIndex += delta;
  if (currentDFRobotIndex < 0) currentDFRobotIndex = DFROBOT_MENU_COUNT - 1;
  if (currentDFRobotIndex >= DFROBOT_MENU_COUNT) currentDFRobotIndex = 0;
  printDFRobotSubMenuItem();

  } else if (currentLevel == LEVEL_BT_SUB) {
    currentBTIndex += delta;
    if (currentBTIndex < 0) currentBTIndex = BT_MENU_COUNT - 1;
    if (currentBTIndex >= BT_MENU_COUNT) currentBTIndex = 0;
    printBTSubMenuItem();

  } else if (currentLevel == LEVEL_I2C_OLED_SUB) {
    currentI2COLEDIndex += delta;
    if (currentI2COLEDIndex < 0) currentI2COLEDIndex = I2C_OLED_MENU_COUNT - 1;
    if (currentI2COLEDIndex >= I2C_OLED_MENU_COUNT) currentI2COLEDIndex = 0;
    printI2COLEDSubMenuItem();

  } else if (currentLevel == LEVEL_MATRIX_SUB) {
    currentMatrixIndex += delta;
    if (currentMatrixIndex < 0) currentMatrixIndex = MATRIX_MENU_COUNT - 1;
    if (currentMatrixIndex >= MATRIX_MENU_COUNT) currentMatrixIndex = 0;
    printMatrixSubMenuItem();   // in lại menu + header Matrix

  } else if (currentLevel == LEVEL_RS485_SUB) {
    currentRS485Index += delta;
    if (currentRS485Index < 0) currentRS485Index = RS485_MENU_COUNT - 1;
    if (currentRS485Index >= RS485_MENU_COUNT) currentRS485Index = 0;
    printRS485SubMenuItem();
  }
}


// ======================
// Xử lý nút nhấn
// ======================
void onButtonClick() {
  unsigned long now = millis();

  if (appState == STATE_I2C_AGR12) {
    stopAGR12Mode();
    appState     = STATE_MENU;
    currentLevel = LEVEL_I2C_SUB;
    printI2CSubMenuItem();
    return;
  }

  if (appState == STATE_DFROBOT_URM37) {
  stopURM37Mode();
  appState = STATE_MENU;
  currentLevel = LEVEL_DFROBOT_SUB;
  currentDFRobotIndex = 1;
  printDFRobotSubMenuItem();
  return;
  }


if (appState == STATE_MENU && currentLevel == LEVEL_DFROBOT_SUB) {
  switch (currentDFRobotIndex) {
    case 0: // DFRobotAnalog (giữ nguyên mode cũ)
      startDFRobotAnalogMode();
      break;

    case 1: // URM37
      appState = STATE_DFROBOT_URM37;
      startURM37Mode();
      break;

    case 2: // Back -> về menu 16 mục
      currentLevel = LEVEL_MAIN;
      currentMainIndex = 15; // đứng đúng mục DFRobot
      printMainMenuItem();
      break;
  }
  return;
}


  // ======================
  // MENU SETTINGS (ẩn)
  // ======================
  if (appState == STATE_MENU && currentLevel == LEVEL_SETTINGS_MENU) {
    if (currentSettingsIndex == 0) {
      // vào chỉnh Set Buzzer
      currentLevel = LEVEL_SETTINGS_BUZZER_EDIT;
      settingsCountdownSec = normalizeCountdownSec(COUNTDOWN_SECONDS);
      printSettingsBuzzerEdit();
    } 
    else if (currentSettingsIndex == 1) {
      // Back to Menu Test
      exitSettingsMenuToMain();
    } 
    else { 
      // About
      currentLevel = LEVEL_SETTINGS_ABOUT;
      printSettingsAbout();
    }
    return;
  }

  if (appState == STATE_MENU && currentLevel == LEVEL_SETTINGS_ABOUT) {
    // Nhấn nút -> quay lại Menu Settings
    currentLevel = LEVEL_SETTINGS_MENU;
    printSettingsMenuItem();
    return;
  }

  if (appState == STATE_MENU && currentLevel == LEVEL_SETTINGS_BUZZER_EDIT) {
    // nhấn để LƯU
    COUNTDOWN_SECONDS = normalizeCountdownSec(settingsCountdownSec);
    saveCountdownSetting(COUNTDOWN_SECONDS);

    // áp dụng ngay (reset countdown + tắt còi nếu đang kêu)
    restartCountdownNow(now);

    // quay về Menu Settings
    currentLevel = LEVEL_SETTINGS_MENU;
    printSettingsMenuItem();
    return;
  }

  // Đang ở TM1637 -> nhấn 1 lần để thoát về MENU
  if (appState == STATE_TM1637) {
    stopTM1637Mode();
    appState = STATE_MENU;
    printMainMenuItem();
    return;
  }

  // Đang ở Test PCA9685
  //  - 1 click: thoát về menu I2C (xử lý trễ trong updatePCA9685TestMode)
  //  - 2 click nhanh: reset 16 servo về 90°
  if (appState == STATE_PCA9685_TEST) {
    pcaOnRawButtonClick();
    return;
  }

  // Đang ở Analog -> về MENU
  if (appState == STATE_ANALOG) {
    appState = STATE_MENU;
    printMainMenuItem();
    return;
  }

  // Đang ở DFRobot Analog -> về MENU, đưa con trỏ về DFRobot
  if (appState == STATE_DFROBOT_ANALOG) {
    appState = STATE_MENU;
    currentLevel = LEVEL_DFROBOT_SUB;
    currentDFRobotIndex = 0;
    printDFRobotSubMenuItem();
    return;
  }


  // Đang ở I2C Scan -> về submenu I2C
  if (appState == STATE_I2C_SCAN) {
    appState     = STATE_MENU;
    currentLevel = LEVEL_I2C_SUB;
    printI2CSubMenuItem();
    return;
  }

  // Đang ở AM2315C -> nhấn nút để quay về submenu I2C
  if (appState == STATE_I2C_AM2315C) {
    stopAM2315CMode();          // hiện tại không làm gì, nhưng để tương lai dễ mở rộng
    appState     = STATE_MENU;
    currentLevel = LEVEL_I2C_SUB;
    printI2CSubMenuItem();
    return;
  }

  if (appState == STATE_I2C_DHT20_AHT20) {
    stopDHT20AHT20Mode();
    appState     = STATE_MENU;
    currentLevel = LEVEL_I2C_SUB;
    printI2CSubMenuItem();
    return;
  }

  // Đang ở Traffic Led -> tắt đèn & về MENU
  if (appState == STATE_TRAFFIC_LED) {
    stopTrafficLedMode();
    appState = STATE_MENU;
    printMainMenuItem();
    return;
  }
  
  if (appState == STATE_I2C_MPU6050) {
    stopMPU6050Mode();
    appState     = STATE_MENU;
    currentLevel = LEVEL_I2C_SUB;
    printI2CSubMenuItem();
    return;
  }

  // Đang ở SIMPLE SCREEN -> về menu phù hợp
  if (appState == STATE_SIMPLE_SCREEN) {
    appState = STATE_MENU;
    if (currentLevel == LEVEL_I2C_SUB) {
      printI2CSubMenuItem();
    } else if (currentLevel == LEVEL_I2C_OLED_SUB) {
      printI2COLEDSubMenuItem();
    } else if (currentLevel == LEVEL_BT_SUB) {
      printBTSubMenuItem();
    } else if (currentLevel == LEVEL_MATRIX_SUB) {
      printMatrixSubMenuItem();
    } else if (currentLevel == LEVEL_RS485_SUB) {
      printRS485SubMenuItem();
    } else {
      printMainMenuItem();
    }
    return;
  }

  // Đang ở NeoPixel -> tắt led & về MENU
  if (appState == STATE_NEOPIXEL) {
    stopNeoPixelMode();        // tắt hết NeoPixel
    appState = STATE_MENU;
    printMainMenuItem();
    return;
  }

  // Đang ở Matrix Led -> nút nhấn để quay về sub-menu Led Matrix
  if (appState == STATE_MATRIX_8X32) {
    stopMatrixLedMode();       // clear + shutdown toàn bộ matrix
    appState     = STATE_MENU;
    currentLevel = LEVEL_MATRIX_SUB;
    printMatrixSubMenuItem();
    return;
  }

  // Đang ở RS485 SHTC3 -> nhấn nút để thoát về submenu RS485
  if (appState == STATE_RS485_SHTC3) {
    stopRS485SHTC3Mode();    // dọn LCD dòng 1–3
    appState     = STATE_MENU;
    currentLevel = LEVEL_RS485_SUB;
    printRS485SubMenuItem();
    return;
  }
  
  // Đang ở RS485 AGH3485 -> nhấn nút để thoát về submenu RS485
  if (appState == STATE_RS485_AGH3485) {
    stopRS485AGH3485Mode();    // dọn LCD dòng 1–3
    appState     = STATE_MENU;
    currentLevel = LEVEL_RS485_SUB;
    printRS485SubMenuItem();
    return;
  }


  // Đang ở 4x7 Segment HC595 (4/8 LED)
  //  - Lần nhấn đầu (ở trang chọn 4/8) -> handle4x7HC595ButtonClick() trả true -> CHỌN mode, KHÔNG thoát
  //  - Lần nhấn sau (khi đang RUN)     -> handle... trả false -> thoát về MENU như cũ
  if (appState == STATE_SEGMENT_4X7_HC595) {
    if (!handle4x7HC595ButtonClick()) {
      // Không còn xử lý nội bộ -> thoát mode
      stop4x7HC595Mode();
      appState = STATE_MENU;
      printMainMenuItem();
    }
    return;
  }

  // Đang ở MAX6675 -> nhấn nút để quay về MENU
  if (appState == STATE_MAX6675) {
    stopMAX6675Mode();
    appState = STATE_MENU;
    printMainMenuItem();
    return;
  }

  // Đang ở submenu RS485: chọn sensor
  if (appState == STATE_MENU && currentLevel == LEVEL_RS485_SUB) {
    if (currentRS485Index == 0) {
      // 1. SHTC3
      appState = STATE_RS485_SHTC3;
      startRS485SHTC3Mode();
    } else if (currentRS485Index == 1) {
      // 2. AGH3485 (ASAIR)
      appState = STATE_RS485_AGH3485;
      startRS485AGH3485Mode();
    } else if (currentRS485Index == 2) {
      // 3. Sensor 3 (chưa dùng) -> màn "coming soon"
      lcd.clear();
      lcdPrintLine(0, "RS485 Sensor 3   ");
      lcdPrintLine(1, "(Chua cai dat)   ");
      lcdPrintLine(2, "Nhan nut de Back ");
      lcdPrintLine(3, " ");
      appState = STATE_SIMPLE_SCREEN;
    } else {
      // "<-- Back" -> quay lại MENU chính
      currentLevel     = LEVEL_MAIN;
      currentMainIndex = 4;   // "Sensor RS485" trong mainMenuItems
      strncpy(headerLabel, "Menu", sizeof(headerLabel));
      headerLabel[sizeof(headerLabel) - 1] = '\0';
      printMainMenuItem();
    }
    return;
  }

  // Nếu KHÔNG phải đang ở STATE_MENU thì không xử lý thêm
  if (appState != STATE_MENU) return;

  // ==========================
  // ĐANG Ở MENU: xử lý chọn
  // ==========================
  if (currentLevel == LEVEL_MAIN) {
    // ===== MENU CHÍNH =====
    switch (currentMainIndex) {
      case 0: // I2C
        currentLevel    = LEVEL_I2C_SUB;
        currentI2CIndex = 0;
        strncpy(headerLabel, "I2C Menu", sizeof(headerLabel));
        headerLabel[sizeof(headerLabel) - 1] = '\0';
        printI2CSubMenuItem();
        break;

      case 1: // Analog (ReadAnalog)
        startAnalogMode();
        break;

      case 2: { // Bluetooth -> vào submenu Bluetooth
        currentLevel   = LEVEL_BT_SUB;
        currentBTIndex = 0;
        strncpy(headerLabel, "Bluetooth", sizeof(headerLabel));
        headerLabel[sizeof(headerLabel) - 1] = '\0';
        printBTSubMenuItem();
      } break;

      case 3: { // PS2
        // Chạy mode PS2 (blocking), hiển thị LCD đúng chương trình gốc
        startPS2Mode();

        // Sau khi thoát PS2 -> quay lại MENU chính
        appState     = STATE_MENU;
        currentLevel = LEVEL_MAIN;
        strncpy(headerLabel, "Menu", sizeof(headerLabel));
        headerLabel[sizeof(headerLabel) - 1] = '\0';
        printMainMenuItem();
      } break;

      case 4: { // Sensor RS485
        currentLevel      = LEVEL_RS485_SUB;
        currentRS485Index = 0;
        strncpy(headerLabel, "RS485", sizeof(headerLabel));
        headerLabel[sizeof(headerLabel) - 1] = '\0';
        printRS485SubMenuItem();
      } break;

      case 5: { // Ultrasonic JSN
        // Hiển thị trang thông tin trên LCD
        lcd.clear();
        lcdPrintLine(0, "Ultrasonic JSN");
        lcdPrintLine(1, "JSN-SR04T Mode 0");
        lcdPrintLine(2, "Nhan nut de thoat");
        lcdPrintLine(3, " ");

        // Chạy mode đo khoảng cách (blocking)
        startUltrasonicJSNMode();

        // Sau khi thoát thì quay lại MENU chính
        appState     = STATE_MENU;
        currentLevel = LEVEL_MAIN;
        strncpy(headerLabel, "Menu", sizeof(headerLabel));
        headerLabel[sizeof(headerLabel) - 1] = '\0';
        printMainMenuItem();
      } break;

      case 6: // Traffic Led
        startTrafficLedMode();
        break;

      case 7: { // Max6675
        appState = STATE_MAX6675;
        strncpy(headerLabel, "Max6675", sizeof(headerLabel));
        headerLabel[sizeof(headerLabel) - 1] = '\0';
        updateHeaderRow();

        // Dòng 1–3 do mode MAX6675 tự xử lý
        startMAX6675Mode();
      } break;

      case 8: { // 2x7 Segment HC595
        // Vẽ màn hình hướng dẫn
        lcd.clear();
        lcdPrintLine(0, "2x7 Segment HC595");
        lcdPrintLine(1, "Hien thi 0..9");
        lcdPrintLine(2, "DP chop sau moi so");
        lcdPrintLine(3, "Nhan nut de thoat");

        // Chạy mode 2x7 HC595 (blocking)
        start2x7HC595Mode();

        // Sau khi thoát thì quay lại MENU như bình thường
        appState     = STATE_MENU;
        currentLevel = LEVEL_MAIN;
        strncpy(headerLabel, "Menu", sizeof(headerLabel));
        headerLabel[sizeof(headerLabel) - 1] = '\0';
        printMainMenuItem();
      } break;

      case 9: { // 4/8x7 Segment HC595
        appState = STATE_SEGMENT_4X7_HC595;

        // Header dòng 0 vẫn do updateHeaderRow() vẽ (Menu + countdown)
        strncpy(headerLabel, "4x7 HC595", sizeof(headerLabel));
        headerLabel[sizeof(headerLabel) - 1] = '\0';
        updateHeaderRow();

        // Hướng dẫn trên LCD:
        lcdPrintLine(1, "Nhan nut de thoat");
        lcdPrintLine(2, "Mode: 4/8 LED 7 doan");
        lcdPrintLine(3, "Xoay encoder doi");

        // Khởi động mode 4/8x7
        start4x7HC595Mode();
      } break;

      case 10: { // 4x7 Segment TM1637
        lcd.clear();
        lcdPrintLine(1, "TM1637: 12:34");
        lcdPrintLine(2, "Nhan nut de thoat");
        lcdPrintLine(3, " ");

        // Chạy demo TM1637 (12:34, nháy colon, thoát khi nhấn nút)
        startTM1637Mode();

        // Sau khi thoát thì quay lại MENU như bình thường
        appState     = STATE_MENU;
        currentLevel = LEVEL_MAIN;
        strncpy(headerLabel, "Menu", sizeof(headerLabel));
        headerLabel[sizeof(headerLabel) - 1] = '\0';
        printMainMenuItem();
      } break;

      case 11: { // Matrix (sub-menu)
        currentLevel       = LEVEL_MATRIX_SUB;
        currentMatrixIndex = 0;
        strncpy(headerLabel, "Matrix", sizeof(headerLabel));
        headerLabel[sizeof(headerLabel) - 1] = '\0';
        printMatrixSubMenuItem();
      } break;

      case 12: { // Neopixel
        lcd.clear();
        lcdPrintLine(0, "Neopixel");
        lcdPrintLine(1, "Chu ky RGB+Trang");
        lcdPrintLine(2, "Nhan nut de dung");
        lcdPrintLine(3, " ");

        // Chạy mode NeoPixel (blocking)
        startNeoPixelMode();

        // Sau khi thoát thì quay lại MENU chính
        appState     = STATE_MENU;
        currentLevel = LEVEL_MAIN;
        strncpy(headerLabel, "Menu", sizeof(headerLabel));
        headerLabel[sizeof(headerLabel) - 1] = '\0';
        printMainMenuItem();
      } break;

      case 15: { // DFRobot
        currentLevel = LEVEL_DFROBOT_SUB;
        currentDFRobotIndex = 0;
        printDFRobotSubMenuItem();
      } break;

      default:
        // Grove, Maker... dùng màn hình mặc định
        showMainFunctionScreen(currentMainIndex);
        break;
    }

  } else if (currentLevel == LEVEL_I2C_SUB) {
    switch (currentI2CIndex) {
      case 0: // I2C Scan
        startI2CScanMode();
        break;

      case 1: { // Test PCA9685
        appState = STATE_PCA9685_TEST;
        strncpy(headerLabel, "Test PCA9685", sizeof(headerLabel));
        headerLabel[sizeof(headerLabel) - 1] = '\0';
        updateHeaderRow();
        startPCA9685TestMode();
      } break;

      case 2: { // OLED IIC
        currentLevel        = LEVEL_I2C_OLED_SUB;
        currentI2COLEDIndex = 0;
        strncpy(headerLabel, "OLED IIC", sizeof(headerLabel));
        headerLabel[sizeof(headerLabel) - 1] = '\0';
        printI2COLEDSubMenuItem();
      } break;

      case 3: { // AM2315C
        appState = STATE_I2C_AM2315C;
        strncpy(headerLabel, "AM2315C", sizeof(headerLabel));
        headerLabel[sizeof(headerLabel) - 1] = '\0';
        updateHeaderRow();          // vẽ header: "AM2315C  xxxs"
        startAM2315CMode();         // khởi động mode
      } break;

      case 4: { // DHT20_AHT20
        appState = STATE_I2C_DHT20_AHT20;
        strncpy(headerLabel, "DHT20_AHT20", sizeof(headerLabel));
        headerLabel[sizeof(headerLabel) - 1] = '\0';
        // KHÔNG cần updateHeaderRow vì mode sẽ tự dùng dòng 0
        startDHT20AHT20Mode();
      } break;

      case 5: { // MPU6050
        appState = STATE_I2C_MPU6050;
        strncpy(headerLabel, "MPU6050", sizeof(headerLabel));
        headerLabel[sizeof(headerLabel) - 1] = '\0';
        // KHÔNG updateHeaderRow để không ghi đè dòng 0 (mode tự hiển thị)
        startMPU6050Mode();
      } break;

      case 6: { // AGR12
        appState = STATE_I2C_AGR12;
        // không updateHeaderRow để giữ nguyên hiển thị line0 của AGR12
        startAGR12Mode();
      } break;

      case 7: // <-- Back
        currentLevel = LEVEL_MAIN;
        strncpy(headerLabel, "Menu", sizeof(headerLabel));
        headerLabel[sizeof(headerLabel) - 1] = '\0';
        printMainMenuItem();
      break;

    }
  } else if (currentLevel == LEVEL_I2C_OLED_SUB) {
    // ===== SUB-MENU OLED IIC =====
    switch (currentI2COLEDIndex) {
      case 0: { // OLED 0.91"
        // Chạy demo OLED 0.91" (blocking)
        startOLED091Mode();

        // Sau khi thoát -> quay về menu OLED
        appState     = STATE_MENU;
        currentLevel = LEVEL_I2C_OLED_SUB;
        strncpy(headerLabel, "OLED", sizeof(headerLabel));
        headerLabel[sizeof(headerLabel) - 1] = '\0';
        printI2COLEDSubMenuItem();
      } break;

      case 1: { // OLED 0.96"
        startOLED096Mode();

        appState     = STATE_MENU;
        currentLevel = LEVEL_I2C_OLED_SUB;
        strncpy(headerLabel, "OLED", sizeof(headerLabel));
        headerLabel[sizeof(headerLabel) - 1] = '\0';
        printI2COLEDSubMenuItem();
      } break;

      case 2: { // OLED 1.3"
        startOLED13Mode();

        appState     = STATE_MENU;
        currentLevel = LEVEL_I2C_OLED_SUB;
        strncpy(headerLabel, "OLED", sizeof(headerLabel));
        headerLabel[sizeof(headerLabel) - 1] = '\0';
        printI2COLEDSubMenuItem();
      } break;

      case 3: { // <-- Back -> quay về submenu I2C
        currentLevel = LEVEL_I2C_SUB;
        strncpy(headerLabel, "I2C Menu", sizeof(headerLabel));
        headerLabel[sizeof(headerLabel) - 1] = '\0';
        printI2CSubMenuItem();
      } break;
    }

  } else if (currentLevel == LEVEL_BT_SUB) {
    switch (currentBTIndex) {
      case 0: { // 1. JDY-33
        // Hiển thị màn hình chờ 5 giây trước khi gửi AT
        lcd.clear();
        lcdPrintLine(0, "Bluetooth: JDY-33");
        lcdPrintLine(1, "Khoi dong sau 5s");
        lcdPrintLine(2, "Vui long cho...");
        lcdPrintLine(3, " ");

        // Đợi 5 giây để module ổn định rồi mới gửi lệnh AT
        delay(5000);

        // Chạy chương trình JDY-33 (blocking, dùng header BluetoothJDY33Mode.h)
        startBluetoothJDY33Mode();

        // Sau khi thoát JDY-33 -> quay lại submenu Bluetooth
        appState     = STATE_MENU;
        currentLevel = LEVEL_BT_SUB;
        strncpy(headerLabel, "BLE", sizeof(headerLabel));
        headerLabel[sizeof(headerLabel) - 1] = '\0';
        printBTSubMenuItem();
      } break;

      case 1: { // 2. HC-05
        // Chạy mode HC-05 (blocking, giống chương trình gốc)
        startBluetoothHC05Mode();

        // Nếu sau này ta cho phép thoát khỏi HC-05,
        // phần dưới sẽ đưa ta quay lại submenu Bluetooth:
        appState     = STATE_MENU;
        currentLevel = LEVEL_BT_SUB;
        strncpy(headerLabel, "BLE", sizeof(headerLabel));
        headerLabel[sizeof(headerLabel) - 1] = '\0';
        printBTSubMenuItem();
      } break;


      case 2: { // 3. BT Slot 3 (để trống phát triển sau)
        appState = STATE_SIMPLE_SCREEN;
        strncpy(headerLabel, "BLE", sizeof(headerLabel));
        headerLabel[sizeof(headerLabel) - 1] = '\0';
        updateHeaderRow();
        lcdPrintLine(1, "BT Slot 3");
        lcdPrintLine(2, "Chua cai dat");
        lcdPrintLine(3, "Nhan nut de quay lai");
      } break;

      case 3: { // <-- Back
        currentLevel = LEVEL_MAIN;
        strncpy(headerLabel, "Menu", sizeof(headerLabel));
        headerLabel[sizeof(headerLabel) - 1] = '\0';
        printMainMenuItem();
      } break;
    }
    } else if (currentLevel == LEVEL_MATRIX_SUB) {
    // ===== SUB-MENU MATRIX =====
    switch (currentMatrixIndex) {
      case 0: { // Matrix 8x32
        appState = STATE_MATRIX_8X32;
        drawMatrixHeader(0);       // Header: "Matrix 1/1  NNs"
        lcdPrintLine(1, "Nhan nut de thoat");
        lcdPrintLine(2, "LED: hieu ung mua");
        lcdPrintLine(3, " ");
        startMatrix8x32Mode();
      } break;

      case 1: { // <-- Back
        currentLevel = LEVEL_MAIN;
        strncpy(headerLabel, "Menu", sizeof(headerLabel));
        headerLabel[sizeof(headerLabel) - 1] = '\0';
        printMainMenuItem();
      } break;
    }

  } else if (currentLevel == LEVEL_RS485_SUB) {
    // ===== SUB-MENU SENSOR RS485 =====
    switch (currentRS485Index) {
      case 0: { // 1. SHTC3
        appState = STATE_RS485_SHTC3;
        startRS485SHTC3Mode();  // bắt đầu đọc cảm biến + set LCD
      } break;

      case 1:   // 2. Sensor 2 (chưa dùng)
      case 2: { // 3. Sensor 3 (chưa dùng)
        appState = STATE_SIMPLE_SCREEN;
        lcd.clear();
        lcdPrintLine(0, "Sensor RS485");
        lcdPrintLine(1, rs485SubMenuItems[currentRS485Index]);
        lcdPrintLine(2, "Chua cai dat");
        lcdPrintLine(3, "Nhan nut de quay lai");
      } break;

      case 3: { // <-- Back
        currentLevel = LEVEL_MAIN;
        strncpy(headerLabel, "Menu", sizeof(headerLabel));
        headerLabel[sizeof(headerLabel) - 1] = '\0';
        printMainMenuItem();
      } break;
    } 
  } else if (currentLevel == LEVEL_DFROBOT_SUB) {
  switch (currentDFRobotIndex) {
    case 0: // DFRobotAnalog
      startDFRobotAnalogMode();
      break;

    case 1: // URM37 Ultrasonic
      startURM37Mode();
      break;

    case 2: // <-- Back (về menu 16 mục)
      currentLevel = LEVEL_MAIN;
      currentMainIndex = 15; // đứng lại đúng mục DFRobot
      strncpy(headerLabel, "Menu", sizeof(headerLabel));
      headerLabel[sizeof(headerLabel) - 1] = '\0';
      printMainMenuItem();
      break;
  }
  return;
  }
}

// ======================
// In sub-menu Led Matrix
// ======================
void printMatrixSubMenuItem() {
  lcd.clear();

  // Header riêng cho Led Matrix: Matrix 1/1 + countdown
  drawMatrixHeader(currentMatrixIndex);

  // Dòng 1: tên option hiện tại
  lcdPrintLine(1, matrixSubMenuItems[currentMatrixIndex]);

  // Dòng 2-3: hướng dẫn
  lcdPrintLine(2, "Nhan nut de chon");
  lcdPrintLine(3, "Xoay de doi muc");
}

// ======================
// In sub-menu Sensor RS485
// ======================
void printRS485SubMenuItem() {
  lcd.clear();

  // Header riêng cho RS485: RS485 x/3 + countdown
  drawRS485Header(currentRS485Index);

  // Dòng 1: tên sensor / mục hiện tại
  lcdPrintLine(1, rs485SubMenuItems[currentRS485Index]);

  // Dòng 2–3: hướng dẫn
  lcdPrintLine(2, "Nhan nut de chon");
  lcdPrintLine(3, "Xoay de doi Sensor");
}

// ======================
// In sub-menu Bluetooth
// ======================
void printBTSubMenuItem() {
  lcd.clear();

  // Header riêng cho Bluetooth: "BLE x/3  NNs"
  drawBTHeader(currentBTIndex);

  // Dòng 1: tên mục hiện tại
  lcdPrintLine(1, btSubMenuItems[currentBTIndex]);

  // Dòng 2–3: hướng dẫn
  lcdPrintLine(2, "Nhan nut de chon");
  lcdPrintLine(3, "Xoay de doi muc");
}

// ======================
// SETTINGS HELPERS
// ======================
uint16_t normalizeCountdownSec(uint16_t sec) {
  if (sec < 180) sec = 180;
  if (sec > 600) sec = 600;

  // làm tròn về step 30s
  sec = (uint16_t)(((sec + 15) / 30) * 30);

  if (sec < 180) sec = 180;
  if (sec > 600) sec = 600;
  return sec;
}

void loadCountdownSetting() {
  prefs.begin(PREF_NS, true);
  uint16_t v = prefs.getUShort(PREF_KEY_CD_S, 180);
  prefs.end();

  COUNTDOWN_SECONDS = normalizeCountdownSec(v);
}

void saveCountdownSetting(uint16_t sec) {
  sec = normalizeCountdownSec(sec);
  prefs.begin(PREF_NS, false);
  prefs.putUShort(PREF_KEY_CD_S, sec);
  prefs.end();
}

void restartCountdownNow(unsigned long now) {
  countdownStartMillis = now;
  countdownRemaining   = (int)COUNTDOWN_SECONDS;
  countdownFinished    = false;

  buzzerActive           = false;
  buzzerState            = false;
  lastBuzzerToggleMillis = now;
  digitalWrite(BUZZER_PIN, LOW);
}

void printSettingsMenuItem() {
  lcd.clear();
  lcdPrintLine(0, "   Menu Settings   ");

  lcdPrintLine(1, (currentSettingsIndex == 0) ? ">Set Buzzer" : " Set Buzzer");
  lcdPrintLine(2, (currentSettingsIndex == 1) ? ">Back to Menu Test" : " Back to Menu Test");
  lcdPrintLine(3, (currentSettingsIndex == 2) ? ">About" : " About");
}

static void fmtMMSS(char* out, size_t outsz, uint16_t sec) {
  uint16_t m = sec / 60;
  uint16_t s = sec % 60;
  snprintf(out, outsz, "%02u:%02u", (unsigned)m, (unsigned)s);
}

void printSettingsBuzzerEdit() {
  lcd.clear();
  lcdPrintLine(0, "Set Buzzer");
  char mmss[6];
  fmtMMSS(mmss, sizeof(mmss), settingsCountdownSec);

  char l1[21];
  snprintf(l1, sizeof(l1), "Time: %s", mmss);
  lcdPrintLine(1, l1);

  lcdPrintLine(2, "Xoay: +/- 30s");
  lcdPrintLine(3, "Nhan nut de LUU");
}

void enterSettingsMenu() {
  // khóa menu 16 mục: chuyển level sang Settings
  settingsSavedHeaderEnabled = headerEnabled;
  headerEnabled = false;

  currentLevel = LEVEL_SETTINGS_MENU;
  currentSettingsIndex = 0;
  printSettingsMenuItem();
}

void exitSettingsMenuToMain() {
  headerEnabled = settingsSavedHeaderEnabled;

  appState     = STATE_MENU;
  currentLevel = LEVEL_MAIN;

  strncpy(headerLabel, "Menu", sizeof(headerLabel));
  headerLabel[sizeof(headerLabel) - 1] = '\0';
  printMainMenuItem();
}

void printSettingsAbout() {
  lcd.clear();
  lcdPrintLine(0, "HS03 Multi Test Kit");
  lcdPrintLine(1, "Trainer: hnghao");
  lcdPrintLine(2, "Version: 1.0");
  lcdPrintLine(3, "Base on ChatGPT");
}
