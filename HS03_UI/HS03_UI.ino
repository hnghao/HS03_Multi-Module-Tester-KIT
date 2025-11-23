#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_ADS1X15.h>
#include <string.h>
#include <Adafruit_NeoPixel.h>   // NeoPixel WS2812
#include <LedControl.h>          // MAX7219

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
#define NEOPIXEL_LED_COUNT 150    // Tối đa 150 led

// LCD
#define LCD_I2C_ADDR     0x27

// ADS1115
#define ADS1115_I2C_ADDR 0x48   // ADDR nối GND -> 0x48

// MAX7219 Matrix Led (8x8, 8x32)
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
  LEVEL_MATRIX_SUB          // sub-menu cho Led Matrix
};

enum AppState {
  STATE_SPLASH,
  STATE_MENU,
  STATE_I2C_SCAN,
  STATE_ANALOG,
  STATE_DFROBOT_ANALOG,
  STATE_TRAFFIC_LED,
  STATE_NEOPIXEL,
  STATE_SIMPLE_SCREEN,
  STATE_MATRIX_8X8,     // Matrix 8x8
  STATE_MATRIX_8X32     // Matrix 8x32
};

// ======================
// Cấu hình menu
// ======================
// Menu chính: 1 mục "Led Matrix"
const char* mainMenuItems[] = {
  "I2C",                 //  0
  "DFRobot",             //  1
  "Maker",               //  2
  "Grove",               //  3
  "Sensor RS485",        //  4
  "Traffic Led",         //  5
  "Neopixel",            //  6
  "Max6675",             //  7
  "Ultrasonic JSN",      //  8
  "2x7 Segment HC595",   //  9
  "4x7 Segment HC595",   // 10
  "4x7 Segment TM1637",  // 11
  "PS2",                 // 12
  "BLE",                 // 13
  "Analog",              // 14
  "Led Matrix"           // 15
};

const int MAIN_MENU_COUNT = sizeof(mainMenuItems) / sizeof(mainMenuItems[0]);

// Sub-menu I2C
const char* const i2cSubMenuItems[] = {
  "Scan",
  "Test Data - VL53L1X",
  "<-- Back"
};
const int I2C_MENU_COUNT = sizeof(i2cSubMenuItems) / sizeof(i2cSubMenuItems[0]);

// Sub-menu Led Matrix
const char* const matrixSubMenuItems[] = {
  "Matrix 8x8",
  "Matrix 8x32",
  "<-- Back"
};
const int MATRIX_MENU_COUNT = sizeof(matrixSubMenuItems) / sizeof(matrixSubMenuItems[0]);

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
int currentMatrixIndex = 0;    // index cho sub-menu Led Matrix

int  lastClkState     = HIGH;
bool lastBtnState     = HIGH;
unsigned long lastBtnTime     = 0;
unsigned long lastEncoderTime = 0;
const unsigned long BTN_DEBOUNCE      = 200; // ms
const unsigned long ENCODER_DEBOUNCE  = 3;   // ms

char headerLabel[16] = "";

// Countdown 120s + buzzer 10s
const uint16_t       COUNTDOWN_SECONDS     = 120;
const unsigned long  BUZZER_DURATION       = 10000UL;
unsigned long        countdownStartMillis  = 0;
int                  countdownRemaining    = COUNTDOWN_SECONDS;
bool                 countdownFinished     = false;

bool                 buzzerActive          = false;
unsigned long        buzzerStartMillis     = 0;
bool                 buzzerState           = false;
unsigned long        lastBuzzerToggleMillis = 0;
const unsigned long  BUZZER_TOGGLE_INTERVAL = 200;

// Thời gian cập nhật cho Analog & DFRobot
const unsigned long  ANALOG_UPDATE_INTERVAL   = 200;
unsigned long        lastAnalogUpdate         = 0;

const unsigned long  DFROBOT_UPDATE_INTERVAL  = 300;
unsigned long        lastDFRobotUpdate        = 0;

// ======================
// Prototype
// ======================
void onEncoderTurn(int direction);
void onButtonClick();
void printMatrixSubMenuItem();

// Hàm vẽ header riêng cho Led Matrix: "Matrix 1/2" hoặc "Matrix 2/2"
void drawMatrixHeader(uint8_t funcIndex);

// Kéo các file header
#include "Display.h"
#include "CountdownBuzzer.h"
#include "I2CScanMode.h"
#include "AnalogMode.h"
#include "DFRobotAnalog.h"
#include "SimpleScreens.h"
#include "TrafficLedMode.h"
#include "NeoPixelMode.h"
#include "MatrixLedMode.h"   // led matrix 8x8 & 8x32 (mưa rơi)

// ======================
// Hàm vẽ header cho Led Matrix
// ======================
// funcIndex: 0 -> Matrix 1/2, 1 -> Matrix 2/2
// ======================
// Hàm vẽ header cho Led Matrix
// ======================
// funcIndex: 0 -> Matrix 1/2, 1 -> Matrix 2/2, 2 (Back) vẫn coi là 2/2
void drawMatrixHeader(uint8_t funcIndex) {
  // Xác định số chức năng hiện tại (1/2 hoặc 2/2)
  uint8_t cur = (funcIndex > 1) ? 2 : (funcIndex + 1);  // chỉ 2 chức năng chính: 8x8 & 8x32

  // Đảm bảo countdownRemaining không âm (phòng hờ)
  int cd = countdownRemaining;
  if (cd < 0) cd = 0;

  // Ghép cả dòng 0: "Matrix X/2  NNs"
  // Ví dụ: "Matrix 1/2   98s"
  char buf[21];
  snprintf(buf, sizeof(buf), "Matrix %u/2 %3ds", cur, cd);

  // Đệm thêm khoảng trắng cho đủ 20 ký tự
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
  lcdPrintLine(0, "HShop Multi Tool Tester");
  lcdPrintLine(1, "Version: 1.00");
  lcdPrintLine(2, "Xin kinh chao quy khach !");
  lcdPrintLine(3, "Xin kinh chao quy khach !");
  delay(1500);

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
  countdownStartMillis = millis();
  countdownRemaining   = COUNTDOWN_SECONDS;
  countdownFinished    = false;

  // Về menu chính
  lcd.clear();
  appState     = STATE_MENU;
  currentLevel = LEVEL_MAIN;
  strncpy(headerLabel, "Menu", sizeof(headerLabel));
  headerLabel[sizeof(headerLabel) - 1] = '\0';
  printMainMenuItem();
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

    case STATE_DFROBOT_ANALOG:
      updateDFRobotAnalogMode(now);
      break;

    case STATE_TRAFFIC_LED:
      updateTrafficLedMode(now);
      break;

    case STATE_I2C_SCAN:
      updateI2CScanMode(now);
      break;

    case STATE_NEOPIXEL:
      updateNeoPixelMode(now);
      break;

    case STATE_MATRIX_8X8:
      updateMatrix8x8Mode();     // mưa rơi 8x8
      break;

    case STATE_MATRIX_8X32:
      updateMatrix8x32Mode();    // mưa rơi 8x32
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
  int btnState = digitalRead(ENCODER_SW_PIN);
  if (btnState != lastBtnState) {
    if ((now - lastBtnTime) > BTN_DEBOUNCE) {
      if (lastBtnState == HIGH && btnState == LOW) {
        onButtonClick();
      }
      lastBtnTime = now;
    }
    lastBtnState = btnState;
  }

  // ------------------------------
  // ĐẢM BẢO HEADER LED MATRIX LUÔN ĐÚNG
  // ------------------------------
  // Nếu đang ở sub-menu Led Matrix
  if (appState == STATE_MENU && currentLevel == LEVEL_MATRIX_SUB) {
    drawMatrixHeader(currentMatrixIndex);
  }
  // Nếu đang chạy Matrix 8x8
  else if (appState == STATE_MATRIX_8X8) {
    drawMatrixHeader(0);
  }
  // Nếu đang chạy Matrix 8x32
  else if (appState == STATE_MATRIX_8X32) {
    drawMatrixHeader(1);
  }
}

// ======================
// Xử lý xoay encoder
// ======================
void onEncoderTurn(int direction) {
  // Chỉ cho phép xoay khi ở MENU
  if (appState != STATE_MENU) return;

  if (currentLevel == LEVEL_MAIN) {
    currentMainIndex += direction;
    if (currentMainIndex < 0) currentMainIndex = MAIN_MENU_COUNT - 1;
    if (currentMainIndex >= MAIN_MENU_COUNT) currentMainIndex = 0;
    printMainMenuItem();
  } else if (currentLevel == LEVEL_I2C_SUB) {
    currentI2CIndex += direction;
    if (currentI2CIndex < 0) currentI2CIndex = I2C_MENU_COUNT - 1;
    if (currentI2CIndex >= I2C_MENU_COUNT) currentI2CIndex = 0;
    printI2CSubMenuItem();
  } else if (currentLevel == LEVEL_MATRIX_SUB) {
    currentMatrixIndex += direction;
    if (currentMatrixIndex < 0) currentMatrixIndex = MATRIX_MENU_COUNT - 1;
    if (currentMatrixIndex >= MATRIX_MENU_COUNT) currentMatrixIndex = 0;
    printMatrixSubMenuItem();   // in lại menu + header Matrix
  }
}

// ======================
// Xử lý nút nhấn
// ======================
void onButtonClick() {
  // 1. Đang ở Analog -> về MENU
  if (appState == STATE_ANALOG) {
    appState = STATE_MENU;
    printMainMenuItem();
    return;
  }

  // 2. Đang ở DFRobot Analog -> về MENU
  if (appState == STATE_DFROBOT_ANALOG) {
    appState = STATE_MENU;
    currentMainIndex = 1;   // DFRobot
    printMainMenuItem();
    return;
  }

  // 3. Đang ở I2C Scan -> về submenu I2C
  if (appState == STATE_I2C_SCAN) {
    appState     = STATE_MENU;
    currentLevel = LEVEL_I2C_SUB;
    printI2CSubMenuItem();
    return;
  }

  // 4. Đang ở Traffic Led -> tắt đèn & về MENU
  if (appState == STATE_TRAFFIC_LED) {
    stopTrafficLedMode();
    appState = STATE_MENU;
    printMainMenuItem();
    return;
  }

  // 5. Đang ở SIMPLE SCREEN -> về menu phù hợp
  if (appState == STATE_SIMPLE_SCREEN) {
    appState = STATE_MENU;
    if (currentLevel == LEVEL_I2C_SUB) {
      printI2CSubMenuItem();
    } else if (currentLevel == LEVEL_MATRIX_SUB) {
      printMatrixSubMenuItem();
    } else {
      printMainMenuItem();
    }
    return;
  }

  // 6. Đang ở NeoPixel -> tắt led & về MENU
  if (appState == STATE_NEOPIXEL) {
    stopNeoPixelMode();        // tắt hết NeoPixel
    appState = STATE_MENU;
    printMainMenuItem();
    return;
  }

  // 7. Đang ở Matrix Led -> nút nhấn để quay về sub-menu Led Matrix
  if (appState == STATE_MATRIX_8X8 || appState == STATE_MATRIX_8X32) {
    stopMatrixLedMode();       // clear + shutdown toàn bộ matrix
    appState     = STATE_MENU;
    currentLevel = LEVEL_MATRIX_SUB;
    printMatrixSubMenuItem();
    return;
  }

  // 8. Đang ở MENU: xử lý chọn chức năng
  if (appState == STATE_MENU) {
    if (currentLevel == LEVEL_MAIN) {
      switch (currentMainIndex) {
        case 0: // I2C
          currentLevel    = LEVEL_I2C_SUB;
          currentI2CIndex = 0;
          strncpy(headerLabel, "I2C Menu", sizeof(headerLabel));
          headerLabel[sizeof(headerLabel) - 1] = '\0';
          printI2CSubMenuItem();
          break;

        case 1: // DFRobot
          startDFRobotAnalogMode();
          break;

        case 5: // Traffic Led
          startTrafficLedMode();
          break;

        case 14: // Analog (ReadAnalog)
          startAnalogMode();
          break;

        case 15: { // Led Matrix (sub-menu)
          currentLevel       = LEVEL_MATRIX_SUB;
          currentMatrixIndex = 0;
          strncpy(headerLabel, "Matrix", sizeof(headerLabel));
          headerLabel[sizeof(headerLabel) - 1] = '\0';
          printMatrixSubMenuItem();
        } break;

        case 6: // Neopixel
          startNeoPixelMode();
          break;

        default:
          // Các mục khác: hiện màn hình đơn giản
          showMainFunctionScreen(currentMainIndex);
          break;
      }
    } else if (currentLevel == LEVEL_I2C_SUB) {
      switch (currentI2CIndex) {
        case 0: // I2C Scan
          startI2CScanMode();
          break;
        case 1: // VL53L1X Test (stub)
          showVL53L1XTestScreen();
          break;
        case 2: // Back
          currentLevel = LEVEL_MAIN;
          strncpy(headerLabel, "Menu", sizeof(headerLabel));
          headerLabel[sizeof(headerLabel) - 1] = '\0';
          printMainMenuItem();
          break;
      }
    } else if (currentLevel == LEVEL_MATRIX_SUB) {
      switch (currentMatrixIndex) {
        case 0: { // Matrix 8x8
          appState = STATE_MATRIX_8X8;
          drawMatrixHeader(0);       // Header: "Matrix 1/2"
          lcdPrintLine(1, "Nhan nut de thoat");
          lcdPrintLine(2, "LED: hieu ung mua");
          lcdPrintLine(3, " ");
          startMatrix8x8Mode();
        } break;

        case 1: { // Matrix 8x32
          appState = STATE_MATRIX_8X32;
          drawMatrixHeader(1);       // Header: "Matrix 2/2"
          lcdPrintLine(1, "Nhan nut de thoat");
          lcdPrintLine(2, "LED: hieu ung mua");
          lcdPrintLine(3, " ");
          startMatrix8x32Mode();
        } break;

        case 2: { // Back
          currentLevel = LEVEL_MAIN;
          strncpy(headerLabel, "Menu", sizeof(headerLabel));
          headerLabel[sizeof(headerLabel) - 1] = '\0';
          printMainMenuItem();
        } break;
      }
    }
  }
}

// ======================
// In sub-menu Led Matrix
// ======================
void printMatrixSubMenuItem() {
  lcd.clear();

  // Header riêng cho Led Matrix: Matrix X/2 (countdown vẫn do updateCountdown() vẽ bên phải)
  drawMatrixHeader(currentMatrixIndex);

  // Dòng 1: tên option hiện tại
  lcdPrintLine(1, matrixSubMenuItems[currentMatrixIndex]);

  // Dòng 2-3: hướng dẫn
  lcdPrintLine(2, "Nhan nut de chon");
  lcdPrintLine(3, "Xoay de doi muc");
}
