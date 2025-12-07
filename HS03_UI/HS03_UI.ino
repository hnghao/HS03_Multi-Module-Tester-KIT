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
#define NEOPIXEL_LED_COUNT 144    // Tối đa 150 led

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
  LEVEL_MATRIX_SUB,
  LEVEL_RS485_SUB        // submenu cho Sensor RS485
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
  STATE_MATRIX_8X32,     // Matrix 8x32
  STATE_RS485_SHTC3,     // Sensor RS485: SHTC3
  STATE_SEGMENT_4X7_HC595,
  STATE_TM1637,           // 4x7 Segment TM1637
  STATE_MAX6675
};

// ======================
// Cấu hình menu
// ======================
// Menu chính
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
  "Test PCA9685",
  "<-- Back"
};

const int I2C_MENU_COUNT = sizeof(i2cSubMenuItems) / sizeof(i2cSubMenuItems[0]);

// Sub-menu Led Matrix (chỉ còn Matrix 8x32 + Back)
const char* const matrixSubMenuItems[] = {
  "Matrix 8x32",
  "<-- Back"
};
const int MATRIX_MENU_COUNT = sizeof(matrixSubMenuItems) / sizeof(matrixSubMenuItems[0]);

// Sub-menu Sensor RS485
const char* const rs485SubMenuItems[] = {
  "1. SHTC3",
  "2. Sensor 2",
  "3. Sensor 3",
  "<-- Back"
};
const int RS485_MENU_COUNT = sizeof(rs485SubMenuItems) / sizeof(rs485SubMenuItems[0]);

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
int currentRS485Index  = 0;    // index cho sub-menu RS485

int  lastClkState     = HIGH;
bool lastBtnState     = HIGH;
unsigned long lastBtnTime     = 0;
unsigned long lastEncoderTime = 0;
const unsigned long BTN_DEBOUNCE      = 200; // ms
const unsigned long ENCODER_DEBOUNCE  = 3;   // ms

char headerLabel[16] = "";

// Cờ bật/tắt vẽ header dòng 0
bool headerEnabled = true;

// Countdown 120s + buzzer 10s
const uint16_t       COUNTDOWN_SECONDS     = 180;
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
void printRS485SubMenuItem();

void drawMatrixHeader(uint8_t funcIndex);
void drawRS485Header(uint8_t funcIndex);

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

    case STATE_MATRIX_8X32:
      updateMatrix8x32Mode();    // mưa rơi 8x32
      break;

    case STATE_RS485_SHTC3:
      updateRS485SHTC3Mode(now); // đọc SHTC3 RS485 & hiển thị LCD
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
  // ĐẢM BẢO HEADER LED MATRIX & RS485 LUÔN ĐÚNG
  // ------------------------------
  if (appState == STATE_MENU) {
    if (currentLevel == LEVEL_MATRIX_SUB) {
      drawMatrixHeader(currentMatrixIndex);
    } else if (currentLevel == LEVEL_RS485_SUB) {
      drawRS485Header(currentRS485Index);
    }
  } else if (appState == STATE_MATRIX_8X32) {
    drawMatrixHeader(0);
  } else if (appState == STATE_RS485_SHTC3) {
    drawRS485Header(0);   // SHTC3 là sensor 1/3
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
  } else if (currentLevel == LEVEL_RS485_SUB) {
    currentRS485Index += direction;
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

  // Đang ở TM1637 -> nhấn 1 lần để thoát về MENU
  if (appState == STATE_TM1637) {
    stopTM1637Mode();
    appState = STATE_MENU;
    printMainMenuItem();
    return;
  }

  // Đang ở Analog -> về MENU
  if (appState == STATE_ANALOG) {
    appState = STATE_MENU;
    printMainMenuItem();
    return;
  }

  // Đang ở DFRobot Analog -> về MENU
  if (appState == STATE_DFROBOT_ANALOG) {
    appState = STATE_MENU;
    currentMainIndex = 1;   // DFRobot
    printMainMenuItem();
    return;
  }

  // Đang ở I2C Scan -> về submenu I2C
  if (appState == STATE_I2C_SCAN) {
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

  // Đang ở SIMPLE SCREEN -> về menu phù hợp
  if (appState == STATE_SIMPLE_SCREEN) {
    appState = STATE_MENU;
    if (currentLevel == LEVEL_I2C_SUB) {
      printI2CSubMenuItem();
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

  // Đang ở 4x7 Segment HC595
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

  // Đang ở MENU: xử lý chọn chức năng
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

        case 4: { // Sensor RS485
          currentLevel      = LEVEL_RS485_SUB;
          currentRS485Index = 0;
          strncpy(headerLabel, "RS485", sizeof(headerLabel));
          headerLabel[sizeof(headerLabel) - 1] = '\0';
          printRS485SubMenuItem();
        } break;

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

        case 6: { // Neopixel
        // Hiển thị trang thông tin NeoPixel trên LCD
        lcd.clear();
        lcdPrintLine(0, "Neopixel");
        lcdPrintLine(1, "Chu ky RGB+Trang");
        lcdPrintLine(2, "Nhan nut de dung");
        lcdPrintLine(3, " ");

        // Chạy mode NeoPixel (blocking, tuong tu TM1637 / 4x7 HC595)
        startNeoPixelMode();

        // Sau khi thoat -> quay ve MENU
        appState     = STATE_MENU;
        currentLevel = LEVEL_MAIN;
        strncpy(headerLabel, "Menu", sizeof(headerLabel));
        headerLabel[sizeof(headerLabel) - 1] = '\0';
        printMainMenuItem();
        } break;

        case 11: { // 4x7 Segment TM1637
        // Không đổi appState sang STATE_TM1637 nữa,
        // TM1637 sẽ tự chạy blocking bên trong startTM1637Mode()

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

        case 10: { // 4x7 Segment HC595
        appState = STATE_SEGMENT_4X7_HC595;

        // Header dòng 0 vẫn do updateHeaderRow() vẽ (Menu + index + countdown)
        strncpy(headerLabel, "4x7 HC595", sizeof(headerLabel));
        headerLabel[sizeof(headerLabel) - 1] = '\0';
        updateHeaderRow();

        // Hướng dẫn trên LCD:
        lcdPrintLine(1, "Nhan nut de thoat");
        lcdPrintLine(2, "Mode: 4/8 LED 7 doan");
        lcdPrintLine(3, "Xoay encoder doi");

        // Khởi động mode 4x7 (khởi tạo chân + tắt an toàn)
        start4x7HC595Mode();
      } break;

        case 9: { // 2x7 Segment HC595
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

        case 8: { // Ultrasonic JSN
        // Hiển thị trang thông tin trên LCD
        lcd.clear();
        lcdPrintLine(0, "Ultrasonic JSN");
        lcdPrintLine(1, "JSN-SR04T Mode 0");
        lcdPrintLine(2, "Nhan nut de thoat");
        lcdPrintLine(3, " ");

        // Chạy mode đo khoảng cách (blocking giống TM1637 / 4x7 / 2x7)
        startUltrasonicJSNMode();

        // Sau khi thoát thì quay lại MENU chính
        appState     = STATE_MENU;
        currentLevel = LEVEL_MAIN;
        strncpy(headerLabel, "Menu", sizeof(headerLabel));
        headerLabel[sizeof(headerLabel) - 1] = '\0';
        printMainMenuItem();
        } break;

        case 12: { // PS2
        // Chạy mode PS2 (blocking), hiển thị LCD đúng như chương trình gốc
        startPS2Mode();

        // Sau khi thoát PS2 -> quay lại MENU chính
        appState     = STATE_MENU;
        currentLevel = LEVEL_MAIN;
        strncpy(headerLabel, "Menu", sizeof(headerLabel));
        headerLabel[sizeof(headerLabel) - 1] = '\0';
        printMainMenuItem();
        } break;

        case 7: { // Max6675  (nếu index khác thì sửa số 7 cho đúng)
          appState = STATE_MAX6675;
          strncpy(headerLabel, "Max6675", sizeof(headerLabel));
          headerLabel[sizeof(headerLabel) - 1] = '\0';
          updateHeaderRow();

          // Dòng 1–3 do mode tự xử lý
          startMAX6675Mode();
        } break;

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
        case 1: { // Test PCA9685
        // Chạy chương trình test PCA9685 (blocking, tự xử lý countdown + buzzer)
        startPCA9685TestMode();

        // Sau khi thoát test, quay lại submenu I2C như cũ
        strncpy(headerLabel, "I2C Menu", sizeof(headerLabel));
        headerLabel[sizeof(headerLabel) - 1] = '\0';
        printI2CSubMenuItem();
        } break;
        case 2: // Back
          currentLevel = LEVEL_MAIN;
          strncpy(headerLabel, "Menu", sizeof(headerLabel));
          headerLabel[sizeof(headerLabel) - 1] = '\0';
          printMainMenuItem();
          break;
      }
    } else if (currentLevel == LEVEL_MATRIX_SUB) {
      switch (currentMatrixIndex) {
        case 0: { // Matrix 8x32
          appState = STATE_MATRIX_8X32;
          drawMatrixHeader(0);       // Header: "Matrix 1/1"
          lcdPrintLine(1, "Nhan nut de thoat");
          lcdPrintLine(2, "LED: hieu ung mua");
          lcdPrintLine(3, " ");
          startMatrix8x32Mode();
        } break;

        case 1: { // Back
          currentLevel = LEVEL_MAIN;
          strncpy(headerLabel, "Menu", sizeof(headerLabel));
          headerLabel[sizeof(headerLabel) - 1] = '\0';
          printMainMenuItem();
        } break;
      }
    } else if (currentLevel == LEVEL_RS485_SUB) {
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
    }
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
