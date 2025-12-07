#ifndef PS2_MODE_H
#define PS2_MODE_H

#include <Arduino.h>
#include <PS2X_lib.h>          // Thư viện PS2
#include <LiquidCrystal_I2C.h>
#include <string.h>

// ================== DÙNG LẠI LCD2004 CỦA HS03_UI ==================
// Trong HS03_UI.ino đã có:
//   LiquidCrystal_I2C lcd(LCD_I2C_ADDR, 20, 4);
//   Wire.begin(LCD_SDA_PIN, LCD_SCL_PIN);
extern LiquidCrystal_I2C lcd;

// Countdown + buzzer của HS03_UI
extern void updateCountdown(unsigned long now);
extern void handleBuzzer(unsigned long now);

// Cờ bật/tắt header dòng 0
extern bool headerEnabled;

// Nút nhấn Encoder để thoát
// ENCODER_SW_PIN đã #define trong HS03_UI.ino (15)

// ================== CẤU HÌNH CHÂN PS2 =================
#define PS2_CLK 2
#define PS2_ATT 1
#define PS2_CMD 4
#define PS2_DAT 5

// ================== KÍCH THƯỚC LCD ====================
static const uint8_t PS2_LCD_COLS = 20;
static const uint8_t PS2_LCD_ROWS = 4;

// ================== ĐỐI TƯỢNG VÀ BIẾN TOÀN CỤC ==============
static PS2X ps2x; // PS2 Controller

static int  ps2_error   = 0;
static byte ps2_type    = 0;
static byte ps2_vibrate = 0;

// Cache nội dung 4 dòng LCD để tránh nháy (giống code gốc)
static char ps2_prevLcd[4][PS2_LCD_COLS + 1] = { "", "", "", "" };

static unsigned long ps2_lastLcdUpdate   = 0;
static const unsigned long PS2_LCD_UPDATE_INTERVAL = 100;   // ms

// Cờ chế độ hiển thị: true = L1/R1, false = L2/R2
static bool ps2_showR1L1 = true;
// true = hiển thị joystick trái (LX/LY)
// false = hiển thị joystick phải (RX/RY)
bool showLeftStick = true;
//====================================================
// Viết 1 dòng lên LCD, chỉ update khi khác nội dung cũ
// (giữ NGUYÊN logic & nội dung hiển thị như code gốc)
//====================================================
inline void lcdWriteLine(uint8_t row, const char* text) {
  if (row >= PS2_LCD_ROWS) return;

  char buf[PS2_LCD_COLS + 1];
  // Cắt chuỗi tối đa 20 ký tự
  strncpy(buf, text, PS2_LCD_COLS);
  buf[PS2_LCD_COLS] = '\0';

  // Nếu giống dòng cũ => không update để đỡ nháy
  if (strncmp(ps2_prevLcd[row], buf, PS2_LCD_COLS) == 0) return;

  lcd.setCursor(0, row);
  lcd.print("                    "); // 20 khoảng trắng
  lcd.setCursor(0, row);
  lcd.print(buf);

  strncpy(ps2_prevLcd[row], buf, PS2_LCD_COLS + 1);
}

//====================================================
// DualShock / Unknown - MỘT TRANG ĐẦY THÔNG TIN
// - Bình thường: hiển thị L1/R1
// - Khi nhấn L2/R2: chuyển sang hiển thị L2/R2
// - Khi nhấn lại L1/R1: trở về L1/R1
//====================================================
inline void updateDualShockLCD_OnePage() {
  char line[21];

  const char* typeStr;
  switch (ps2_type) {
    case 1:  typeStr = "DS"; break;      // DualShock
    case 0:  typeStr = "UN"; break;      // Unknown
    case 2:  typeStr = "GH"; break;      // GuitarHero (phòng hờ)
    default: typeStr = "OT"; break;      // Other
  }

  // Dòng 0: Loại tay cầm + Vibration + Blue analog (X)
  uint8_t blueAnalog = ps2x.Analog(PSAB_BLUE);
  // Ví dụ: "T:DS V:100 B:255"
  snprintf(line, sizeof(line), "T:%-2s V:%3d B:%3d", typeStr, ps2_vibrate, blueAnalog);
  lcdWriteLine(0, line);

  // Dòng 1: D-pad + Start / Select
  char u = ps2x.Button(PSB_PAD_UP)    ? 'U' : '.';
  char d = ps2x.Button(PSB_PAD_DOWN)  ? 'D' : '.';
  char l = ps2x.Button(PSB_PAD_LEFT)  ? 'L' : '.';
  char r = ps2x.Button(PSB_PAD_RIGHT) ? 'R' : '.';

  // Ví dụ: "UDLR:UDLR ST:1 SE:0"
  snprintf(line, sizeof(line),
           "UDLR:%c%c%c%c ST:%d SE:%d",
           u, d, l, r,
           ps2x.Button(PSB_START)  ? 1 : 0,
           ps2x.Button(PSB_SELECT) ? 1 : 0);
  lcdWriteLine(1, line);

  // Chuẩn bị giá trị hiển thị cho R* và L*
  const char* rLabel = ps2_showR1L1 ? "R1" : "R2";
  const char* lLabel = ps2_showR1L1 ? "L1" : "L2";

  int rVal = ps2_showR1L1 ? (ps2x.Button(PSB_R1) ? 1 : 0)
                          : (ps2x.Button(PSB_R2) ? 1 : 0);

  int lVal = ps2_showR1L1 ? (ps2x.Button(PSB_L1) ? 1 : 0)
                          : (ps2x.Button(PSB_L2) ? 1 : 0);

  // Dòng 2: Nút mặt + Rx (R1 hoặc R2)
  // Hoa = đang nhấn, thường = thả
  char xChar = ps2x.Button(PSB_BLUE)  ? 'X' : 'x'; // X
  char oChar = ps2x.Button(PSB_RED)   ? 'O' : 'o'; // Circle
  char tChar = ps2x.Button(PSB_GREEN) ? 'T' : 't'; // Triangle
  char sChar = ps2x.Button(PSB_PINK)  ? 'S' : 's'; // Square

  // Ví dụ: "X:x O:o T:t S:s R1:1" hoặc "R2:1"
  snprintf(line, sizeof(line),
           "X:%c O:%c T:%c S:%c %s:%d",
           xChar, oChar, tChar, sChar, rLabel, rVal);
  lcdWriteLine(2, line);

  // Dòng 3: Joystick trái + Lx (L1 hoặc L2)
  // Dòng 3: Joystick (trái hoặc phải) + Lx (L1 hoặc L2)
  int LX = ps2x.Analog(PSS_LX);
  int LY = ps2x.Analog(PSS_LY);
  int RX = ps2x.Analog(PSS_RX);
  int RY = ps2x.Analog(PSS_RY);

  // Chọn stick nào sẽ hiển thị
  int showX = showLeftStick ? LX : RX;
  int showY = showLeftStick ? LY : RY;

  const char* axisXLabel = showLeftStick ? "LX" : "RX";
  const char* axisYLabel = showLeftStick ? "LY" : "RY";

  // Ví dụ:
  //  - Mode mặc định: "LX:123 LY:045 L1:1"
  //  - Nhấn R3 -> "RX:200 RY:050 L1:1"
  snprintf(line, sizeof(line),
           "%s:%3d %s:%3d %s:%d",
           axisXLabel, showX,
           axisYLabel, showY,
           lLabel, lVal);
  lcdWriteLine(3, line);
}

//====================================================
// Guitar Hero Controller (type == 2) - MỘT TRANG
//====================================================
inline void updateGuitarHeroLCD() {
  char line[21];

  snprintf(line, sizeof(line), "PS2:GuitarHero V:%3d", ps2_vibrate);
  lcdWriteLine(0, line);

  char g = ps2x.Button(GREEN_FRET)  ? 'G' : '.';
  char r = ps2x.Button(RED_FRET)    ? 'R' : '.';
  char y = ps2x.Button(YELLOW_FRET) ? 'Y' : '.';
  char b = ps2x.Button(BLUE_FRET)   ? 'B' : '.';
  char o = ps2x.Button(ORANGE_FRET) ? 'O' : '.';

  // Dòng 1: trạng thái 5 phím Fret
  snprintf(line, sizeof(line), "Fret:%c%c%c%c%c", g, r, y, b, o);
  lcdWriteLine(1, line);

  // Dòng 2: Strum, Star Power, Start/Select
  char up = ps2x.Button(UP_STRUM)   ? '^' : '.';
  char dn = ps2x.Button(DOWN_STRUM) ? 'v' : '.';

  snprintf(line, sizeof(line), "Str:%c %c SP:%d ST:%d",
           up, dn,
           ps2x.Button(STAR_POWER) ? 1 : 0,
           ps2x.Button(PSB_START)  ? 1 : 0);
  lcdWriteLine(2, line);

  // Dòng 3: Whammy + Select
  snprintf(line, sizeof(line), "Whammy:%3d SE:%d",
           ps2x.Analog(WHAMMY_BAR),
           ps2x.Button(PSB_SELECT) ? 1 : 0);
  lcdWriteLine(3, line);
}

//====================================================
// MODE PS2 – GỌI THAY CHO setup()/loop() GỐC
//====================================================
inline void startPS2Mode() {
  // Tắt header "Menu + ... + countdown" để dành toàn bộ 4 dòng cho PS2
  bool oldHeaderEnabled = headerEnabled;
  headerEnabled = false;

  // Reset cache LCD cho mode PS2
  for (uint8_t r = 0; r < PS2_LCD_ROWS; r++) {
    memset(ps2_prevLcd[r], 0, PS2_LCD_COLS + 1);
  }
  ps2_lastLcdUpdate = 0;
  ps2_showR1L1      = true;

  lcd.clear();

  // Giữ NGUYÊN nội dung hiển thị như code gốc
  lcdWriteLine(0, "ESP32-S3 + PS2");
  lcdWriteLine(1, "Khoi tao tay cam");
  delay(500);

  // Cấu hình tay cầm
  ps2_error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_ATT, PS2_DAT, true, true);

  if (ps2_error == 0) {
    Serial.println("Found Controller, configured successful");
    ps2_type = ps2x.readType();

    lcdWriteLine(2, "Controller OK");

    if (ps2_type == 0) {
      lcdWriteLine(3, "Type: Unknown");
      Serial.println("Unknown Controller type");
    } else if (ps2_type == 1) {
      lcdWriteLine(3, "Type: DualShock");
      Serial.println("DualShock Controller Found");
    } else if (ps2_type == 2) {
      lcdWriteLine(3, "Type: GuitarHero");
      Serial.println("GuitarHero Controller Found");
    } else {
      lcdWriteLine(3, "Type: Other");
    }

  } else {
    // Hiển thị lỗi lên Serial + LCD (giữ nguyên text)
    if (ps2_error == 1) {
      Serial.println("No controller found, check wiring");
      lcdWriteLine(2, "Err1: No controller");
    } else if (ps2_error == 2) {
      Serial.println("Controller found but not accepting commands");
      lcdWriteLine(2, "Err2: No command");
    } else if (ps2_error == 3) {
      Serial.println("Controller refusing Pressures mode");
      lcdWriteLine(2, "Err3: No Pressures");
    }
  }

  delay(1000);

  // ===================== VÒNG LẶP CHÍNH CỦA MODE PS2 =====================
  bool exitRequested = false;

  while (!exitRequested) {
    unsigned long now = millis();
    updateCountdown(now);
    handleBuzzer(now);

    // Nếu không tìm thấy tay cầm thì chỉ hiển thị lỗi
    if (ps2_error != 0) {
      // Giữ đúng hành vi gốc: chỉ chờ, không update thêm
      // nhưng vẫn cho thoát bằng nút encoder
    } else {
      // Đọc tay cầm + rung (dựa trên độ nhấn nút X)
      ps2_vibrate = ps2x.Analog(PSAB_BLUE);
      ps2x.read_gamepad(false, ps2_vibrate);

      // ====== LOGIC CHUYỂN CHẾ ĐỘ HIỂN THỊ L1/R1 <-> L2/R2 ======
      // Khi nhấn L2 hoặc R2 -> chuyển sang hiển thị L2/R2
      if (ps2x.ButtonPressed(PSB_L2) || ps2x.ButtonPressed(PSB_R2)) {
        ps2_showR1L1 = false;
      }
      // Khi nhấn L1 hoặc R1 -> quay về hiển thị L1/R1
      if (ps2x.ButtonPressed(PSB_L1) || ps2x.ButtonPressed(PSB_R1)) {
        ps2_showR1L1 = true;
      }
        // ====== LOGIC CHUYỂN HIỂN THỊ JOYSTICK TRÁI (LX/LY) <-> PHẢI (RX/RY) ======
      // Nhấn R3 (ấn joystick phải) -> hiển thị RX/RY
      if (ps2x.ButtonPressed(PSB_R3)) {
        showLeftStick = false;
      }
      // Nhấn L3 (ấn joystick trái) -> quay về hiển thị LX/LY
      if (ps2x.ButtonPressed(PSB_L3)) {
        showLeftStick = true;
      }
      // ==========================================================

      // Cập nhật LCD định kỳ (không quá nhanh để tránh nháy)
      if (now - ps2_lastLcdUpdate >= PS2_LCD_UPDATE_INTERVAL) {
        ps2_lastLcdUpdate = now;

        if (ps2_type == 2) {
          // Guitar Hero view
          updateGuitarHeroLCD();
        } else {
          // DualShock / Unknown -> 1 trang đầy đủ
          updateDualShockLCD_OnePage();
        }
      }
    }

    // Nhấn nút encoder để thoát về Menu
    if (digitalRead(ENCODER_SW_PIN) == LOW) {
      delay(30); // debounce
      if (digitalRead(ENCODER_SW_PIN) == LOW) {
        exitRequested = true;
      }
    }

    delay(10); // giống code gốc, vừa phải, tránh chiếm CPU 100%
  }

  // Chờ nhả nút hoàn toàn (tránh dính nhấn khi quay về menu)
  while (digitalRead(ENCODER_SW_PIN) == LOW) {
    unsigned long now2 = millis();
    updateCountdown(now2);
    handleBuzzer(now2);
    delay(10);
  }
  
  // Khôi phục lại trạng thái header như trước khi vào PS2
  headerEnabled = oldHeaderEnabled;
}

#endif // PS2_MODE_H
