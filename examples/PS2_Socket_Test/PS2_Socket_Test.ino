#include <PS2X_lib.h>          // Thư viện PS2
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <string.h>

//================= CẤU HÌNH CHÂN PS2 =================
#define PS2_CLK 2
#define PS2_ATT 1
#define PS2_CMD 4
#define PS2_DAT 5

//================= CẤU HÌNH LCD2004 I2C =============
#define I2C_SDA   6      // Chân SDA cho ESP32-S3
#define I2C_SCL   7      // Chân SCL cho ESP32-S3
#define LCD_ADDR  0x27   // Địa chỉ I2C của LCD (0x27 hoặc 0x3F)

#define LCD_COLS 20
#define LCD_ROWS 4

PS2X ps2x; // PS2 Controller
LiquidCrystal_I2C lcd(LCD_ADDR, LCD_COLS, LCD_ROWS);

//================= BIẾN TOÀN CỤC =====================
int  error   = 0; 
byte type    = 0;
byte vibrate = 0;

// Cache nội dung 4 dòng LCD để tránh nháy
char prevLcd[4][LCD_COLS + 1] = { "", "", "", "" };

unsigned long lastLcdUpdate   = 0;
const unsigned long LCD_UPDATE_INTERVAL = 100;   // ms

// Cờ chế độ hiển thị: true = L1/R1, false = L2/R2
bool showR1L1 = true;

//====================================================
// Viết 1 dòng lên LCD, chỉ update khi khác nội dung cũ
//====================================================
void lcdWriteLine(uint8_t row, const char* text) {
  if (row >= LCD_ROWS) return;

  char buf[LCD_COLS + 1];
  // Cắt chuỗi tối đa 20 ký tự
  strncpy(buf, text, LCD_COLS);
  buf[LCD_COLS] = '\0';

  // Nếu giống dòng cũ => không update để đỡ nháy
  if (strncmp(prevLcd[row], buf, LCD_COLS) == 0) return;

  lcd.setCursor(0, row);
  lcd.print("                    "); // 20 khoảng trắng
  lcd.setCursor(0, row);
  lcd.print(buf);

  strncpy(prevLcd[row], buf, LCD_COLS + 1);
}

//====================================================
// DualShock / Unknown - MỘT TRANG ĐẦY THÔNG TIN
// - Bình thường: hiển thị L1/R1
// - Khi nhấn L2/R2: chuyển sang hiển thị L2/R2
// - Khi nhấn lại L1/R1: trở về L1/R1
//====================================================
void updateDualShockLCD_OnePage() {
  char line[21];

  const char* typeStr;
  switch (type) {
    case 1:  typeStr = "DS"; break;      // DualShock
    case 0:  typeStr = "UN"; break;      // Unknown
    case 2:  typeStr = "GH"; break;      // GuitarHero (phòng hờ)
    default: typeStr = "OT"; break;      // Other
  }

  // Dòng 0: Loại tay cầm + Vibration + Blue analog (X)
  uint8_t blueAnalog = ps2x.Analog(PSAB_BLUE);
  // Ví dụ: "T:DS V:100 B:255"
  snprintf(line, sizeof(line), "T:%-2s V:%3d B:%3d", typeStr, vibrate, blueAnalog);
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
  const char* rLabel = showR1L1 ? "R1" : "R2";
  const char* lLabel = showR1L1 ? "L1" : "L2";

  int rVal = showR1L1 ? (ps2x.Button(PSB_R1) ? 1 : 0)
                      : (ps2x.Button(PSB_R2) ? 1 : 0);

  int lVal = showR1L1 ? (ps2x.Button(PSB_L1) ? 1 : 0)
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
  int LX = ps2x.Analog(PSS_LX);
  int LY = ps2x.Analog(PSS_LY);

  // Ví dụ: "LX:123 LY:045 L1:1" hoặc "L2:1"
  snprintf(line, sizeof(line), "LX:%3d LY:%3d %s:%d", LX, LY, lLabel, lVal);
  lcdWriteLine(3, line);
}

//====================================================
// Guitar Hero Controller (type == 2) - MỘT TRANG
//====================================================
void updateGuitarHeroLCD() {
  char line[21];

  snprintf(line, sizeof(line), "PS2:GuitarHero V:%3d", vibrate);
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
// SETUP
//====================================================
void setup() {
  Serial.begin(57600);

  // Khởi động I2C và LCD
  Wire.begin(I2C_SDA, I2C_SCL);
  lcd.init();
  lcd.backlight();
  lcd.clear();

  lcdWriteLine(0, "ESP32-S3 + PS2");
  lcdWriteLine(1, "Khoi tao tay cam");
  delay(500);

  // Cấu hình tay cầm
  error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_ATT, PS2_DAT, true, true);

  if (error == 0) {
    Serial.println("Found Controller, configured successful");
    type = ps2x.readType();

    lcdWriteLine(2, "Controller OK");

    if (type == 0) {
      lcdWriteLine(3, "Type: Unknown");
      Serial.println("Unknown Controller type");
    } else if (type == 1) {
      lcdWriteLine(3, "Type: DualShock");
      Serial.println("DualShock Controller Found");
    } else if (type == 2) {
      lcdWriteLine(3, "Type: GuitarHero");
      Serial.println("GuitarHero Controller Found");
    } else {
      lcdWriteLine(3, "Type: Other");
    }

  } else {
    // Hiển thị lỗi lên Serial + LCD
    if (error == 1) {
      Serial.println("No controller found, check wiring");
      lcdWriteLine(2, "Err1: No controller");
    } else if (error == 2) {
      Serial.println("Controller found but not accepting commands");
      lcdWriteLine(2, "Err2: No command");
    } else if (error == 3) {
      Serial.println("Controller refusing Pressures mode");
      lcdWriteLine(2, "Err3: No Pressures");
    }
  }

  delay(1000);
}

//====================================================
// LOOP
//====================================================
void loop() {
  // Nếu không tìm thấy tay cầm thì chỉ hiển thị lỗi
  if (error != 0) {
    delay(500);
    return;
  }

  // Đọc tay cầm + rung (dựa trên độ nhấn nút X)
  vibrate = ps2x.Analog(PSAB_BLUE);
  ps2x.read_gamepad(false, vibrate);

  // ====== LOGIC CHUYỂN CHẾ ĐỘ HIỂN THỊ L1/R1 <-> L2/R2 ======
  // Khi nhấn L2 hoặc R2 -> chuyển sang hiển thị L2/R2
  if (ps2x.ButtonPressed(PSB_L2) || ps2x.ButtonPressed(PSB_R2)) {
    showR1L1 = false;
  }
  // Khi nhấn L1 hoặc R1 -> quay về hiển thị L1/R1
  if (ps2x.ButtonPressed(PSB_L1) || ps2x.ButtonPressed(PSB_R1)) {
    showR1L1 = true;
  }
  // ==========================================================

  unsigned long now = millis();

  // Cập nhật LCD định kỳ (không quá nhanh để tránh nháy)
  if (now - lastLcdUpdate >= LCD_UPDATE_INTERVAL) {
    lastLcdUpdate = now;

    if (type == 2) {
      // Guitar Hero view
      updateGuitarHeroLCD();
    } else {
      // DualShock / Unknown -> 1 trang đầy đủ
      updateDualShockLCD_OnePage();
    }
  }

  delay(10);
}
