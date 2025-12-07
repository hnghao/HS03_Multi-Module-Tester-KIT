#ifndef SEGMENT4X7_HC595_MODE_H
#define SEGMENT4X7_HC595_MODE_H

#include <Arduino.h>

// ==========================
// Cấu hình chân & logic phần cứng
// ==========================

// 74HC595 nối với ESP32-S3
#define SEG4X7_PIN_DIO   4   // SER (DS)
#define SEG4X7_PIN_SCLK  1   // SHCP (SRCLK)
#define SEG4X7_PIN_RCLK  2   // STCP (RCLK/LATCH)

// 3641BS là Common Cathode (CC) -> bảng mã CC
#define SEG4X7_LED_IS_CC         1   // để tham khảo

// SEG_ACTIVE_HIGH:
//   1 = segment ON khi xuất '1' từ 74HC595
//   0 = segment ON khi xuất '0' (active LOW)
#define SEG4X7_SEG_ACTIVE_HIGH   0

// DIGIT_ACTIVE_HIGH:
//   1 = chọn digit bằng mức '1'
//   0 = chọn digit bằng mức '0'
#define SEG4X7_DIGIT_ACTIVE_HIGH 1

// 4 digit cơ bản nằm ở Q0..Q3 (board 4 LED 7 đoạn)
#define SEG4X7_DIGIT_BITS_4      0b00001111
// 8 digit: Q0..Q7
#define SEG4X7_DIGIT_BITS_8      0xFF

// ==========================
// Liên kết với LCD & hàm in dòng từ HS03_UI
// ==========================
class LiquidCrystal_I2C;
extern LiquidCrystal_I2C lcd;
extern void lcdPrintLine(uint8_t row, const char* text);

// CHÂN ENCODER dùng #define ENCODER_CLK_PIN / ENCODER_DT_PIN
// đã có trong HS03_UI.ino, không cần extern ở đây.

// ==========================
// Bảng mã số cho CC: bit0..6 = a..g, bit7 = dp
// ==========================
static const uint8_t SEG4X7_DIGIT_CC[10] = {
  0b00111111, // 0
  0b00000110, // 1
  0b01011011, // 2
  0b01001111, // 3
  0b01100110, // 4
  0b01101101, // 5
  0b01111101, // 6
  0b00000111, // 7
  0b01111111, // 8
  0b01101111  // 9
};

// Nếu cần đổi mapping chân a..g..dp thì sửa hàm này
static inline uint8_t SEG4X7_REMAP_SEG(uint8_t x) { return x; }

// Tạo byte segment thực tế cho 1 chữ số + trạng thái dấu chấm
static inline uint8_t seg4x7MakeSegByte(uint8_t d, bool dpOn) {
  if (d > 9) d = 0;
  uint8_t pat = SEG4X7_DIGIT_CC[d];      // CC: 1 = sáng
  if (dpOn) pat |=  (1 << 7);
  else      pat &= ~(1 << 7);

  pat = SEG4X7_REMAP_SEG(pat);

  // Chuyển sang mức vật lý
  return SEG4X7_SEG_ACTIVE_HIGH ? pat : (uint8_t)~pat;
}

// ==========================
// Hàm low-level điều khiển 2×74HC595
// ==========================

static inline void seg4x7ShiftOutByte(uint8_t data) {
  for (int i = 7; i >= 0; --i) {
    digitalWrite(SEG4X7_PIN_SCLK, LOW);
    digitalWrite(SEG4X7_PIN_DIO, (data >> i) & 0x01);
    digitalWrite(SEG4X7_PIN_SCLK, HIGH);
  }
}

static inline void seg4x7WriteDual595(uint8_t segByte, uint8_t digitByte) {
  digitalWrite(SEG4X7_PIN_RCLK, LOW);
  // Shift SEGMENTS trước, rồi DIGITS (giống code mẫu ban đầu)
  seg4x7ShiftOutByte(segByte);
  seg4x7ShiftOutByte(digitByte);
  digitalWrite(SEG4X7_PIN_RCLK, HIGH);
}

// ==========================
// Mode 4 LED / 8 LED + phase chọn / chạy
// ==========================

enum Segment4x7Variant : uint8_t {
  SEG4X7_VARIANT_4LED = 0,
  SEG4X7_VARIANT_8LED = 1
};

enum Segment4x7Phase : uint8_t {
  SEG4X7_PHASE_SELECT = 0,   // Trang chọn 4 LED / 8 LED
  SEG4X7_PHASE_RUN    = 1    // Đang chạy hiệu ứng + chỉnh độ sáng
};

// ==========================
// Biến trạng thái cho mode 4x7
// ==========================
static bool              seg4x7Inited        = false;
static Segment4x7Phase   seg4x7Phase         = SEG4X7_PHASE_SELECT;
static Segment4x7Variant seg4x7Variant       = SEG4X7_VARIANT_4LED;
static uint8_t           seg4x7MenuIndex     = 0;      // 0 = 4 LED, 1 = 8 LED

static uint8_t       seg4x7CurrentDigit  = 0;
static bool          seg4x7DpOn          = false;
static unsigned long seg4x7LastStepMs    = 0;
static const unsigned long SEG4X7_STEP_INTERVAL = 600; // ms

// Encoder (dùng riêng trong mode 4x7)
static int           seg4x7LastEncA      = HIGH;
static unsigned long seg4x7LastEncMove   = 0;
static const unsigned long SEG4X7_ENC_DEBOUNCE  = 5;   // ms

// Độ sáng: 0..10 (0 = tắt, 10 = sáng tối đa)
static int8_t        seg4x7BrightnessLevel      = 8;
static const int8_t  SEG4X7_BRIGHTNESS_MIN      = 0;
static const int8_t  SEG4X7_BRIGHTNESS_MAX      = 10;

// PWM mềm cho toàn bộ dãy 7 đoạn
static const unsigned long SEG4X7_PWM_BASE_MS   = 1;   // 1 ms
static const uint8_t       SEG4X7_PWM_STEPS     = 10;  // 10 mức -> chu kỳ 10ms ~100Hz

// Byte đang "mong muốn hiển thị"
static uint8_t seg4x7SegOnByte   = 0;
static uint8_t seg4x7DigitsOn    = 0;

// Byte tắt hết
static uint8_t seg4x7SegOffByte  = 0;
static uint8_t seg4x7DigitsOff   = 0;

// Tính mask digit bật tùy theo 4/8 LED
static inline uint8_t seg4x7CalcDigitsOnMask() {
  uint8_t baseMask = (seg4x7Variant == SEG4X7_VARIANT_4LED)
                       ? SEG4X7_DIGIT_BITS_4
                       : SEG4X7_DIGIT_BITS_8;
  return SEG4X7_DIGIT_ACTIVE_HIGH ? baseMask : (uint8_t)~baseMask;
}

// ==========================
// Cập nhật LCD
// ==========================

// Trang chọn 4 LED / 8 LED
static inline void seg4x7UpdateLCDSelectPage() {
  lcdPrintLine(1, "4x7 Segment HC595");
  if (seg4x7MenuIndex == 0) {
    lcdPrintLine(2, ">4 LED 7 doan");
    lcdPrintLine(3, " 8 LED 7 doan");
  } else {
    lcdPrintLine(2, " 4 LED 7 doan");
    lcdPrintLine(3, ">8 LED 7 doan");
  }
}

// Trang RUN: hiển thị mode + độ sáng
static inline void seg4x7UpdateLCDRunInfo() {
  lcdPrintLine(1, "4x7 Segment HC595");

  char buf[21];
  if (seg4x7Variant == SEG4X7_VARIANT_4LED) {
    snprintf(buf, sizeof(buf), "Mode: 4 LED 7 doan");
  } else {
    snprintf(buf, sizeof(buf), "Mode: 8 LED 7 doan");
  }
  lcdPrintLine(2, buf);

  int percent = (seg4x7BrightnessLevel * 100) / SEG4X7_BRIGHTNESS_MAX;
  if (seg4x7BrightnessLevel == 0) {
    snprintf(buf, sizeof(buf), "Do sang:   0%% (OFF)");
  } else {
    snprintf(buf, sizeof(buf), "Do sang: %3d%%", percent);
  }
  lcdPrintLine(3, buf);
}

// ==========================
// API: start / update / stop
// ==========================

// Gọi khi vào chức năng "4x7 Segment HC595"
inline void startSegment4x7HC595Mode() {
  // Cấu hình chân
  pinMode(SEG4X7_PIN_DIO,  OUTPUT);
  pinMode(SEG4X7_PIN_SCLK, OUTPUT);
  pinMode(SEG4X7_PIN_RCLK, OUTPUT);
  digitalWrite(SEG4X7_PIN_DIO,  LOW);
  digitalWrite(SEG4X7_PIN_SCLK, LOW);
  digitalWrite(SEG4X7_PIN_RCLK, LOW);

  // Tính byte tắt
  seg4x7SegOffByte = SEG4X7_SEG_ACTIVE_HIGH   ? 0x00 : 0xFF;
  seg4x7DigitsOff  = SEG4X7_DIGIT_ACTIVE_HIGH ? 0x00 : 0xFF;

  // Lúc mới vào, ở TRANG CHỌN -> tắt toàn bộ
  seg4x7WriteDual595(seg4x7SegOffByte, seg4x7DigitsOff);

  seg4x7Inited        = true;
  seg4x7Phase         = SEG4X7_PHASE_SELECT;
  seg4x7Variant       = SEG4X7_VARIANT_4LED;
  seg4x7MenuIndex     = 0;

  seg4x7CurrentDigit  = 0;
  seg4x7DpOn          = false;
  seg4x7LastStepMs    = 0;

  seg4x7BrightnessLevel = 8;

  // Khởi tạo encoder riêng cho mode
  seg4x7LastEncA     = digitalRead(ENCODER_CLK_PIN);
  seg4x7LastEncMove  = millis();

  // LCD: trang chọn 4/8 LED
  seg4x7UpdateLCDSelectPage();
}

// Gọi trong loop() khi appState == STATE_SEGMENT_4X7_HC595
inline void updateSegment4x7HC595Mode(unsigned long now) {
  if (!seg4x7Inited) return;

  // PHASE: CHỌN 4 LED / 8 LED
  if (seg4x7Phase == SEG4X7_PHASE_SELECT) {
    int encA = digitalRead(ENCODER_CLK_PIN);
    if (encA != seg4x7LastEncA && encA == LOW) {  // cạnh xuống
      if (now - seg4x7LastEncMove > SEG4X7_ENC_DEBOUNCE) {
        int encB = digitalRead(ENCODER_DT_PIN);
        int dir  = (encB == HIGH) ? +1 : -1;

        if (dir > 0) seg4x7MenuIndex++;
        else         seg4x7MenuIndex--;

        if (seg4x7MenuIndex > 1) seg4x7MenuIndex = 0;
        // chỉ 0 hoặc 1
        seg4x7UpdateLCDSelectPage();
        seg4x7LastEncMove = now;
      }
    }
    seg4x7LastEncA = encA;

    // Đảm bảo tắt hết LED trong trang chọn
    seg4x7WriteDual595(seg4x7SegOffByte, seg4x7DigitsOff);
    return;
  }

  // PHASE: RUN (đã chọn 4 LED hoặc 8 LED)
  // 1) Đọc encoder để chỉnh ĐỘ SÁNG
  int encA = digitalRead(ENCODER_CLK_PIN);
  if (encA != seg4x7LastEncA && encA == LOW) {  // cạnh xuống
    if (now - seg4x7LastEncMove > SEG4X7_ENC_DEBOUNCE) {
      int encB = digitalRead(ENCODER_DT_PIN);
      int dir  = (encB == HIGH) ? +1 : -1;

      seg4x7BrightnessLevel += dir;
      if (seg4x7BrightnessLevel < SEG4X7_BRIGHTNESS_MIN)
        seg4x7BrightnessLevel = SEG4X7_BRIGHTNESS_MIN;
      if (seg4x7BrightnessLevel > SEG4X7_BRIGHTNESS_MAX)
        seg4x7BrightnessLevel = SEG4X7_BRIGHTNESS_MAX;

      seg4x7UpdateLCDRunInfo();
      seg4x7LastEncMove = now;
    }
  }
  seg4x7LastEncA = encA;

  // 2) Hiệu ứng số: lần lượt hiện 0..9, chớp dp
  if (now - seg4x7LastStepMs >= SEG4X7_STEP_INTERVAL) {
    seg4x7LastStepMs = now;

    seg4x7DpOn          = !seg4x7DpOn;                            // chớp dấu chấm
    seg4x7CurrentDigit  = (seg4x7CurrentDigit + 1) % 10;          // 0..9

    seg4x7SegOnByte     = seg4x7MakeSegByte(seg4x7CurrentDigit, seg4x7DpOn);
  }

  // 3) PWM phần mềm để dim sáng toàn bộ 7 đoạn
  uint8_t level = seg4x7BrightnessLevel;
  if (level <= 0) {
    // Tắt hoàn toàn
    seg4x7WriteDual595(seg4x7SegOffByte, seg4x7DigitsOff);
  } else {
    unsigned long period = SEG4X7_PWM_BASE_MS * SEG4X7_PWM_STEPS; // 10ms
    unsigned long phase  = now % period;
    unsigned long onTime = (unsigned long)SEG4X7_PWM_BASE_MS * level; // 1..10 ms

    // digitsOn phụ thuộc 4/8 LED
    seg4x7DigitsOn = seg4x7CalcDigitsOnMask();

    if (level >= SEG4X7_BRIGHTNESS_MAX) {
      // Sáng tối đa: luôn bật
      seg4x7WriteDual595(seg4x7SegOnByte, seg4x7DigitsOn);
    } else {
      if (phase < onTime) {
        seg4x7WriteDual595(seg4x7SegOnByte, seg4x7DigitsOn);
      } else {
        seg4x7WriteDual595(seg4x7SegOffByte, seg4x7DigitsOff);
      }
    }
  }
}

// Tắt hoàn toàn khi thoát mode (nhấn nút encoder ở RUN)
inline void stopSegment4x7HC595Mode() {
  if (!seg4x7Inited) return;
  seg4x7Inited = false;

  seg4x7WriteDual595(seg4x7SegOffByte, seg4x7DigitsOff);
}

// ==========================
// Xử lý nút nhấn cho mode 4x7
// - Ở trang chọn: nhấn = xác nhận 4/8, chuyển sang RUN, KHÔNG thoát menu
// - Ở RUN: nhấn = yêu cầu thoát về Menu (trả false)
// ==========================
inline bool handle4x7HC595ButtonClick() {
  if (!seg4x7Inited) return false;

  if (seg4x7Phase == SEG4X7_PHASE_SELECT) {
    // Xác nhận lựa chọn 4/8 LED
    seg4x7Variant = (seg4x7MenuIndex == 0)
                      ? SEG4X7_VARIANT_4LED
                      : SEG4X7_VARIANT_8LED;

    // Chuẩn bị chạy
    seg4x7BrightnessLevel = 8;
    seg4x7CurrentDigit    = 0;
    seg4x7DpOn            = false;
    seg4x7LastStepMs      = millis();
    seg4x7SegOnByte       = seg4x7MakeSegByte(seg4x7CurrentDigit, seg4x7DpOn);

    seg4x7Phase           = SEG4X7_PHASE_RUN;
    seg4x7UpdateLCDRunInfo();

    return true;    // ĐÃ xử lý nút, KHÔNG thoát về Menu
  }

  // Đang ở RUN -> trả false để .ino thoát mode
  return false;
}

// ==========================
// Alias để tương thích với HS03_UI.ino hiện tại
// ==========================
inline void startSegment4x7Mode()                 { startSegment4x7HC595Mode(); }
inline void updateSegment4x7Mode(unsigned long n) { updateSegment4x7HC595Mode(n); }
inline void stopSegment4x7Mode()                  { stopSegment4x7HC595Mode(); }

// HS03_UI đang gọi các hàm tên sau:
inline void start4x7HC595Mode()                   { startSegment4x7HC595Mode(); }
inline void update4x7HC595Mode(unsigned long n)   { updateSegment4x7HC595Mode(n); }
inline void stop4x7HC595Mode()                    { stopSegment4x7HC595Mode(); }

#endif // SEGMENT4X7_HC595_MODE_H
