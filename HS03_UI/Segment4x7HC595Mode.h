#ifndef SEGMENT4X7_HC595_MODE_H
#define SEGMENT4X7_HC595_MODE_H

#include <Arduino.h>
#include <Wire.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// ==========================
// Cấu hình chân & logic phần cứng (GIỮ NGUYÊN)
// ==========================
#define SEG4X7_PIN_DIO   4   // SER (DS)
#define SEG4X7_PIN_SCLK  1   // SHCP (SRCLK)
#define SEG4X7_PIN_RCLK  2   // STCP (RCLK/LATCH)

#define SEG4X7_LED_IS_CC         1
#define SEG4X7_SEG_ACTIVE_HIGH   0  // active LOW
#define SEG4X7_DIGIT_ACTIVE_HIGH 1

// ==========================
// Link LCD & I2CScanBus từ project (GIỮ NGUYÊN)
// ==========================
class LiquidCrystal_I2C;
extern LiquidCrystal_I2C lcd;
extern void lcdPrintLine(uint8_t row, const char* text);
extern TwoWire I2CScanBus;

// Encoder pins từ HS03_UI.ino
#ifndef ENCODER_CLK_PIN
#define ENCODER_CLK_PIN 17
#endif
#ifndef ENCODER_DT_PIN
#define ENCODER_DT_PIN 16
#endif

// ==========================
// Bảng mã số CC: bit0..6=a..g, bit7=dp
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

static inline uint8_t SEG4X7_REMAP_SEG(uint8_t x) { return x; }

// idx=0 là hàng đơn vị (bên phải), idx=1 là hàng chục...
static inline uint8_t seg4x7MapDigitIndex(uint8_t idx, uint8_t digitsCount) {
  (void)digitsCount;
  return idx;
}

static inline uint8_t seg4x7MakeSegByte(uint8_t d, bool dpOn) {
  if (d > 9) d = 0;
  uint8_t pat = SEG4X7_DIGIT_CC[d];
  if (dpOn) pat |= 0x80; else pat &= ~0x80;

  pat = SEG4X7_REMAP_SEG(pat);
  return SEG4X7_SEG_ACTIVE_HIGH ? pat : (uint8_t)~pat;
}

// ==========================
// Low-level 2×74HC595
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
  seg4x7ShiftOutByte(segByte);
  seg4x7ShiftOutByte(digitByte);
  digitalWrite(SEG4X7_PIN_RCLK, HIGH);
}

// ==========================
// Mode state (GIỮ NGUYÊN)
// ==========================
enum Segment4x7Variant : uint8_t { SEG4X7_VARIANT_4LED = 0, SEG4X7_VARIANT_8LED = 1 };
enum Segment4x7Phase   : uint8_t { SEG4X7_PHASE_SELECT = 0, SEG4X7_PHASE_RUN  = 1 };

static bool              seg4x7Inited      = false;
static Segment4x7Phase   seg4x7Phase       = SEG4X7_PHASE_SELECT;
static Segment4x7Variant seg4x7Variant     = SEG4X7_VARIANT_4LED;
static uint8_t           seg4x7MenuIndex   = 0;

// Counter + DP
static uint32_t      seg4x7Counter     = 0;
static unsigned long seg4x7LastCountMs = 0;
static unsigned long seg4x7LastDpMs    = 0;

// Intro 3s
static bool          seg4x7IntroActive  = false;
static unsigned long seg4x7IntroStartMs = 0;
static const unsigned long SEG4X7_INTRO_DURATION_MS = 3000;

// ==========================
// TỐC ĐỘ ĐẾM (CHẬM LẠI GẤP ĐÔI)
// ==========================
static const unsigned long SEG4X7_COUNT_INTERVAL_4_MS = 100; // trước: 50
static const unsigned long SEG4X7_COUNT_INTERVAL_8_MS = 2;   // trước: 1
static const unsigned long SEG4X7_DP_INTERVAL_MS      = 1000;

// Encoder
static int           seg4x7LastEncA    = HIGH;
static unsigned long seg4x7LastEncMove = 0;
static const unsigned long SEG4X7_ENC_DEBOUNCE = 5;

// Brightness 0..10 (GIỮ NGUYÊN)
static int8_t        seg4x7BrightnessLevel = 8;
static const int8_t  SEG4X7_BRIGHTNESS_MIN = 0;
static const int8_t  SEG4X7_BRIGHTNESS_MAX = 10;

// ==========================
// Refresh TASK: sửa để KHÔNG làm đứng loop
// - Pin sang core 0
// - Có vTaskDelay() để nhường CPU cho loopTask
// ==========================
static TaskHandle_t seg4x7TaskHandle = nullptr;

static volatile uint8_t seg4x7DigitsBuf[8] = {0}; // idx0 = hàng đơn vị
static volatile bool    seg4x7DpOnV        = false;
static volatile bool    seg4x7AllOnV       = false;
static volatile bool    seg4x7ForceOffV    = true;
static volatile uint8_t seg4x7DigitsCountV = 4;
static volatile uint8_t seg4x7BrightnessV  = 8;

static uint8_t seg4x7SegOffByte = 0;
static uint8_t seg4x7DigitsOff  = 0;

static const uint16_t SEG4X7_SCAN_DELAY_US = 250; // quét nhanh, nhưng vẫn ổn
static const uint8_t  SEG4X7_PWM_STEPS     = 10;  // tương ứng 0..10

static inline uint8_t seg4x7ValidMaskFromCount(uint8_t digitsCount) {
  return (digitsCount <= 4) ? 0x0F : 0xFF;
}

static inline uint8_t seg4x7MakeDigitSelectByte(uint8_t idx, uint8_t digitsCount) {
  uint8_t mapped = seg4x7MapDigitIndex(idx, digitsCount);
  uint8_t raw    = (uint8_t)(1u << mapped);
  uint8_t mask   = seg4x7ValidMaskFromCount(digitsCount);

  if (SEG4X7_DIGIT_ACTIVE_HIGH) {
    return raw & mask;
  } else {
    uint8_t out = (uint8_t)(~raw) & mask;
    out |= (uint8_t)(~mask);
    return out;
  }
}

static void seg4x7RefreshTask(void* pv) {
  (void)pv;
  uint8_t pwmPhase = 0;

  for (;;) {
    bool forceOff   = seg4x7ForceOffV;
    uint8_t bright  = seg4x7BrightnessV;
    uint8_t count   = seg4x7DigitsCountV;
    if (count < 1) count = 4;
    if (count > 8) count = 8;

    if (forceOff || bright == 0) {
      seg4x7WriteDual595(seg4x7SegOffByte, seg4x7DigitsOff);
      vTaskDelay(pdMS_TO_TICKS(10)); // nhường CPU rõ ràng
      continue;
    }

    // PWM duty 0..10
    uint8_t duty = bright;
    if (duty > SEG4X7_PWM_STEPS) duty = SEG4X7_PWM_STEPS;
    bool pwmOn = (bright >= SEG4X7_BRIGHTNESS_MAX) || (pwmPhase < duty);

    // Quét 1 frame
    for (uint8_t pos = 0; pos < count; pos++) {
      if (!pwmOn) {
        seg4x7WriteDual595(seg4x7SegOffByte, seg4x7DigitsOff);
      } else {
        bool allOn = seg4x7AllOnV;
        bool dpOn  = seg4x7DpOnV;

        uint8_t d   = allOn ? 8 : seg4x7DigitsBuf[pos];
        bool dpThis = allOn ? true : dpOn;

        uint8_t segByte   = seg4x7MakeSegByte(d, dpThis);
        uint8_t digitByte = seg4x7MakeDigitSelectByte(pos, count);

        seg4x7WriteDual595(segByte, digitByte);
      }

      delayMicroseconds(SEG4X7_SCAN_DELAY_US);
    }

    // next PWM phase
    pwmPhase++;
    if (pwmPhase >= SEG4X7_PWM_STEPS) pwmPhase = 0;

    // QUAN TRỌNG: nhường CPU cho loopTask => tránh đứng LCD / đứng chương trình
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

// ==========================
// Helpers
// ==========================
static inline uint8_t seg4x7DigitsCount() {
  return (seg4x7Variant == SEG4X7_VARIANT_4LED) ? 4 : 8;
}

static inline uint32_t seg4x7MaxValue() {
  return (seg4x7Variant == SEG4X7_VARIANT_4LED) ? 1000u : 1000000u;
}

static inline unsigned long seg4x7CountIntervalMs() {
  return (seg4x7Variant == SEG4X7_VARIANT_4LED) ? SEG4X7_COUNT_INTERVAL_4_MS : SEG4X7_COUNT_INTERVAL_8_MS;
}

static inline void seg4x7UpdateDigitsBufferFromCounter(uint32_t value, uint8_t digitsCount) {
  for (uint8_t i = 0; i < digitsCount; i++) {
    seg4x7DigitsBuf[i] = (uint8_t)(value % 10);
    value /= 10;
  }
}

// ==========================
// LCD pages (GIỮ NGUYÊN)
// ==========================
static inline void seg4x7UpdateLCDSelectPage() {
  lcdPrintLine(1, "4/8x7 Segment HC595");
  if (seg4x7MenuIndex == 0) {
    lcdPrintLine(2, ">4 LED 7 doan");
    lcdPrintLine(3, " 8 LED 7 doan");
  } else {
    lcdPrintLine(2, " 4 LED 7 doan");
    lcdPrintLine(3, ">8 LED 7 doan");
  }
}

static inline void seg4x7UpdateLCDRunInfo() {
  lcdPrintLine(1, "4/8x7 Segment HC595");

  char buf[21];
  if (seg4x7Variant == SEG4X7_VARIANT_4LED) snprintf(buf, sizeof(buf), "Mode: 4 LED 7 doan");
  else                                      snprintf(buf, sizeof(buf), "Mode: 8 LED 7 doan");
  lcdPrintLine(2, buf);

  int percent = (seg4x7BrightnessLevel * 100) / SEG4X7_BRIGHTNESS_MAX;
  if (seg4x7BrightnessLevel == 0) snprintf(buf, sizeof(buf), "Do sang:   0%% (OFF)");
  else                            snprintf(buf, sizeof(buf), "Do sang: %3d%%", percent);
  lcdPrintLine(3, buf);
}

// ==========================
// API start/update/stop + button click (GIỮ LOGIC)
// ==========================
inline void startSegment4x7HC595Mode() {
  I2CScanBus.end();

  pinMode(SEG4X7_PIN_DIO,  OUTPUT);
  pinMode(SEG4X7_PIN_SCLK, OUTPUT);
  pinMode(SEG4X7_PIN_RCLK, OUTPUT);
  digitalWrite(SEG4X7_PIN_DIO,  LOW);
  digitalWrite(SEG4X7_PIN_SCLK, LOW);
  digitalWrite(SEG4X7_PIN_RCLK, LOW);

  seg4x7SegOffByte = SEG4X7_SEG_ACTIVE_HIGH   ? 0x00 : 0xFF;
  seg4x7DigitsOff  = SEG4X7_DIGIT_ACTIVE_HIGH ? 0x00 : 0xFF;

  seg4x7Inited = true;
  seg4x7Phase  = SEG4X7_PHASE_SELECT;
  seg4x7Variant = SEG4X7_VARIANT_4LED;
  seg4x7MenuIndex = 0;

  seg4x7Counter = 0;
  seg4x7LastCountMs = millis();
  seg4x7LastDpMs    = millis();

  seg4x7IntroActive  = false;
  seg4x7IntroStartMs = 0;

  seg4x7BrightnessLevel = 8;
  seg4x7BrightnessV = (uint8_t)seg4x7BrightnessLevel;

  seg4x7LastEncA    = digitalRead(ENCODER_CLK_PIN);
  seg4x7LastEncMove = millis();

  seg4x7DigitsCountV = 4;
  seg4x7DpOnV        = false;
  seg4x7AllOnV       = false;
  seg4x7ForceOffV    = true;
  seg4x7UpdateDigitsBufferFromCounter(0, 4);

  // Tạo task quét: pin CORE 0 để không “đè” loopTask
  if (seg4x7TaskHandle == nullptr) {
    xTaskCreatePinnedToCore(
      seg4x7RefreshTask,
      "seg4x7_refresh",
      3072,
      nullptr,
      1,            // priority = 1 (không cao hơn loopTask)
      &seg4x7TaskHandle,
      0             // core 0
    );
  }

  seg4x7UpdateLCDSelectPage();
}

inline void updateSegment4x7HC595Mode(unsigned long now) {
  if (!seg4x7Inited) return;

  if (seg4x7Phase == SEG4X7_PHASE_SELECT) {
    seg4x7ForceOffV = true;
    seg4x7AllOnV    = false;

    int encA = digitalRead(ENCODER_CLK_PIN);
    if (encA != seg4x7LastEncA && encA == LOW) {
      if (now - seg4x7LastEncMove > SEG4X7_ENC_DEBOUNCE) {
        int encB = digitalRead(ENCODER_DT_PIN);
        int dir  = (encB == HIGH) ? +1 : -1;

        if (dir > 0) seg4x7MenuIndex++;
        else         seg4x7MenuIndex--;

        if (seg4x7MenuIndex > 1) seg4x7MenuIndex = 0;
        seg4x7UpdateLCDSelectPage();
        seg4x7LastEncMove = now;
      }
    }
    seg4x7LastEncA = encA;
    return;
  }

  // RUN
  seg4x7ForceOffV    = false;
  seg4x7DigitsCountV = seg4x7DigitsCount();

  // Encoder brightness
  int encA = digitalRead(ENCODER_CLK_PIN);
  if (encA != seg4x7LastEncA && encA == LOW) {
    if (now - seg4x7LastEncMove > SEG4X7_ENC_DEBOUNCE) {
      int encB = digitalRead(ENCODER_DT_PIN);
      int dir  = (encB == HIGH) ? +1 : -1;

      seg4x7BrightnessLevel += dir;
      if (seg4x7BrightnessLevel < SEG4X7_BRIGHTNESS_MIN) seg4x7BrightnessLevel = SEG4X7_BRIGHTNESS_MIN;
      if (seg4x7BrightnessLevel > SEG4X7_BRIGHTNESS_MAX) seg4x7BrightnessLevel = SEG4X7_BRIGHTNESS_MAX;

      seg4x7BrightnessV = (uint8_t)seg4x7BrightnessLevel;
      seg4x7UpdateLCDRunInfo();
      seg4x7LastEncMove = now;
    }
  }
  seg4x7LastEncA = encA;

  // Intro 3s
  if (seg4x7IntroActive) {
    seg4x7AllOnV = true;
    if (now - seg4x7IntroStartMs >= SEG4X7_INTRO_DURATION_MS) {
      seg4x7IntroActive = false;
      seg4x7AllOnV      = false;

      seg4x7Counter     = 0;
      seg4x7LastCountMs = now;
      seg4x7LastDpMs    = now;
      seg4x7DpOnV       = false;

      seg4x7UpdateDigitsBufferFromCounter(seg4x7Counter, seg4x7DigitsCount());
    }
    return;
  }

  // DP blink 1s
  if (now - seg4x7LastDpMs >= SEG4X7_DP_INTERVAL_MS) {
    seg4x7LastDpMs += SEG4X7_DP_INTERVAL_MS;
    seg4x7DpOnV = !seg4x7DpOnV;
  }

  // Counter chậm lại gấp đôi
  unsigned long interval = seg4x7CountIntervalMs();
  if (interval < 1) interval = 1;

  // catchup nhẹ để tránh giật
  uint8_t catchup = 0;
  while ((unsigned long)(now - seg4x7LastCountMs) >= interval && catchup < 5) {
    seg4x7LastCountMs += interval;
    seg4x7Counter++;
    if (seg4x7Counter > seg4x7MaxValue()) seg4x7Counter = 0;
    catchup++;
  }

  seg4x7UpdateDigitsBufferFromCounter(seg4x7Counter, seg4x7DigitsCount());
}

inline void stopSegment4x7HC595Mode() {
  if (!seg4x7Inited) return;
  seg4x7Inited = false;

  // tắt display trước
  seg4x7ForceOffV = true;
  seg4x7AllOnV    = false;
  seg4x7DpOnV     = false;

  if (seg4x7TaskHandle) {
    vTaskDelete(seg4x7TaskHandle);
    seg4x7TaskHandle = nullptr;
  }

  seg4x7WriteDual595(seg4x7SegOffByte, seg4x7DigitsOff);

  // restore I2CScanBus
  I2CScanBus.begin(I2C_SCAN_SDA_PIN, I2C_SCAN_SCL_PIN, 100000);
}

inline bool handle4x7HC595ButtonClick() {
  if (!seg4x7Inited) return false;

  if (seg4x7Phase == SEG4X7_PHASE_SELECT) {
    seg4x7Variant = (seg4x7MenuIndex == 0) ? SEG4X7_VARIANT_4LED : SEG4X7_VARIANT_8LED;

    seg4x7BrightnessLevel = 8;
    seg4x7BrightnessV     = (uint8_t)seg4x7BrightnessLevel;

    seg4x7Counter     = 0;
    seg4x7LastCountMs = millis();
    seg4x7LastDpMs    = millis();
    seg4x7DpOnV       = false;

    seg4x7IntroActive  = true;
    seg4x7IntroStartMs = millis();
    seg4x7AllOnV       = true;

    seg4x7DigitsCountV = seg4x7DigitsCount();
    seg4x7UpdateDigitsBufferFromCounter(0, seg4x7DigitsCount());

    seg4x7Phase    = SEG4X7_PHASE_RUN;
    seg4x7ForceOffV = false;

    seg4x7UpdateLCDRunInfo();
    return true;
  }

  return false;
}

// ==========================
// Alias tương thích HS03_UI.ino
// ==========================
inline void start4x7HC595Mode()                 { startSegment4x7HC595Mode(); }
inline void update4x7HC595Mode(unsigned long n) { updateSegment4x7HC595Mode(n); }
inline void stop4x7HC595Mode()                  { stopSegment4x7HC595Mode(); }

inline void startSegment4x7Mode()                 { startSegment4x7HC595Mode(); }
inline void updateSegment4x7Mode(unsigned long n) { updateSegment4x7HC595Mode(n); }
inline void stopSegment4x7Mode()                  { stopSegment4x7HC595Mode(); }

#endif // SEGMENT4X7_HC595_MODE_H
