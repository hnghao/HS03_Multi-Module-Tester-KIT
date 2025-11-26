//---------------- Cấu hình chân ESP32-S3 ----------------
#define PIN_DIO   4   // SER (DS)
#define PIN_SCLK  1   // SHCP (SRCLK)
#define PIN_RCLK  2   // STCP (RCLK/LATCH)

//---------------- Cấu hình logic phần cứng ----------------
// 3641BS là Common Cathode (CC) -> giữ LED_IS_CC = 1 để dùng bảng mã CC.
#define LED_IS_CC             1   // để tham khảo; ta dùng bảng mã CC ở dưới

// SEG_ACTIVE_HIGH:
//   1 = segment ON khi xuất mức '1' từ 74HC595 (trực tiếp nối CC chuẩn)
//   0 = segment ON khi xuất mức '0' (đường segment dùng transistor/ULN, tức active-LOW)
// Dựa theo triệu chứng của bạn, để mặc định = 0 cho đúng ngay:
#define SEG_ACTIVE_HIGH       0

// DIGIT_ACTIVE_HIGH:
//   1 = chọn digit bằng mức '1'
//   0 = chọn digit bằng mức '0' (ví dụ qua ULN/PNP làm active-LOW)
#define DIGIT_ACTIVE_HIGH     1

// 4 bit chọn LED nằm ở Q0..Q3 (nếu bạn đấu ở Q4..Q7 thì đổi thành 0b11110000)
#define DIGIT_BITS            0b00001111

// Thứ tự đổ byte trong chuỗi 2 x 74HC595:
// 1 = đổ SEGMENTS trước, rồi DIGITS (IC_SEG ở xa MCU)
// 0 = đổ DIGITS trước, rồi SEGMENTS
#define CHAIN_ORDER_SEGMENTS_FIRST  1

// Nếu dây a..g..dp không đúng Q0..Q7 theo thứ tự, remap ở đây:
static inline uint8_t REMAP_SEG(uint8_t x) { return x; }

//---------------- Bảng số (Common Cathode): a..g ở bit0..6, dp ở bit7 ----
const uint8_t DIGIT_CC[10] = {
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
// Quy ước bit: [dp g f e d c b a] = [7..0]

//---------------- Tiện ích tạo byte xuất thực tế ------------------------
static inline uint8_t makeSegByte(uint8_t d, bool dpOn) {
  uint8_t pat = DIGIT_CC[d];            // logic ON = 1 (chuẩn CC)
  if (dpOn) pat |=  (1 << 7);           // bật dp
  else      pat &= ~(1 << 7);           // tắt dp
  pat = REMAP_SEG(pat);                 // nếu cần đổi chân

  // Ánh xạ ra vật lý theo mức kích thật sự của phần cứng
  return SEG_ACTIVE_HIGH ? pat : (uint8_t)~pat;
}

static inline uint8_t allDigitsMask() {
  uint8_t base = DIGIT_BITS;            // ví dụ 0000 1111 (bật 4 digit)
  return DIGIT_ACTIVE_HIGH ? base : (uint8_t)~base;
}

//---------------- Hàm shift và ghi đôi 595 -------------------------------
static inline void shiftOutByte(uint8_t data) {
  for (int i = 7; i >= 0; --i) {
    digitalWrite(PIN_SCLK, LOW);
    digitalWrite(PIN_DIO, (data >> i) & 0x01);
    digitalWrite(PIN_SCLK, HIGH);
  }
}

static inline void writeDual595(uint8_t segByte, uint8_t digitByte) {
  digitalWrite(PIN_RCLK, LOW);
#if CHAIN_ORDER_SEGMENTS_FIRST
  shiftOutByte(segByte);
  shiftOutByte(digitByte);
#else
  shiftOutByte(digitByte);
  shiftOutByte(segByte);
#endif
  digitalWrite(PIN_RCLK, HIGH);
}

void setup(){
  //=================4 led 7 doan hc595===================
  pinMode(PIN_DIO, OUTPUT);
  pinMode(PIN_SCLK, OUTPUT);
  pinMode(PIN_RCLK, OUTPUT);
  digitalWrite(PIN_DIO, LOW);
  digitalWrite(PIN_SCLK, LOW);
  digitalWrite(PIN_RCLK, LOW);

  // Đưa về trạng thái tắt an toàn
  const uint8_t SEG_OFF   = SEG_ACTIVE_HIGH   ? 0x00 : 0xFF;
  const uint8_t DIGITS_OFF= DIGIT_ACTIVE_HIGH ? 0x00 : 0xFF;
  writeDual595(SEG_OFF, DIGITS_OFF);
}

void loop(){
  static bool dpOn = false;
  const uint8_t digitsOn = allDigitsMask();

  for (uint8_t d = 0; d <= 9; d++) {
    uint8_t seg = makeSegByte(d, dpOn);

    // Hiển thị đồng thời: bật tất cả digit và nạp cùng 1 byte segment
    writeDual595(seg, digitsOn);

    delay(600);      // thời gian mỗi số
    dpOn = !dpOn;    // chớp dp sau mỗi lần chuyển số
  }
}