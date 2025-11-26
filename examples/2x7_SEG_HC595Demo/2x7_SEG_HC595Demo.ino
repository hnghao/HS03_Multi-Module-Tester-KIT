// ====== ESP32-S3 + 2×74HC595 + 2×LED 7 đoạn CA ======
#define PIN_SDI   1   // SDI -> SER(DS) 74HC595
#define PIN_SCLK  2   // SCLK -> SRCLK(SHCP)
#define PIN_LOAD  4   // LOAD/LATCH -> RCLK(STCP)

// Bản đồ bit: Q0..Q7 = A,B,C,D,E,F,G,DP (bit0=A ... bit6=G, bit7=DP)
// Mảng dưới đây là mẫu cho Common-Cathode (CC). Ta sẽ đảo bit để ra Common-Anode (CA).
const uint8_t DIGIT_CC[10] = {
  // 0     1     2     3     4     5     6     7     8     9     (bit7=DP off)
  0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F
};

// Chuyển mẫu CC sang CA, kèm trạng thái DP.
// dpOn = true => bật dấu chấm
inline uint8_t makeByteCA(uint8_t cc, bool dpOn) {
  uint8_t withDp = cc | (dpOn ? 0x80 : 0x00); // CC: 1 là sáng
  return ~withDp;                              // CA: active LOW -> đảo bit
}

// Gửi 16 bit cho 2 x 74HC595 (mỗi con 8 bit)
// Lưu ý: byte shift đầu tiên sẽ đi tới IC ở xa hơn trong chuỗi.
// Ở đây ta coi "left" là LED bên trái, "right" là LED bên phải.
// Nếu lắp ngược, chỉ cần đổi thứ tự hai shiftOut.
void push595(uint8_t leftCA, uint8_t rightCA) {
  digitalWrite(PIN_LOAD, LOW);
  shiftOut(PIN_SDI, PIN_SCLK, MSBFIRST, leftCA);   // sang IC thứ hai (xa hơn)
  shiftOut(PIN_SDI, PIN_SCLK, MSBFIRST, rightCA);  // sang IC thứ nhất (gần hơn)
  digitalWrite(PIN_LOAD, HIGH);                    // chốt dữ liệu lên ngõ ra
}

// Hiển thị cùng một số trên cả 2 LED; blink cả 2 DP cho dễ nhìn.
// Nếu chỉ muốn chớp DP bên phải, đổi dpLeft=false, dpRight=dpBlink.
void showTwoDigitsSame(uint8_t digit, bool dpBlink) {
  if (digit > 9) digit = 0;
  uint8_t cc = DIGIT_CC[digit];
  bool dpLeft  = dpBlink;
  bool dpRight = dpBlink;

  uint8_t leftCA  = makeByteCA(cc, dpLeft);
  uint8_t rightCA = makeByteCA(cc, dpRight);
  push595(leftCA, rightCA);
}

void setup(){
    //=================2 led 7 doan hc595===================
  pinMode(PIN_SDI,  OUTPUT);
  pinMode(PIN_SCLK, OUTPUT);
  pinMode(PIN_LOAD, OUTPUT);
  digitalWrite(PIN_SDI,  LOW);
  digitalWrite(PIN_SCLK, LOW);
  digitalWrite(PIN_LOAD, LOW);

  // Tắt hết lúc khởi động (CA: 1 = tắt)
  push595(0xFF, 0xFF);
}

void loop(){
  static bool dp = false; // trạng thái dấu chấm sẽ đảo sau mỗi lần đổi số

  for (uint8_t d = 0; d <= 9; d++) {
    showTwoDigitsSame(d, dp);
    dp = !dp;            // đổi trạng thái DP sau mỗi số
    delay(700);          // thời gian giữa các số (chỉnh theo ý)
  }
}