#include <TM1637Display.h>

// Chân kết nối với TM1637
#define CLK_PIN  2    // CLK của TM1637 nối GPIO2 (qua chuyển mức nếu dùng 5V)
#define DIO_PIN  1    // DIO của TM1637 nối GPIO1

TM1637Display display(CLK_PIN, DIO_PIN);

// Thời gian hiển thị cố định: 12:34
const int DISPLAY_TIME = 1234;

bool colonState = false;            // Trạng thái dấu hai chấm
unsigned long lastToggleTime = 0;   // Thời điểm lần cuối đổi trạng thái
const unsigned long TOGGLE_INTERVAL = 1000; // 1000 ms = 1 giây

// Hiệu ứng khởi động: chớp toàn bộ LED 2 lần
void startupBlink() {
  // Dùng uint8_t đúng với prototype của TM1637Display::setSegments
  uint8_t dataOn[]  = {0xFF, 0xFF, 0xFF, 0xFF}; // tất cả segment sáng
  uint8_t dataOff[] = {0x00, 0x00, 0x00, 0x00}; // tắt hết

  for (int i = 0; i < 2; i++) {
    display.setSegments(dataOn);   // Bật tất cả
    delay(200);
    display.setSegments(dataOff);  // Tắt tất cả
    delay(200);
  }
}

void setup() {
  // Độ sáng: 0–7 (0 mờ nhất, 7 sáng nhất)
  display.setBrightness(5);

  // Hiệu ứng khởi động
  startupBlink();

  // Sau khi chớp xong thì hiển thị giờ lần đầu, tắt colon
  display.showNumberDecEx(DISPLAY_TIME, 0x00, true);
}

void loop() {
  unsigned long now = millis();

  // Mỗi 1 giây đổi trạng thái dấu hai chấm
  if (now - lastToggleTime >= TOGGLE_INTERVAL) {
    lastToggleTime = now;
    colonState = !colonState;

    // Bit 6 (0b01000000) điều khiển dấu hai chấm trong TM1637Display
    uint8_t colonMask = colonState ? 0b01000000 : 0x00;

    // Hiển thị số 1234 với colon on/off
    display.showNumberDecEx(DISPLAY_TIME, colonMask, true);
  }
}
