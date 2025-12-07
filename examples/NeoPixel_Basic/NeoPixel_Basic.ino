#include <Adafruit_NeoPixel.h>

// ====== CẤU HÌNH PHẦN CỨNG ======
#define PIN_NEOPIXEL  5     // Chân DIN của dải LED nối với GPIO5 ESP32-S3
#define NUM_LEDS      144   // Số LED tối đa (chỉnh lại theo thực tế nếu cần)

// Tạo đối tượng điều khiển NeoPixel
Adafruit_NeoPixel strip(NUM_LEDS, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

// ====== HÀM HIỆN MỘT MÀU LÊN TOÀN BỘ DẢI LED ======
void showColor(uint32_t color) {
  for (int i = 0; i < NUM_LEDS; i++) {
    strip.setPixelColor(i, color);
  }
  strip.show();
}

void setup() {
  // Khởi động Serial (không bắt buộc, chỉ để debug)
  Serial.begin(115200);
  Serial.println("Bat dau NeoPixel ESP32-S3...");

  // Khởi tạo dải LED
  strip.begin();
  strip.show(); // Tắt toàn bộ LED ban đầu
  strip.setBrightness(80); // Độ sáng (0 - 255), có thể tăng/giảm nếu cần
}

void loop() {
  // Màu ĐỎ
  Serial.println("Mau DO");
  showColor(strip.Color(255, 0, 0));
  delay(1000);

  // Màu XANH LÁ
  Serial.println("Mau XANH LA");
  showColor(strip.Color(0, 255, 0));
  delay(1000);

  // Màu XANH DƯƠNG
  Serial.println("Mau XANH DUONG");
  showColor(strip.Color(0, 0, 255));
  delay(1000);

  // Màu TRẮNG
  Serial.println("Mau TRANG");
  showColor(strip.Color(255, 255, 255));
  delay(1000);
}
