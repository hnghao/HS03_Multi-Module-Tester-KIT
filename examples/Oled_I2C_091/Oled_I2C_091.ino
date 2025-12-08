#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Kích thước OLED 0.91" phổ biến: 128x32
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 32

// Chân I2C dùng cho ESP32-S3 (bạn có thể đổi nếu muốn)
#define SDA_OLED 1
#define SCL_OLED 2

// Địa chỉ I2C của OLED (thường là 0x3C, nếu không được thì thử 0x3D)
#define OLED_I2C_ADDRESS 0x3C

// Không dùng chân RESET riêng -> để -1
#define OLED_RESET -1

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Hàm hiển thị full trắng toàn màn hình
void showFullWhite() {
  display.clearDisplay();
  display.fillRect(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT, SSD1306_WHITE);
  display.display();
}

// Hàm hiển thị chữ "HSHOP" nằm giữa màn hình
void showHSHOPCenter() {
  display.clearDisplay();

  display.setTextSize(2);              // Phóng to font lên x2
  display.setTextColor(SSD1306_WHITE); // Chữ màu trắng trên nền đen

  const char* text = "HSHOP";
  const uint8_t textLen = 5;

  // Với font mặc định, mỗi ký tự rộng 6 pixel (5 + 1 khoảng cách)
  int16_t charWidth  = 6 * 2;  // nhân với hệ số setTextSize(2)
  int16_t charHeight = 8 * 2;  // chiều cao font: 8 pixel * 2

  int16_t textWidth  = charWidth * textLen;
  int16_t textHeight = charHeight;

  // Tính toạ độ để chữ nằm giữa màn hình
  int16_t x = (SCREEN_WIDTH  - textWidth)  / 2;
  int16_t y = (SCREEN_HEIGHT - textHeight) / 2;

  display.setCursor(x, y);
  display.print(text);
  display.display();
}

void setup() {
  Serial.begin(115200);

  // Khởi động I2C với SDA, SCL đã chọn
  Wire.begin(SDA_OLED, SCL_OLED);

  // Khởi động OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_I2C_ADDRESS)) {
    Serial.println(F("Không tìm thấy OLED SSD1306! Kiểm tra dây nối/I2C address."));
    while (true) {
      delay(10);
    }
  }

  display.clearDisplay();
  display.display();
}

void loop() {
  // Bước 1: Hiển thị toàn bộ pixel màn hình (full trắng)
  showFullWhite();
  delay(500);

  // Bước 2: Hiển thị chữ "HSHOP" ở giữa
  showHSHOPCenter();
  delay(500);

  // Quay lại loop -> lặp lại 2 hành động trên
}
