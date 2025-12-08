#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Kích thước OLED 0.96" phổ biến: 128x64
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 64

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

// Hàm hiển thị chữ "HSHOP" nằm giữa màn hình (dùng getTextBounds để canh giữa chính xác)
void showHSHOPCenter() {
  display.clearDisplay();

  display.setTextSize(2);              // Phóng to font lên x2
  display.setTextColor(SSD1306_WHITE); // Chữ màu trắng

  const char *text = "HSHOP";

  // Tính kích thước thực của chữ để canh giữa
  int16_t x1, y1;
  uint16_t w, h;

  // getTextBounds(text, x, y, &x1, &y1, &w, &h);
  // x, y ở đây là vị trí giả định ban đầu (0,0)
  display.getTextBounds(text, 0, 0, &x1, &y1, &w, &h);

  // Tính toạ độ để chữ nằm giữa màn hình
  int16_t x = (SCREEN_WIDTH  - w) / 2;
  int16_t y = (SCREEN_HEIGHT - h) / 2;

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

  // Lặp lại
}
