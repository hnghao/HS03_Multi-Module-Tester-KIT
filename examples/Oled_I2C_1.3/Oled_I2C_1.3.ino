#include <Wire.h>
#include <U8g2lib.h>

// Chân I2C dùng cho ESP32-S3
#define SDA_OLED 1
#define SCL_OLED 2

// Tạo đối tượng U8g2 cho SH1106 128x64 I2C (màn 1.3")
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(
  U8G2_R0,          // Không xoay
  U8X8_PIN_NONE     // Không dùng chân RESET riêng
);

// Hàm hiển thị full trắng toàn màn hình
void showFullWhite() {
  u8g2.clearBuffer();
  u8g2.drawBox(0, 0, 128, 64);   // Vẽ box trắng phủ toàn màn
  u8g2.sendBuffer();
}

// Hàm hiển thị chữ "HSHOP" nằm giữa màn hình
void showHSHOPCenter() {
  u8g2.clearBuffer();

  // Chọn font – bạn có thể đổi font khác cho đẹp hơn
  u8g2.setFont(u8g2_font_ncenB14_tr); // Font chữ to, dễ đọc

  const char *text = "HSHOP";

  // Tính độ rộng và cao của chữ để canh giữa
  uint16_t textWidth  = u8g2.getUTF8Width(text);
  int16_t  ascent     = u8g2.getAscent();
  int16_t  descent    = u8g2.getDescent();
  uint16_t textHeight = ascent - descent;

  // Toạ độ để chữ nằm giữa 128x64
  int16_t x = (128 - (int16_t)textWidth) / 2;
  int16_t y = (64 + textHeight) / 2 - 1; // canh theo baseline

  u8g2.setCursor(x, y);
  u8g2.print(text);

  u8g2.sendBuffer();
}

void setup() {
  Serial.begin(115200);

  // Khởi động I2C với chân tùy chọn
  Wire.begin(SDA_OLED, SCL_OLED);

  // (Nếu đường I2C dài/nhiễu, có thể giảm clock xuống 100kHz)
  Wire.setClock(100000); // 100 kHz

  // Khởi động U8g2
  u8g2.begin();
}

void loop() {
  // Bước 1: full trắng
  showFullWhite();
  delay(500);

  // Bước 2: chữ "HSHOP" ở giữa
  showHSHOPCenter();
  delay(500);
}
