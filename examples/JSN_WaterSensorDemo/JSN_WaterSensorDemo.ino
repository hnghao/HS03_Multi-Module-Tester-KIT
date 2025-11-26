// ESP32-S3 + JSN/HC-SR04T Mode 0 (HC-SR04 style)
// TRIG: GPIO2, ECHO: GPIO4 (ECHO phải hạ áp xuống 3.3V trước khi vào ESP32-S3)

#define TRIG_PIN  14
#define ECHO_PIN  2

// Hàm đo 1 lần, trả về khoảng cách (cm)
// Nếu không bắt được echo -> trả về -1
float readDistanceCmOnce() {
  // Đảm bảo TRIG ở mức LOW
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);

  // Xung kích: HIGH ít nhất 10 µs
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Đo độ rộng xung ECHO (µs), timeout 30ms ~ 5m
  unsigned long duration = pulseIn(ECHO_PIN, HIGH, 30000UL);

  if (duration == 0) {
    // Hết thời gian mà không nhận được echo
    return -1.0;
  }

  // Công thức chuẩn HC-SR04: distance(cm) = duration / 58.0
  float distanceCm = duration / 58.0;
  return distanceCm;
}

// Đọc nhiều lần để lọc nhiễu, trả về trung bình
float readDistanceCm(uint8_t samples = 5) {
  float sum = 0;
  uint8_t validCount = 0;

  for (uint8_t i = 0; i < samples; i++) {
    float d = readDistanceCmOnce();
    if (d > 0) {  // chỉ cộng các giá trị hợp lệ
      sum += d;
      validCount++;
    }
    delay(10); // nghỉ nhẹ giữa các lần đo
  }

  if (validCount == 0) {
    return -1.0;
  }

  return sum / validCount;
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT); // ECHO đã được hạ áp xuống 3.3V

  Serial.println(F("ESP32-S3 + JSN/HC-SR04T Mode 0"));
  Serial.println(F("Dang bat dau do khoang cach..."));
}

void loop() {
  float distance = readDistanceCm(5);

  if (distance < 0) {
    Serial.println(F("Khong nhan duoc echo (Out of range hoặc wiring sai)!"));
  } else {
    Serial.print(F("Khoang cach: "));
    Serial.print(distance, 1); // 1 số lẻ
    Serial.println(F(" cm"));
  }

  delay(500); // đo mỗi 0.5 giây
}
