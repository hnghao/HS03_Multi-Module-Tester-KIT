#ifndef TRAFFIC_LED_MODE_H
#define TRAFFIC_LED_MODE_H

int            trafficState       = 0;
unsigned long  lastTrafficChange  = 0;

// Bắt đầu chế độ đèn giao thông
void startTrafficLedMode() {
  appState = STATE_TRAFFIC_LED;
  lcd.clear();

  strncpy(headerLabel, "Traffic Led", sizeof(headerLabel));
  headerLabel[sizeof(headerLabel) - 1] = '\0';
  updateHeaderRow();

  pinMode(LED_RED_PIN, OUTPUT);
  pinMode(LED_YELLOW_PIN, OUTPUT);
  pinMode(LED_GREEN_PIN, OUTPUT);

  digitalWrite(LED_RED_PIN, LOW);
  digitalWrite(LED_YELLOW_PIN, LOW);
  digitalWrite(LED_GREEN_PIN, LOW);

  trafficState      = 0;
  lastTrafficChange = millis();

  // Bật đèn đỏ trước
  digitalWrite(LED_RED_PIN, HIGH);
  lcdPrintLine(1, "RED   -> ON ");
  lcdPrintLine(2, "YELLOW-> OFF");
  lcdPrintLine(3, "GREEN -> OFF");
}

// Cập nhật chu kỳ đèn mỗi 1s
void updateTrafficLedMode(unsigned long now) {
  if (now - lastTrafficChange < 1000UL) return;
  lastTrafficChange = now;

  trafficState = (trafficState + 1) % 3;

  switch (trafficState) {
    case 0: // Đỏ
      digitalWrite(LED_RED_PIN, HIGH);
      digitalWrite(LED_YELLOW_PIN, LOW);
      digitalWrite(LED_GREEN_PIN, LOW);
      lcdPrintLine(1, "RED   -> ON ");
      lcdPrintLine(2, "YELLOW-> OFF");
      lcdPrintLine(3, "GREEN -> OFF");
      break;

    case 1: // Vàng
      digitalWrite(LED_RED_PIN, LOW);
      digitalWrite(LED_YELLOW_PIN, HIGH);
      digitalWrite(LED_GREEN_PIN, LOW);
      lcdPrintLine(1, "RED   -> OFF");
      lcdPrintLine(2, "YELLOW-> ON ");
      lcdPrintLine(3, "GREEN -> OFF");
      break;

    case 2: // Xanh
      digitalWrite(LED_RED_PIN, LOW);
      digitalWrite(LED_YELLOW_PIN, LOW);
      digitalWrite(LED_GREEN_PIN, HIGH);
      lcdPrintLine(1, "RED   -> OFF");
      lcdPrintLine(2, "YELLOW-> OFF");
      lcdPrintLine(3, "GREEN -> ON ");
      break;
  }
}

// Thoát chế độ Traffic Led: tắt hết đèn
void stopTrafficLedMode() {
  digitalWrite(LED_RED_PIN, LOW);
  digitalWrite(LED_YELLOW_PIN, LOW);
  digitalWrite(LED_GREEN_PIN, LOW);
}

#endif
