#ifndef COUNTDOWN_BUZZER_H
#define COUNTDOWN_BUZZER_H

extern bool buzzerEnabled;
extern bool buzzerActive;
extern bool buzzerState;
extern unsigned long lastBuzzerToggleMillis;

// Cập nhật thời gian đếm ngược 120s
void updateCountdown(unsigned long now) {
  // ===== BUZZER OFF: không đếm, luôn giữ 0s =====
  if (!buzzerEnabled) {
    if (countdownRemaining != 0 || !countdownFinished) {
      countdownRemaining = 0;
      countdownFinished  = true;
      updateHeaderRow(); // cập nhật header hiển thị 0s
    }
    buzzerActive = false;
    buzzerState  = false;
    digitalWrite(BUZZER_PIN, LOW);
    return;
  }

  // ===== BUZZER ON: chạy countdown như bình thường =====
  unsigned long elapsed = (now - countdownStartMillis) / 1000UL;
  int remain = (int)COUNTDOWN_SECONDS - (int)elapsed;
  if (remain < 0) remain = 0;

  if (remain != countdownRemaining) {
    countdownRemaining = remain;
    updateHeaderRow();
  }

  // Khi về 0 -> bật còi (handleBuzzer sẽ bíp theo interval)
  if (remain == 0 && !buzzerActive) {
    countdownFinished      = true;
    buzzerActive           = true;
    buzzerState            = false;
    lastBuzzerToggleMillis = 0;
    digitalWrite(BUZZER_PIN, LOW);
  }
}


// Điều khiển buzzer bíp bíp LIÊN TỤC (không giới hạn 10s nữa)
void handleBuzzer(unsigned long now) {
  if (!buzzerEnabled) {
    buzzerActive = false;
    buzzerState  = false;
    digitalWrite(BUZZER_PIN, LOW);
    return;
  }

// Chỉ bíp khi đang được kích hoạt (countdown về 0)
  if (!buzzerActive) {
    buzzerState = false;
    digitalWrite(BUZZER_PIN, LOW);
    return;
  }

  if (now - lastBuzzerToggleMillis >= BUZZER_TOGGLE_INTERVAL) {
    lastBuzzerToggleMillis = now;
    buzzerState = !buzzerState;
    digitalWrite(BUZZER_PIN, buzzerState ? HIGH : LOW);
  }
}

#endif
