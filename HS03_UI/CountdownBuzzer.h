#ifndef COUNTDOWN_BUZZER_H
#define COUNTDOWN_BUZZER_H

// Cập nhật thời gian đếm ngược 120s
void updateCountdown(unsigned long now) {
  if (countdownFinished) return;

  unsigned long elapsed = (now - countdownStartMillis) / 1000UL;
  int remain = (int)COUNTDOWN_SECONDS - (int)elapsed;
  if (remain < 0) remain = 0;

  if (remain != countdownRemaining) {
    countdownRemaining = remain;
    updateHeaderRow();
  }

  if (remain == 0 && !buzzerActive) {
    countdownFinished = true;
    buzzerActive      = true;
    buzzerStartMillis = now;
    buzzerState       = false;
    lastBuzzerToggleMillis = 0;
  }
}

// Điều khiển buzzer bíp bíp 10s
void handleBuzzer(unsigned long now) {
  if (!buzzerActive) return;

  if (now - buzzerStartMillis >= BUZZER_DURATION) {
    buzzerActive = false;
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
