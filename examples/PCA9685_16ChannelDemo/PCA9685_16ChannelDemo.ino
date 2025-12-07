// Arduino code for ESP32-S3 + PCA9685 + Rotary Encoder
// Controls 16 servos simultaneously. Startup -> all servos at 90 degrees.
// Rotary encoder: GPIO17 (CLK), GPIO16 (DT), GPIO15 (SW).
// I2C (Wire) on GPIO1 (SDA) and GPIO2 (SCL).

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// --- Pin definitions (the ones you gave) ---
const int PIN_ENCODER_CLK = 17; // CLK
const int PIN_ENCODER_DT  = 16; // DT
const int PIN_ENCODER_SW  = 15; // SW (button)

// I2C pins
const int I2C_SDA = 1;  // SDA
const int I2C_SCL = 2;  // SCL

// PCA9685
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(); // default address 0x40

// Servo pulse lengths (microseconds) - adjust if your servos need other range
const uint16_t SERVO_MIN_US = 500;   // 0 degrees
const uint16_t SERVO_MAX_US = 2500;  // 180 degrees
const uint16_t FREQUENCY_HZ  = 50;   // 50Hz for servos

// runtime variables
volatile int encoderPos = 90;    // current angle (volatile because changed in ISR)
volatile bool encoderChanged = false;
volatile int lastCLKstate = HIGH;

// debounce for switch
unsigned long lastSwitchMillis = 0;
const unsigned long SWITCH_DEBOUNCE_MS = 50;

// helper
int clampAngle(int a) {
  if (a < 0) return 0;
  if (a > 180) return 180;
  return a;
}

// Convert microseconds pulse to PCA9685 tick (0..4095)
// period = 1 / FREQUENCY_HZ -> in microseconds = 1e6 / FREQUENCY_HZ
uint16_t usToTicks(uint16_t us) {
  const uint32_t period_us = 1000000UL / FREQUENCY_HZ; // e.g. 20000us
  uint32_t ticks = (uint32_t)us * 4096UL / period_us;
  if (ticks > 4095) ticks = 4095;
  return (uint16_t)ticks;
}

// write servo angle to a particular PCA channel (0..15)
void writeServoAngle(uint8_t channel, uint8_t angle) {
  // map angle to microsecond pulse
  uint16_t pulse_us = map(angle, 0, 180, SERVO_MIN_US, SERVO_MAX_US);
  uint16_t ticks = usToTicks(pulse_us);
  pwm.setPWM(channel, 0, ticks);
}

// write same angle to all 16 servos
void writeAllServos(int angle) {
  angle = clampAngle(angle);
  for (uint8_t ch = 0; ch < 16; ch++) {
    writeServoAngle(ch, angle);
  }
}

// Encoder ISR - trigger on CHANGE of CLK
void IRAM_ATTR handleEncoder() {
  int clkState = digitalRead(PIN_ENCODER_CLK);
  int dtState  = digitalRead(PIN_ENCODER_DT);

  // Only on rising edge of CLK (optional: use CHANGE for higher resolution)
  if (clkState != lastCLKstate && clkState == LOW) {
    // when CLK falls, check DT to determine direction
    if (dtState != clkState) {
      // clockwise -> increase angle
      if (encoderPos < 180) encoderPos++;
    } else {
      // counter-clockwise -> decrease angle
      if (encoderPos > 0) encoderPos--;
    }
    encoderChanged = true;
  }
  lastCLKstate = clkState;
}

void setup() {
  Serial.begin(115200);
  delay(50);
  Serial.println("ESP32-S3 PCA9685 16-servo with rotary encoder");

  // Init I2C with specified SDA / SCL pins
  Wire.begin(I2C_SDA, I2C_SCL); // SDA, SCL
  // Initialize PCA9685
  pwm.begin();
  pwm.setPWMFreq(FREQUENCY_HZ);
  delay(10);

  // Encoder pins
  pinMode(PIN_ENCODER_CLK, INPUT_PULLUP);
  pinMode(PIN_ENCODER_DT, INPUT_PULLUP);
  pinMode(PIN_ENCODER_SW, INPUT_PULLUP);

  lastCLKstate = digitalRead(PIN_ENCODER_CLK);

  // Attach interrupt on CLK pin (CHANGE recommended); use FALLING to reduce events
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_CLK), handleEncoder, CHANGE);

  // Initial angle 90 deg
  encoderPos = 90;
  writeAllServos(encoderPos);
  Serial.print("Initial angle: ");
  Serial.println(encoderPos);
}

void loop() {
  // If encoder rotated -> update servos
  if (encoderChanged) {
    noInterrupts();
    int angle = encoderPos;
    encoderChanged = false;
    interrupts();

    angle = clampAngle(angle);
    writeAllServos(angle);
    Serial.print("Angle = ");
    Serial.println(angle);
  }

  // Read switch (button press) to reset to 90 degrees
  if (digitalRead(PIN_ENCODER_SW) == LOW) {
    unsigned long now = millis();
    if (now - lastSwitchMillis > SWITCH_DEBOUNCE_MS) {
      lastSwitchMillis = now;
      // reset
      noInterrupts();
      encoderPos = 90;
      encoderChanged = true;
      interrupts();
      Serial.println("Button pressed: reset to 90");
    }
  }

  // small delay to reduce serial spam and cpu
  delay(5);
}
