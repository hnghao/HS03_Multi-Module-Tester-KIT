/*
  JSN-SR04T-V3.0 Ultrasonic Sensor - Mode 0 Demo
  srt04-mode0.ino
  Uses JSN-SR04T-V3.0 Ultrasonic Sensor
  Displays on Serial Monitor

  Mode 0 is default mode with no jumpers or resistors (emulates HC-SR04)

  DroneBot Workshop 2021
  https://dronebotworkshop.com
*/
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// Define connections to sensor
#define TRIGPIN 14
#define ECHOPIN 9
LiquidCrystal_I2C lcd(0x27, 20, 4);

// Floats to calculate distance
float duration, distance;

void setup() {
  // Set up serial monitor
  Serial.begin(115200);
  Wire.begin(6, 7);
  lcd.init();
  lcd.backlight();
  // Set pinmodes for sensor connections
  pinMode(ECHOPIN, INPUT);
  pinMode(TRIGPIN, OUTPUT);
}

void loop() {

  // Set the trigger pin LOW for 2uS
  digitalWrite(TRIGPIN, LOW);
  delayMicroseconds(2);

  // Set the trigger pin HIGH for 20us to send pulse
  digitalWrite(TRIGPIN, HIGH);
  delayMicroseconds(20);

  // Return the trigger pin to LOW
  digitalWrite(TRIGPIN, LOW);

  // Measure the width of the incoming pulse
  duration = pulseIn(ECHOPIN, HIGH);

  // Determine distance from duration
  // Use 343 metres per second as speed of sound
  // Divide by 1000 as we want millimeters

  distance = (duration / 2) * 0.343;

  // Print result to serial monitor
  // Serial.print("distance: ");
  // Serial.print(distance);
  // Serial.println(" mm");
  lcd.setCursor(0, 0);
  lcd.print("Gia tri khoang cach");
  lcd.setCursor(0, 1);
  lcd.print(distance,1);
  lcd.print("mm");
  // Delay before repeating measurement
  delay(100);
}