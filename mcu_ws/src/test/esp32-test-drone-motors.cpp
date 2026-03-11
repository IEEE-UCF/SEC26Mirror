/**
 * @file esp32-test-drone-motors.cpp
 * @brief Simple motor ramp test: 0 → 0.8 → 0 over 10s, all 4 motors.
 *        10 second delay before start and between cycles.
 *        No FreeRTOS, no micro-ROS — just bare Arduino loop.
 */

#include <Arduino.h>

// Motor pins (Seeed XIAO ESP32-S3)
constexpr uint8_t MOTOR_FR = D0;
constexpr uint8_t MOTOR_BR = D1;
constexpr uint8_t MOTOR_FL = D2;
constexpr uint8_t MOTOR_BL = D3;

constexpr uint32_t PWM_FREQ = 20000;
constexpr uint8_t PWM_RES = 10;
constexpr uint32_t PWM_MAX = (1 << PWM_RES) - 1;

constexpr float MAX_DUTY = 1.0f;
constexpr uint32_t RAMP_MS = 4000;  // 10s total: 5s up, 5s down

void setAll(float duty) {
  uint32_t val = (uint32_t)(duty * PWM_MAX);
  if (val > PWM_MAX) val = PWM_MAX;
  ledcWrite(0, val);
  ledcWrite(1, val);
  ledcWrite(2, val);
  ledcWrite(3, val);
}

void setup() {
  Serial.begin(115200);

  ledcSetup(0, PWM_FREQ, PWM_RES);
  ledcSetup(1, PWM_FREQ, PWM_RES);
  ledcSetup(2, PWM_FREQ, PWM_RES);
  ledcSetup(3, PWM_FREQ, PWM_RES);
  ledcAttachPin(MOTOR_FL, 0);
  ledcAttachPin(MOTOR_FR, 1);
  ledcAttachPin(MOTOR_BR, 2);
  ledcAttachPin(MOTOR_BL, 3);

  setAll(0);
  Serial.println("Motor ramp test ready. Starting in 10 seconds...");
}

void loop() {
  delay(15000);
  Serial.println("Ramping UP...");

  uint32_t half = RAMP_MS / 2;

  // Ramp up: 0 → MAX_DUTY over 5s
  uint32_t start = millis();
  while (millis() - start < half) {
    float t = (float)(millis() - start) / (float)half;
    setAll(t * MAX_DUTY);
    delay(10);
  }
  setAll(MAX_DUTY);
  Serial.printf("Peak: %.0f%%\n", MAX_DUTY * 100);

  // Ramp down: MAX_DUTY → 0 over 5s
  Serial.println("Ramping DOWN...");
  start = millis();
  while (millis() - start < half) {
    float t = 1.0f - (float)(millis() - start) / (float)half;
    setAll(t * MAX_DUTY);
    delay(10);
  }
  setAll(0);
  Serial.println("Done. Next cycle in 10 seconds...");
}
