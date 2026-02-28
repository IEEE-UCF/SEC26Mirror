/**
 * @file MinibotMotorDriver.h
 * @brief H-bridge motor driver with truth-table direction + PWM speed control.
 *
 * Each motor channel uses three pins:
 *   - in1_pin / in2_pin: direction truth table (e.g. DRV8833 / L298N)
 *       IN1=H IN2=L → forward
 *       IN1=L IN2=H → reverse
 *       IN1=L IN2=L → coast
 *   - pwm_pin: speed (0–255 analogWrite)
 */
#pragma once

#include <Arduino.h>

namespace Drivers {

class MinibotMotorDriver {
 public:
  MinibotMotorDriver(uint8_t in1_pin, uint8_t in2_pin, uint8_t pwm_pin)
      : in1_pin_(in1_pin), in2_pin_(in2_pin), pwm_pin_(pwm_pin) {}

  bool init() {
    pinMode(in1_pin_, OUTPUT);
    pinMode(in2_pin_, OUTPUT);
    pinMode(pwm_pin_, OUTPUT);
    digitalWrite(in1_pin_, LOW);
    digitalWrite(in2_pin_, LOW);
    analogWrite(pwm_pin_, 0);
    return true;
  }

  /// @param speed -255 (full reverse) to +255 (full forward), 0 = stop
  void setPWM(int speed) { speed_ = constrain(speed, -255, 255); }

  void update() {
    if (speed_ > 0) {
      digitalWrite(in1_pin_, HIGH);
      digitalWrite(in2_pin_, LOW);
      analogWrite(pwm_pin_, speed_);
    } else if (speed_ < 0) {
      digitalWrite(in1_pin_, LOW);
      digitalWrite(in2_pin_, HIGH);
      analogWrite(pwm_pin_, -speed_);
    } else {
      digitalWrite(in1_pin_, LOW);
      digitalWrite(in2_pin_, LOW);
      analogWrite(pwm_pin_, 0);
    }
  }

 private:
  uint8_t in1_pin_;
  uint8_t in2_pin_;
  uint8_t pwm_pin_;
  int speed_ = 0;
};

}  // namespace Drivers
