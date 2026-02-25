/**
 * @file MinibotMotorDriver.h
 * @brief Simple H-bridge motor driver for minibot (e.g. L298N, DRV8833).
 *
 * Each motor has two PWM pins:
 *   - fwd_pin (A1/B1): PWM high drives forward
 *   - rev_pin (A2/B2): PWM high drives reverse
 *
 * Both low = coast, both high = brake.
 */
#pragma once

#include <Arduino.h>

namespace Drivers {

class MinibotMotorDriver {
 public:
  MinibotMotorDriver(uint8_t fwd_pin, uint8_t rev_pin)
      : fwd_pin_(fwd_pin), rev_pin_(rev_pin) {}

  bool init() {
    pinMode(fwd_pin_, OUTPUT);
    pinMode(rev_pin_, OUTPUT);
    analogWrite(fwd_pin_, 0);
    analogWrite(rev_pin_, 0);
    return true;
  }

  /// @param speed -255 (full reverse) to +255 (full forward), 0 = stop
  void setPWM(int speed) {
    speed_ = constrain(speed, -255, 255);
  }

  void update() {
    if (speed_ > 0) {
      analogWrite(fwd_pin_, speed_);
      analogWrite(rev_pin_, 0);
    } else if (speed_ < 0) {
      analogWrite(fwd_pin_, 0);
      analogWrite(rev_pin_, -speed_);
    } else {
      analogWrite(fwd_pin_, 0);
      analogWrite(rev_pin_, 0);
    }
  }

 private:
  uint8_t fwd_pin_;
  uint8_t rev_pin_;
  int speed_ = 0;
};

}  // namespace Drivers
