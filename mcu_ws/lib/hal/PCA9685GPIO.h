/**
 * @file PCA9685GPIO.h
 * @brief HAL implementation for PCA9685 PWM driver channels
 * @author Claude Code
 * @date 12/24/2025
 */

#pragma once

#include "HALPin.h"
#include "robot/PCA9685Driver.h"

namespace HAL {

/**
 * @brief HAL pin implementation for PCA9685 PWM driver
 *
 * Provides PWM output through PCA9685 channels.
 * Supports digitalWrite (full on/off) and analogWrite (PWM).
 * Does NOT support digitalRead or analogRead.
 */
class PCA9685GPIO : public HALPin {
 public:
  /**
   * @brief Construct a PCA9685 GPIO pin
   * @param driver Pointer to PCA9685 driver instance
   * @param channel Channel number on PCA9685 (0-15)
   */
  PCA9685GPIO(Robot::PCA9685Driver* driver, uint8_t channel)
      : driver_(driver), channel_(channel) {}

  ~PCA9685GPIO() override = default;

  void pinMode(PinMode mode) override {
    // PCA9685 channels are always outputs, pinMode is a no-op
    (void)mode;
  }

  void digitalWrite(DigitalState state) override {
    if (!driver_) return;
    driver_->writeDigital(channel_, state == DigitalState::STATE_HIGH);
  }

  DigitalState digitalRead() override {
    // PCA9685 does not support reading
    return DigitalState::STATE_LOW;
  }

  void analogWrite(uint16_t value) override {
    if (!driver_) return;
    // PCA9685 uses 12-bit PWM (0-4095)
    // Clamp to 12-bit range
    uint16_t duty = value > 4095 ? 4095 : value;
    driver_->writePWM(channel_, duty);
  }

  uint16_t analogRead() override {
    // PCA9685 does not support reading
    return 0;
  }

  const char* getInfo() override {
    snprintf(infoBuffer_, sizeof(infoBuffer_), "PCA9685 ch %d", channel_);
    return infoBuffer_;
  }

  bool supportsDigitalRead() const override { return false; }
  bool supportsAnalogWrite() const override { return true; }
  bool supportsAnalogRead() const override { return false; }

  /**
   * @brief Get the channel number
   * @return PCA9685 channel number (0-15)
   */
  uint8_t getChannel() const { return channel_; }

 private:
  Robot::PCA9685Driver* driver_;
  const uint8_t channel_;
  char infoBuffer_[32];
};

}  // namespace HAL
