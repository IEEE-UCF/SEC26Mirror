/**
 * @file PCA9685Driver.h
 * @brief PCA9685 16-channel PWM driver.
 * @date 2026-02-23 (updated)
 *
 * Hardware: PCA9685 on Wire2.  Two instances (#1 and #2) share Wire2 at
 * different I2C addresses.
 */

#pragma once

#include <Adafruit_PWMServoDriver.h>
#include <Arduino.h>
#include <BaseDriver.h>
#include <Wire.h>

#include "I2CBusLock.h"

namespace Robot {

class PCA9685DriverSetup : public Classes::BaseSetup {
 public:
  /**
   * @param _id      Driver identifier string.
   * @param i2c_addr PCA9685 I2C address (default 0x40).
   * @param freq_hz  PWM frequency in Hz (default 50 for servos).
   * @param wire     I2C bus (default Wire2).
   */
  PCA9685DriverSetup(const char* _id, uint8_t i2c_addr = 0x40,
                     uint16_t freq_hz = 50, TwoWire& wire = Wire2)
      : Classes::BaseSetup(_id),
        i2c_addr_(i2c_addr),
        freq_hz_(freq_hz),
        wire_(wire) {}

  const uint8_t  i2c_addr_;
  const uint16_t freq_hz_;
  TwoWire&       wire_;
};

class PCA9685Driver : public Classes::BaseDriver {
 public:
  explicit PCA9685Driver(const PCA9685DriverSetup& setup);
  ~PCA9685Driver() override = default;

  bool        init()    override;
  const char* getInfo() override { return setup_.getId(); }

  // ── Immediate I2C operations ──────────────────────────────────────────────
  void writeDigital(uint8_t channel, bool on);
  void writePWM(uint8_t channel, uint16_t duty);  // duty: 0..4095

  // ── Buffered operations — applied on applyBuffered() / update() ──────────
  void bufferDigital(uint8_t channel, bool on);
  void bufferPWM(uint8_t channel, uint16_t duty);

  // Called by PCA9685Manager::update() to flush all dirty channels.
  void applyBuffered();
  void update() override;

 private:
  const PCA9685DriverSetup setup_;
  Adafruit_PWMServoDriver  pwm_;
  uint16_t                 buffer_[16];
  bool                     buffer_dirty_[16];
};

}  // namespace Robot
