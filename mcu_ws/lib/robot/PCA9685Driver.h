#pragma once

#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>
#include <BaseDriver.h>

namespace Robot {

class PCA9685DriverSetup : public Classes::BaseSetup {
 public:
  PCA9685DriverSetup(const char* _id, uint8_t i2c_addr = 0x40, uint16_t freq_hz = 50)
      : Classes::BaseSetup(_id), i2c_addr_(i2c_addr), freq_hz_(freq_hz) {}
  const uint8_t i2c_addr_;
  const uint16_t freq_hz_;
};

class PCA9685Driver : public Classes::BaseDriver {
 public:
  explicit PCA9685Driver(const PCA9685DriverSetup& setup);
  ~PCA9685Driver() override = default;

  bool init() override;
  const char* getInfo() override { static const char info[] = "PCA9685Driver"; return info; }

  // Immediate operations
  void writeDigital(uint8_t channel, bool on);
  void writePWM(uint8_t channel, uint16_t duty); // duty: 0..4095

  // Buffered operations - applied on manager update
  void bufferDigital(uint8_t channel, bool on);
  void bufferPWM(uint8_t channel, uint16_t duty);

  // internal access for manager
  void applyBuffered();
  void update() override;

 private:
  Adafruit_PWMServoDriver pwm_;
  uint16_t buffer_[16];
  bool buffer_dirty_[16];
};

}  // namespace Robot
