/**
 * @file MotorDriver.h
 * @author Trevor Cannon
 * @brief Motor driver with HAL support
 * @date 12/12/2025
 * @refactored 12/24/2025 - Added HAL support for flexible pin backends
 */

#ifndef MOTORDRIVER_H
#define MOTORDRIVER_H

#include <Arduino.h>
#include <BaseDriver.h>
#include <HALPin.h>

namespace Drivers {

class MotorDriverSetup : public Classes::BaseSetup {
 public:
  HAL::HALPin* pwm_pin;
  HAL::HALPin* dir_pin;

  ~MotorDriverSetup() = default;
  MotorDriverSetup() = delete;

  MotorDriverSetup(const char* _id, HAL::HALPin* _pwm_pin,
                   HAL::HALPin* _dir_pin)
      : Classes::BaseSetup(_id), pwm_pin(_pwm_pin), dir_pin(_dir_pin) {};

 private:
};

class MotorDriver : public Classes::BaseDriver {
 public:
  const int PWM_MAX = 255;
  const int SPEED_MAX = 255;

  ~MotorDriver() override = default;
  MotorDriver(const MotorDriverSetup& setup)
      : BaseDriver(setup), setup_(setup) {};

  bool init() override;
  void update() override;
  const char* getInfo() override;

  void setPWM(int speed);

 private:
  const MotorDriverSetup setup_;
  int pwmOut;
  bool motorDirection;
  elapsedMicros prevReverseTime;
  char infoBuffer_[64];
};
}  // namespace Drivers

#endif