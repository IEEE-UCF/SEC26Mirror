/**
 * @file MotorDriver.h
 * @author Trevor Cannon
 * @brief Motor driver
 * @date 12/12/2025
 */

#ifndef MOTORDRIVER_H
#define MOTORDRIVER_H

#include <Arduino.h>
#include <BaseDriver.h>

namespace Drivers {

class MotorDriverSetup : public Classes::BaseSetup {
 public:
  const int pwm_pin;
  const int dir_pin;

  ~MotorDriverSetup() = default;
  MotorDriverSetup() = delete;

  MotorDriverSetup(const char* _id, const int _pwm_pin, const int _dir_pin)
      : Classes::BaseSetup(_id), pwm_pin(_pwm_pin), dir_pin(_dir_pin){};

 private:
};

class MotorDriver : public Classes::BaseDriver {
 public:
  const int PWM_MAX = 255;
  const int SPEED_MAX = 255;

  ~MotorDriver() override = default;
  MotorDriver(const MotorDriverSetup& setup)
      : BaseDriver(setup), setup_(setup){};

  bool init() override;
  void update() override;
  const char* getInfo() override;

  void setPWM(int speed);

 private:
  const MotorDriverSetup setup_;
  int pwmOut;
  bool motorDirection;
  char infoBuffer_[64];
};
}  // namespace Drivers

#endif