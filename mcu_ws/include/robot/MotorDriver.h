/**
 * @file MotorDriver.h
 * @author Trevor Cannon
 * @brief Motor driver
 * @date 12/12/2025
 */

#ifndef MOTORDRIVER_H
#define MOTORDRIVER_H

#include <BaseDriver.h>

namespace Drivers {

class MotorDriverSetup : public Classes::BaseSetup {
 public:
  int pwm;

  ~MotorDriverSetup() = default;
  MotorDriverSetup() = delete;

  MotorDriverSetup(char* _id) : Classes::BaseSetup(_id) {};

 private:
};

class MotorDriver : public Classes::BaseDriver {
 public:
  ~MotorDriver() override = default;
  MotorDriver(const MotorDriverSetup& setup)
      : setup_(setup), BaseDriver(setup) {};

  bool init() override;
  void update() override;
  char* getInfo() override;

 private:
  const MotorDriverSetup setup_;
};
}  // namespace Drivers

#endif