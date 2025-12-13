/**
 * @file MotorDriver.h
 * @author Trevor Cannon
 * @brief Motor driver
 * @date 12/12/2025
 */

#ifndef MOTORDRIVER_H
#define MOTORDRIVER_H

namespace Drivers {

class MotorDriverSetup : public Classes::BaseSetup {
 public:
 private:
};

class MotorDriver : public Classes::BaseDriver {
 public:
  ~MotorDriver() = default;

 private:
  const MotorDriverSetup setup_;
};
}  // namespace Drivers

#endif