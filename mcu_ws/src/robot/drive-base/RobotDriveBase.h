/**
 * @file RobotDriveBase.h
 * @author Trevor Cannon
 * @brief MCU Drive Base
 * @date 12/14/2025
 */

#ifndef ROBOTDRIVEBASE_H
#define ROBOTDRIVEBASE_H

#include <memory>

#include "EncoderDriver.h"
#include "Localization.h"
#include "MotorDriver.h"
#include "Vector2D.h"

struct DriveBaseSetup {
  std::vector<std::unique_ptr<MotorDriver>> motors_;
  std::vector<std::unique_ptr<EncoderDriver>> encoders_;

  // PID stuff  (this might be too much for one file lolz)

  int numMotors;
};

class RobotDriveBase {
 public:
  ~RobotDriveBase() = default;
  RobotDriveBase(const DriveBaseSetup& _setup);

  void setMotorSpeeds(const int motorSpeeds[]);
  void setSingleMotorSpeed(const int motorSpeed, const int index);

 private:
  DriveBaseSetup driveSetup_;
  Localization localization_;

  Vector2D currentVelocityPose_;
  Vector2D targetVelocityPose_;
};

#endif