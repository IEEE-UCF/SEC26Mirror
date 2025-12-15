/**
 * @file RobotDriveBase.h
 * @author Trevor Cannon
 * @brief MCU Drive Base
 * @date 12/14/2025
 */

#ifndef ROBOTDRIVEBASE_H
#define ROBOTDRIVEBASE_H

#include <memory>
#include <vector>

#include "EncoderDriver.h"
#include "MotorDriver.h"
#include "Vector2D.h"
#include "localization-subsystem/Localization.h"
#include "localization-subsystem/RobotConfig.h"

enum class DriveMode { MANUAL, AUTO };

struct DriveBaseSetup {
  std::vector<Drivers::MotorDriverSetup> motorSetups;
  std::vector<Drivers::EncoderDriverSetup> encoderSetups;

  // PID Config
};

class RobotDriveBase {
 public:
  ~RobotDriveBase() = default;
  RobotDriveBase(const DriveBaseSetup& setup_);

  void driveVelocity(Vector2D& targetVelocity);
  void driveSetPoint(Pose2D& targetPose);

  void setMotorSpeeds(int leftSpeed, int rightSpeed);

  void resetPose();

  void update(float dt);

 private:
  void readEncoders();
  void updateLocalization(float yaw);
  void velocityFromPose();
  void velocityControl(float dt);

  DriveBaseSetup setup_;
  Localization localization_;

  std::vector<std::unique_ptr<Drivers::MotorDriver>> leftMotors_;
  std::vector<std::unique_ptr<Drivers::MotorDriver>> rightMotors_;
  std::unique_ptr<Drivers::EncoderDriver> leftEncoder_;
  std::unique_ptr<Drivers::EncoderDriver> rightEncoder_;

  Drivers::EncoderData leftTicks_;
  Drivers::EncoderData rightTicks_;

  DriveMode currentMode_ = DriveMode::AUTO;

  Vector2D targetVelocity_;
  Pose2D targetPose_;
};

#endif