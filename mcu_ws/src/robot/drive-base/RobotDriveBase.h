/**
 * @file RobotDriveBase.h
 * @author Trevor Cannon
 * @brief MCU Drive Base
 * @date 12/14/2025
 */

#ifndef ROBOTDRIVEBASE_H
#define ROBOTDRIVEBASE_H

#include <Pose2D.h>
#include <TankDriveLocalization.h>
#include <Vector2D.h>

#include <memory>
#include <vector>

#include "EncoderDriver.h"
#include "MotorDriver.h"
#include "motion_profile.h"
#include "pid_controller.h"
#include "traj_controller.h"

enum class DriveMode { MANUAL, VELOCITY_DRIVE, POSE_DRIVE, TRAJECTORY_DRIVE };

struct DriveBaseSetup {
  std::vector<Drivers::MotorDriverSetup> motorSetups;
  std::vector<Drivers::EncoderDriverSetup> encoderSetups;

  PIDController::Config leftWheelPIDSetup;
  PIDController::Config rightWheelPIDSetup;

  Drive::TankDriveLocalizationSetup localizationSetup;

  // Motion profile configs for smooth velocity ramping
  SCurveMotionProfile::Config linearProfileConfig;
  SCurveMotionProfile::Config angularProfileConfig;

  // Trajectory controller config for path following
  TrajectoryController::Config trajControllerConfig;

  // Robot constants for control (kept for backward compatibility)
  float maxVelocity;
  float maxAcceleration;
  float maxAngularVelocity;
  float maxAngularAcceleration;
};

class RobotDriveBase {
 public:
  ~RobotDriveBase() = default;
  RobotDriveBase(const DriveBaseSetup& setup_);

  void driveVelocity(Vector2D& targetVelocity);
  void driveSetPoint(Pose2D& targetPose);
  void driveTrajectory(const TrajectoryController::Waypoint* waypoints,
                       size_t count);

  void manualMotorSpeeds(int leftSpeed, int rightSpeed);

  Vector2D getCurrentVelocity(float dt);

  void resetPose(Pose2D& newPose);
  void stop();

  void update(float yaw, float dt);

  DriveMode getCurrentMode() const { return currentMode_; };
  Pose2D getCurrentPose() { return localization_.getPose(); };

 private:
  void velocityControl(float dt);
  void setPointControl(float dt);
  void trajectoryControl(float dt);
  void writeMotorSpeeds(int leftSpeed, int rightSpeed);

  // Wheel velocity PIDs
  PIDController leftWheelPID_;
  PIDController rightWheelPID_;

  // Motion profiles for smooth velocity ramping
  SCurveMotionProfile linearMotionProfile_;
  SCurveMotionProfile angularMotionProfile_;

  // Trajectory controller for path following
  TrajectoryController trajController_;

  DriveBaseSetup setup_;
  Drive::TankDriveLocalization localization_;

  std::vector<std::unique_ptr<Drivers::MotorDriver>> leftMotors_;
  std::vector<std::unique_ptr<Drivers::MotorDriver>> rightMotors_;
  std::unique_ptr<Drivers::EncoderDriver> leftEncoder_;
  std::unique_ptr<Drivers::EncoderDriver> rightEncoder_;

  Drivers::EncoderData leftTicks_ = Drivers::EncoderData(0);
  Drivers::EncoderData rightTicks_ = Drivers::EncoderData(0);
  long prevLeftTicks_;
  long prevRightTicks_;

  DriveMode currentMode_ = DriveMode::MANUAL;

  Vector2D currentVelocity_;
  Vector2D targetVelocity_;
  Pose2D targetPose_;
  Pose2D prevPose_;
};

#endif