#include "RobotDriveBase.h"

RobotDriveBase::RobotDriveBase(const DriveBaseSetup& setup)
    : setup_(setup) {

      };

void RobotDriveBase::driveVelocity(Vector2D& targetVelocity) {
  targetVelocity_ = targetVelocity;
}

void RobotDriveBase::driveSetPoint(Pose2D& targetPose) {
  targetPose_ = targetPose;
}

void RobotDriveBase::setMotorSpeeds(int leftSpeed, int rightSpeed) {
  currentMode_ = DriveMode::MANUAL;

  for (auto& motor : leftMotors_) motor->setPWM(leftSpeed);
  for (auto& motor : rightMotors_) motor->setPWM(rightSpeed);
}

void RobotDriveBase::stop() {
  currentMode_ = DriveMode::MANUAL;
  for (auto& motor : leftMotors_) motor->setPWM(0);
  for (auto& motor : rightMotors_) motor->setPWM(0);
}

void RobotDriveBase::resetPose(Pose2D newPose) {
  localization_ = Localization(newPose.x, newPose.y, newPose.theta);
}

void readEncoders() {
  leftTicks_ = leftEncoder_->getPosition();
  rightTicks_ = rightEncoder_->getPosition();
}

void RobotDriveBase::updateLocalization(float yaw) {
  localization_.update(leftTicks_.position, rightTicks_.position, yaw);
}

void RobotDriveBase::update(float dt) {
  readEncoders();
  updateLocalization();

  if (currentMode_ = DriveMode::MANUAL) return;
}
