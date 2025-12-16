#include "RobotDriveBase.h"

#include <string>

RobotDriveBase::RobotDriveBase(const DriveBaseSetup& setup) : setup_(setup) {
  // this is kinda dumb, also assumes one encoder per side

  for (const auto& setup : setup.motorSetups) {
    auto newMotor = std::make_unique<Drivers::MotorDriver>(setup);

    std::string id = setup.getId();

    if (id.find("left") != std::string::npos) {
      leftMotors_.push_back(std::move(newMotor));
    } else if (id.find("right") != std::string::npos) {
      rightMotors_.push_back(std::move(newMotor));
    }
  }

  for (const auto& setup : setup.encoderSetups) {
    auto newEncoder = std::make_unique<Drivers::EncoderDriver>(setup);

    std::string id = setup.getId();

    if (id.find("left") != std::string::npos) {
      leftEncoder_ = std::move(newEncoder);
    } else if (id.find("right") != std::string::npos) {
      rightEncoder_ = std::move(newEncoder);
    }
  }
};

void RobotDriveBase::driveVelocity(Vector2D& targetVelocity) {
  currentMode_ = DriveMode::VELOCITY_DRIVE;
  targetVelocity_ = targetVelocity;

  // Reset PIDs
}

void RobotDriveBase::driveSetPoint(Pose2D& targetPose) {
  currentMode_ = DriveMode::POSE_DRIVE;
  targetPose_ = targetPose;

  // Reset PIDs
}

void RobotDriveBase::manualMotorSpeeds(int leftSpeed, int rightSpeed) {
  currentMode_ = DriveMode::MANUAL;

  for (auto& motor : leftMotors_) motor->setPWM(leftSpeed);
  for (auto& motor : rightMotors_) motor->setPWM(rightSpeed);
}

void RobotDriveBase::applyMotorSpeeds(int leftSpeed, int rightSpeed) {
  for (auto& motor : leftMotors_) motor->setPWM(leftSpeed);
  for (auto& motor : rightMotors_) motor->setPWM(rightSpeed);
}

Vector2D RobotDriveBase::getCurrentVelocity() {
  const float MIN_DT = 0.001f;
  if (prevDt_ < MIN_DT) return Vector2D();

  Pose2D currentPose = localization_.getPose();

  float dX = currentPose.getX() - prevPose_.getX();
  float dY = currentPose.getY() - prevPose_.getY();
  float dTheta = currentPose.getTheta() - prevPose_.getTheta();

  Pose2D dPose = Pose2D(dX, dY, dTheta);
  dPose.normalizeTheta();

  float worldVx = dPose.x / prevDt_;
  float worldVy = dPose.y / prevDt_;
  float omega = dPose.theta / prevDt_;

  float cosTheta = cosf(-currentPose.theta);
  float sinTheta = sinf(-currentPose.theta);

  float robotVx = worldVx * cosTheta - worldVy * sinTheta;
  float robotVy = worldVx * sinTheta + worldVy * cosTheta;

  return Vector2D(robotVx, robotVy, omega);
}

void RobotDriveBase::stop() { manualMotorSpeeds(0, 0); }

void RobotDriveBase::resetPose(Pose2D& newPose) {
  localization_ = Localization(newPose.x, newPose.y, newPose.theta);
  prevPose_ = newPose;
  prevDt_ = 0.0f;
}

void RobotDriveBase::velocityControl(float dt) {
  Vector2D currentVelocity = getCurrentVelocity();

  float errorVx = targetVelocity_.getX() - currentVelocity.getX();
  float errorVy = targetVelocity_.getY() - currentVelocity.getY();
  float errorOmega = targetVelocity_.getTheta() - currentVelocity.getTheta();

  // PID
  // applyMotorSpeeds(leftPID output,rightPID output)
}

void RobotDriveBase::setPointControl(float dt) {
  Pose2D currentPose = localization_.getPose();
  float errorX = targetPose_.getX() - currentPose.getX();
  float errorY = targetPose_.getY() - currentPose.getY();
  float errorTheta = targetPose_.getTheta() - currentPose.getTheta();

  Pose2D errorPose = Pose2D(errorX, errorY, errorTheta);
  errorPose.normalizeTheta();

  float distance =
      std::sqrt(errorPose.x * errorPose.x + errorPose.y * errorPose.y);

  // PID
  // applyMotorSpeeds(leftPID output,rightPID output)
}

void RobotDriveBase::update(float yaw, float dt) {
  if (leftEncoder_ == nullptr || rightEncoder_ == nullptr) return;

  prevDt_ = dt;
  prevPose_ = localization_.getPose();

  leftTicks_ = leftEncoder_->getPosition();
  rightTicks_ = rightEncoder_->getPosition();
  localization_.update(leftTicks_.position, rightTicks_.position, yaw);

  switch (currentMode_) {
    case DriveMode::MANUAL:
      break;
    case DriveMode::VELOCITY_DRIVE:
      velocityControl(dt);
      break;
    case DriveMode::POSE_DRIVE:
      setPointControl(dt);
      break;
    default:
      break;
  }
}
