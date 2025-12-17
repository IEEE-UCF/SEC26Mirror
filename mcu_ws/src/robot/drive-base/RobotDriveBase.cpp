#include "RobotDriveBase.h"

#include <algorithm>
#include <string>

/**
 * @brief Constructor for RobotDriveBase object
 * @param DriveBaseSetup setup object
 */
RobotDriveBase::RobotDriveBase(const DriveBaseSetup& setup) : setup_(setup) {
  // this is kinda dumb, also assumes one encoder per side

  for (const auto& motorSetup : setup.motorSetups) {
    auto newMotor = std::make_unique<Drivers::MotorDriver>(motorSetup);

    std::string id = motorSetup.getId();

    if (id.find("left") != std::string::npos) {
      leftMotors_.push_back(std::move(newMotor));
    } else if (id.find("right") != std::string::npos) {
      rightMotors_.push_back(std::move(newMotor));
    }
  }

  for (const auto& encoderSetup : setup.encoderSetups) {
    auto newEncoder = std::make_unique<Drivers::EncoderDriver>(encoderSetup);

    std::string id = encoderSetup.getId();

    if (id.find("left") != std::string::npos) {
      leftEncoder_ = std::move(newEncoder);
    } else if (id.find("right") != std::string::npos) {
      rightEncoder_ = std::move(newEncoder);
    }
  }
  prevPose_ = localization_.getPose();
  prevLeftTicks_ = leftEncoder_ ? leftEncoder_->getPosition().position : 0;
  prevRightTicks_ = rightEncoder_ ? rightEncoder_->getPosition().position : 0;
};

/**
 * @brief Sets drive mode to drive by velocity given a target velocity pose
 * @param targetVelocity Vector2D velocity pose
 */
void RobotDriveBase::driveVelocity(Vector2D& targetVelocity) {
  currentMode_ = DriveMode::VELOCITY_DRIVE;
  targetVelocity_ = targetVelocity;

  // Reset PIDs
  // leftPID.reset()
  // rightPID.reset()
}

/**
 * @brief Sets drive mode to drive given a setpoint pose
 * @param targetPose Pose2D position pose
 */
void RobotDriveBase::driveSetPoint(Pose2D& targetPose) {
  currentMode_ = DriveMode::POSE_DRIVE;
  targetPose_ = targetPose;

  // Reset PIDs
  // leftPID.reset()
  // rightPID.reset()
}

/**
 * @brief Manual driving mode to directly control motor speeds
 * @param leftSpeed left motor speed
 * @param rightSpeed right motorspeed
 */
void RobotDriveBase::manualMotorSpeeds(int leftSpeed, int rightSpeed) {
  currentMode_ = DriveMode::MANUAL;

  for (auto& motor : leftMotors_) motor->setPWM(leftSpeed);
  for (auto& motor : rightMotors_) motor->setPWM(rightSpeed);
}

/**
 * @brief Applies motor speeds from PIDs
 * @param leftSpeed left motor speed
 * @param rightSpeed right motorspeed
 */
void RobotDriveBase::writeMotorSpeeds(int leftSpeed, int rightSpeed) {
  for (auto& motor : leftMotors_) motor->setPWM(leftSpeed);
  for (auto& motor : rightMotors_) motor->setPWM(rightSpeed);
}

/**
 * @brief Calculates the current velocity using encoder ticks, fall back to pose
 * differentiation if encoder fails
 * @param none
 */
Vector2D RobotDriveBase::getCurrentVelocity(float dt) {
  if (dt < 0.001f) return currentVelocity_;

  long dleftTicks = leftTicks_.position - prevLeftTicks_;
  long dRightTicks = rightTicks_.position - prevRightTicks_;

  // encoder ticks for velocity
  if (dleftTicks != 0 || dRightTicks != 0) {
    float dleftDist = static_cast<float>(dleftTicks) * RobotConfig::IN_PER_TICK;
    float drightDist =
        static_cast<float>(dRightTicks) * RobotConfig::IN_PER_TICK;

    float leftVel = dleftDist / dt;
    float rightVel = drightDist / dt;

    float vx = 0.5f * (leftVel + rightVel);
    float omega = (rightVel - leftVel) / RobotConfig::TRACK_WIDTH;

    return Vector2D(vx, 0.0f, omega);
  }

  Pose2D currentPose = localization_.getPose();

  float dX = currentPose.getX() - prevPose_.getX();
  float dY = currentPose.getY() - prevPose_.getY();
  float dTheta = currentPose.getTheta() - prevPose_.getTheta();

  Pose2D dPose = Pose2D(dX, dY, dTheta);
  dPose.normalizeTheta();

  float worldVx = dPose.x / dt;
  float worldVy = dPose.y / dt;
  float omega = dPose.theta / dt;

  float cosTheta = cosf(-currentPose.theta);
  float sinTheta = sinf(-currentPose.theta);
  float robotVx = worldVx * cosTheta - worldVy * sinTheta;
  float robotVy = worldVx * sinTheta + worldVy * cosTheta;

  return Vector2D(robotVx, robotVy, omega);
}

/**
 * @brief sets all motor speeds to 0, stops
 * @param none
 */
void RobotDriveBase::stop() { manualMotorSpeeds(0, 0); }

/**
 * @brief Resets pose to given coordinates
 * @param newPose Pose2D object
 */
void RobotDriveBase::resetPose(Pose2D& newPose) {
  localization_ = Localization(newPose.x, newPose.y, newPose.theta);
  prevPose_ = newPose;
}

/**
 * @brief Control loop for drive by target velocity mode
 * @param dt timestep
 */
void RobotDriveBase::velocityControl(float dt) {
  if (dt < 0.001f) return;

  // ramp up linear + angular velocity to prevent sudden changes
  float linearAccelLimit = RobotConfig::MAX_ACCELERATION * dt;
  float deltaV = std::clamp(targetVelocity_.getX() - currentVelocity_.getX(),
                            -linearAccelLimit, linearAccelLimit);

  float angularAccelLimit = RobotConfig::MAX_ANGULAR_ACCELERATION * dt;
  float deltaOmega =
      std::clamp(targetVelocity_.getTheta() - currentVelocity_.getTheta(),
                 -angularAccelLimit, angularAccelLimit);

  // ideal state we are targeting to stay within limits
  float rampedV = currentVelocity_.getX() + deltaV;
  float rampedOmega = currentVelocity_.getTheta() + deltaOmega;

  // inverse kinematics
  float targetLeftWheelVel =
      rampedV - (rampedOmega * RobotConfig::TRACK_WIDTH) * 0.5f;
  float targetRightWheelVel =
      rampedV + (rampedOmega * RobotConfig::TRACK_WIDTH) * 0.5f;

  float actualLeftVel =
      (static_cast<float>(leftTicks_.position - prevLeftTicks_) *
       RobotConfig::IN_PER_TICK) /
      dt;
  float actualRightVel =
      (static_cast<float>(rightTicks_.position - prevRightTicks_) *
       RobotConfig::IN_PER_TICK) /
      dt;

  // int leftOutput = leftPID.calculate(actualLeftVel, targetLeftWheelVel, dt);
  // int rightOutput = rightPID.calculate(actualRightVel, targetRightWheelVel,
  // dt);

  // applyMotorSpeeds(leftOutput, rightOutput);
}

/**
 * @brief Control loop for drive by target setpoint mode
 * @param dt timestep
 */
void RobotDriveBase::setPointControl(float dt) {
  Pose2D currentPose = localization_.getPose();

  float deltaX = targetPose_.getX() - currentPose.getX();
  float deltaY = targetPose_.getY() - currentPose.getY();

  float distanceError = std::hypot(deltaX, deltaY);

  float targetAngle = std::atan2(deltaY, deltaX);

  float angleError = targetAngle - currentPose.getTheta();
  angleError = std::remainder(angleError, 2 * PI);

  // stop if close to target
  const float DISTANCE_THRESHOLD = 0.5f;
  if (distanceError < DISTANCE_THRESHOLD) {
    targetVelocity_ = Vector2D(0, 0, 0);
    velocityControl(dt);
    return;
  }

  // control gains may need to be adjusted
  const float K_LINEAR = 2.0f;   // Proportional gain for forward speed
  const float K_ANGULAR = 4.0f;  // Proportional gain for turning speed

  float v_cmd = K_LINEAR * distanceError;
  float omega_cmd = K_ANGULAR * angleError;

  // --- Optimization: Slow down forward speed if we aren't facing the target
  // yet --- This prevents the robot from driving in huge arcs. As the angle
  // error increases, cos(angleError) decreases, slowing down Vx.
  v_cmd *= std::max(0.0f, std::cos(angleError));

  // clamp to set limits
  v_cmd =
      std::clamp(v_cmd, -RobotConfig::MAX_VELOCITY, RobotConfig::MAX_VELOCITY);
  omega_cmd = std::clamp(omega_cmd, -RobotConfig::MAX_ANGULAR_VELOCITY,
                         RobotConfig::MAX_ANGULAR_VELOCITY);

  targetVelocity_ = Vector2D(v_cmd, 0.0f, omega_cmd);

  velocityControl(dt);
}

/**
 * @brief Main update loop, updates encoders, localization, picks control loop
 * based on current mode
 * @param dt timestep
 * @param yaw current yaw
 */
void RobotDriveBase::update(float yaw, float dt) {
  if (leftEncoder_ == nullptr || rightEncoder_ == nullptr) return;

  leftTicks_ = leftEncoder_->getPosition();
  rightTicks_ = rightEncoder_->getPosition();

  currentVelocity_ = getCurrentVelocity(dt);

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
  prevPose_ = localization_.getPose();
  prevLeftTicks_ = leftTicks_.position;
  prevRightTicks_ = rightTicks_.position;
}
