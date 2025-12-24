#include "RobotDriveBase.h"

#include <algorithm>
#include <string>

/**
 * @brief Constructor for RobotDriveBase object
 * @param DriveBaseSetup setup object
 */
RobotDriveBase::RobotDriveBase(const DriveBaseSetup& setup)
    : setup_(setup),
      localization_(setup.localizationSetup),
      leftWheelPID_(setup.leftWheelPIDSetup),
      rightWheelPID_(setup.rightWheelPIDSetup),
      linearMotionProfile_(setup.linearProfileConfig),
      angularMotionProfile_(setup.angularProfileConfig),
      trajController_(setup.trajControllerConfig) {
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

  // Set motion profile goals for smooth ramping
  // Reset profiles to current velocity to avoid jumps
  linearMotionProfile_.reset(
      {currentVelocity_.getX(), currentVelocity_.getX(), 0.0f});
  angularMotionProfile_.reset(
      {currentVelocity_.getTheta(), currentVelocity_.getTheta(), 0.0f});

  // Set target velocity as goal (final velocity = target velocity)
  linearMotionProfile_.setGoal({targetVelocity_.getX(), targetVelocity_.getX()});
  angularMotionProfile_.setGoal(
      {targetVelocity_.getTheta(), targetVelocity_.getTheta()});

  // Reset PIDs
  leftWheelPID_.reset();
  rightWheelPID_.reset();
}

/**
 * @brief Sets drive mode to drive given a setpoint pose
 * @param targetPose Pose2D position pose
 */
void RobotDriveBase::driveSetPoint(Pose2D& targetPose) {
  currentMode_ = DriveMode::POSE_DRIVE;
  targetPose_ = targetPose;

  // Reset PIDs
  leftWheelPID_.reset();
  rightWheelPID_.reset();
}

/**
 * @brief Sets drive mode to follow a trajectory waypoint path
 * @param waypoints Array of waypoints to follow
 * @param count Number of waypoints in the array
 */
void RobotDriveBase::driveTrajectory(
    const TrajectoryController::Waypoint* waypoints, size_t count) {
  currentMode_ = DriveMode::TRAJECTORY_DRIVE;

  // Set trajectory and reset controller
  trajController_.setTrajectory(waypoints, count);
  trajController_.reset();

  // Reset PIDs
  leftWheelPID_.reset();
  rightWheelPID_.reset();
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
  if (dleftTicks != 0 && dRightTicks != 0) {
    float dleftDist = static_cast<float>(dleftTicks) *
                      setup_.localizationSetup.inches_per_tick;
    float drightDist = static_cast<float>(dRightTicks) *
                       setup_.localizationSetup.inches_per_tick;

    float leftVel = dleftDist / dt;
    float rightVel = drightDist / dt;

    float vx = 0.5f * (leftVel + rightVel);
    float omega = (rightVel - leftVel) / setup_.localizationSetup.track_width;

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
  localization_.reset();
  prevPose_ = newPose;
}

/**
 * @brief Control loop for drive by target velocity mode
 * @param dt timestep
 */
void RobotDriveBase::velocityControl(float dt) {
  if (dt < 0.001f) return;

  // Update motion profiles to get smooth ramped velocities
  MotionState linearState = linearMotionProfile_.update(dt);
  MotionState angularState = angularMotionProfile_.update(dt);

  // Use profile velocity outputs (already acceleration/jerk limited)
  float rampedV = linearState.vel;
  float rampedOmega = angularState.vel;

  // Inverse kinematics: convert chassis velocity (v, omega) to wheel velocities
  float targetLeftWheelVel =
      rampedV - (rampedOmega * setup_.localizationSetup.track_width) * 0.5f;
  float targetRightWheelVel =
      rampedV + (rampedOmega * setup_.localizationSetup.track_width) * 0.5f;

  // Calculate actual wheel velocities from encoder ticks
  float actualLeftVel =
      (static_cast<float>(leftTicks_.position - prevLeftTicks_) *
       setup_.localizationSetup.inches_per_tick) /
      dt;
  float actualRightVel =
      (static_cast<float>(rightTicks_.position - prevRightTicks_) *
       setup_.localizationSetup.inches_per_tick) /
      dt;

  // PID control on wheel velocities
  float leftWheelOutput =
      leftWheelPID_.update(targetLeftWheelVel, actualLeftVel, dt);
  float rightWheelOutput =
      rightWheelPID_.update(targetRightWheelVel, actualRightVel, dt);

  writeMotorSpeeds(static_cast<int>(leftWheelOutput),
                   static_cast<int>(rightWheelOutput));
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
  v_cmd = std::clamp(v_cmd, -setup_.maxVelocity, setup_.maxVelocity);
  omega_cmd = std::clamp(omega_cmd, -setup_.maxAngularVelocity,
                         setup_.maxAngularVelocity);

  targetVelocity_ = Vector2D(v_cmd, 0.0f, omega_cmd);

  velocityControl(dt);
}

/**
 * @brief Control loop for trajectory following mode
 * @param dt timestep
 */
void RobotDriveBase::trajectoryControl(float dt) {
  if (dt < 0.001f) return;

  // Get current pose from localization
  Pose2D currentPose = localization_.getPose();

  // Convert Pose2D to TrajectoryController::Pose2D
  TrajectoryController::Pose2D trajPose;
  trajPose.x = currentPose.getX();
  trajPose.y = currentPose.getY();
  trajPose.theta = currentPose.getTheta();

  // Update trajectory controller to get commanded chassis velocities (v, w)
  TrajectoryController::Command cmd = trajController_.update(trajPose, dt);

  // If trajectory is finished, stop
  if (cmd.finished) {
    writeMotorSpeeds(0, 0);
    return;
  }

  // Convert chassis velocities (v, w) to wheel velocities
  float targetLeftWheelVel = 0.0f;
  float targetRightWheelVel = 0.0f;
  TrajectoryController::chassisToWheelSpeeds(
      cmd.v, cmd.w, setup_.localizationSetup.track_width, &targetLeftWheelVel,
      &targetRightWheelVel);

  // Calculate actual wheel velocities from encoder ticks
  float actualLeftVel =
      (static_cast<float>(leftTicks_.position - prevLeftTicks_) *
       setup_.localizationSetup.inches_per_tick) /
      dt;
  float actualRightVel =
      (static_cast<float>(rightTicks_.position - prevRightTicks_) *
       setup_.localizationSetup.inches_per_tick) /
      dt;

  // PID control on wheel velocities
  float leftWheelOutput =
      leftWheelPID_.update(targetLeftWheelVel, actualLeftVel, dt);
  float rightWheelOutput =
      rightWheelPID_.update(targetRightWheelVel, actualRightVel, dt);

  writeMotorSpeeds(static_cast<int>(leftWheelOutput),
                   static_cast<int>(rightWheelOutput));
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
    case DriveMode::TRAJECTORY_DRIVE:
      trajectoryControl(dt);
      break;
    default:
      break;
  }
  prevPose_ = localization_.getPose();
  prevLeftTicks_ = leftTicks_.position;
  prevRightTicks_ = rightTicks_.position;
}
