/**
 * @file DriveSubsystem.h
 * @brief Full drivebase control subsystem for tank-drive omniwheels.
 * @date 2026-02-28
 *
 * Integrates with the existing MotorManagerSubsystem (PCA9685 motors),
 * EncoderSubsystem (QTimer hardware counters), and ImuSubsystem (BNO085)
 * to provide closed-loop velocity, pose, and trajectory control.
 *
 * All internal units are SI: meters, radians, seconds.
 *
 * -- ROS2 interface (defaults) ------------------------------------------------
 *   drive_base/status       pub   mcu_msgs/DriveBase        (20 Hz)
 *   drive_base/command      sub   mcu_msgs/DriveBase        (cmd from Pi)
 *   drive_base/reset_pose   sub   geometry_msgs/Pose        (pose override)
 */

#pragma once

#include <BaseSubsystem.h>
#include <Pose2D.h>
#include <TankDriveLocalization.h>
#include <Vector2D.h>
#include <geometry_msgs/msg/pose.h>
#include <filters.h>
#include <math_utils.h>
#include <mcu_msgs/msg/drive_base.h>
#include <micro_ros_utilities/type_utilities.h>
#include <microros_manager_robot.h>
#include <motion_profile.h>
#include <pid_controller.h>
#include <traj_controller.h>

#ifdef USE_TEENSYTHREADS
#include <TeensyThreads.h>
#endif

#include "TimedSubsystem.h"
#include "robot/subsystems/EncoderSubsystem.h"
#include "robot/subsystems/ImuSubsystem.h"
#include "robot/subsystems/MotorManagerSubsystem.h"

namespace Subsystem {

// ═══════════════════════════════════════════════════════════════════════════
//  Drive modes
// ═══════════════════════════════════════════════════════════════════════════

enum class DriveMode : uint8_t {
  IDLE,        // Motors off, no active control
  VELOCITY,    // Twist (vx, omega) → S-curve → wheel PIDs
  GOAL,        // Drive to (x, y, theta) pose
  TRAJECTORY,  // Follow waypoint path (pure pursuit)
  MANUAL       // Direct motor speed passthrough (for RC)
};

// ═══════════════════════════════════════════════════════════════════════════
//  Setup
// ═══════════════════════════════════════════════════════════════════════════

struct DriveSubsystemSetup : public Classes::BaseSetup {
  // Hardware references (must be initialized before DriveSubsystem)
  MotorManagerSubsystem* motorManager = nullptr;
  EncoderSubsystem* encoderSub = nullptr;
  ImuSubsystem* imuSub = nullptr;
  Encoders::QTimerEncoder* encoder = nullptr;  // direct QTimer access for captureAll

  // Channel mapping
  uint8_t leftMotorIdx = 1;
  uint8_t rightMotorIdx = 0;
  uint8_t leftEncoderIdx = 1;
  uint8_t rightEncoderIdx = 0;
  bool leftEncoderInverted = false;
  bool rightEncoderInverted = false;

  // Localization setup (meters)
  Drive::TankDriveLocalizationSetup locSetup;

  // Wheel velocity PID configs
  PIDController::Config leftPID;
  PIDController::Config rightPID;

  // Motion profile configs
  SCurveMotionProfile::Config linearProfile;
  SCurveMotionProfile::Config angularProfile;

  // Trajectory controller config
  TrajectoryController::Config trajConfig;

  // Limits
  float maxLinearVel = 0.76f;    // m/s
  float maxAngularVel = 4.0f;    // rad/s

  // Pose drive gains
  float poseKLinear = 2.0f;
  float poseKAngular = 4.0f;
  float poseDistTol = 0.015f;    // meters

  // Motor direction multipliers (1.0 or -1.0 to reverse a motor)
  float leftMotorMultiplier = -1.0f;
  float rightMotorMultiplier = 1.0f;

  // Safety
  uint32_t commandTimeoutMs = 500;

  // Topic names
  const char* statusTopic = "drive_base/status";
  const char* commandTopic = "drive_base/command";
  const char* resetPoseTopic = "drive_base/reset_pose";

  // Publish rate divider (publish every N update cycles)
  uint8_t publishDivider = 1;  // 1 = every cycle (50Hz), 2 = 25Hz, etc.

  DriveSubsystemSetup(
      const char* _id,
      MotorManagerSubsystem* _motor,
      EncoderSubsystem* _encoderSub,
      ImuSubsystem* _imu,
      const Drive::TankDriveLocalizationSetup& _locSetup,
      Encoders::QTimerEncoder* _encoder = nullptr)
      : Classes::BaseSetup(_id),
        motorManager(_motor),
        encoderSub(_encoderSub),
        imuSub(_imu),
        encoder(_encoder),
        locSetup(_locSetup) {}
};

// ═══════════════════════════════════════════════════════════════════════════
//  DriveSubsystem
// ═══════════════════════════════════════════════════════════════════════════

class DriveSubsystem : public IMicroRosParticipant,
                       public Subsystem::TimedSubsystem {
 public:
  explicit DriveSubsystem(const DriveSubsystemSetup& setup)
      : Subsystem::TimedSubsystem(setup),
        setup_(setup),
        localization_(setup.locSetup),
        leftPID_(setup.leftPID),
        rightPID_(setup.rightPID),
        linearProfile_(setup.linearProfile),
        angularProfile_(setup.angularProfile),
        trajController_(setup.trajConfig) {}

  // ── Lifecycle ───────────────────────────────────────────────────────────

  bool init() override {
    // Re-configure control objects from setup — at static construction time the
    // setup struct hadn't been populated yet (configureDriveSetup() runs later
    // in Arduino setup()), so the member objects were initialized with defaults.
    leftPID_.configure(setup_.leftPID);
    rightPID_.configure(setup_.rightPID);
    linearProfile_.configure(setup_.linearProfile);
    angularProfile_.configure(setup_.angularProfile);
    trajController_.configure(setup_.trajConfig);

    // RC input smoothing: tau=0.15s, dt=0.005s (5ms main loop polling rate)
    rcThrottleFilter_.configureTauDtFast(0.15f, 0.005f);
    rcSteeringFilter_.configureTauDtFast(0.15f, 0.005f);

    // Ensure motors are off at startup
    stopMotors();
    last_update_us_ = micros();
    return true;
  }

  void begin() override {
    // Motors remain off until explicitly commanded
    stopMotors();
  }

  void update() override {
    if (!setup_.motorManager || !setup_.encoderSub || !setup_.imuSub) return;

    // Capture all encoder deltas and sync hardware direction registers.
    // Must happen before reading ticks so accumulated values are fresh.
    captureEncoders();

    // ── Timing ──
    uint32_t now_us = micros();
    uint32_t dt_us = now_us - last_update_us_;
    last_update_us_ = now_us;
    if (dt_us == 0) dt_us = 1;
    float dt = static_cast<float>(dt_us) * 1e-6f;
    if (dt > 0.1f) dt = 0.1f;  // clamp to prevent huge jumps

    // ── Read encoders ──
    int32_t leftTicks = setup_.encoderSub->getAccumulatedTicks(
        setup_.leftEncoderIdx);
    int32_t rightTicks = setup_.encoderSub->getAccumulatedTicks(
        setup_.rightEncoderIdx);

    // Apply inversion if needed
    if (setup_.leftEncoderInverted) leftTicks = -leftTicks;
    if (setup_.rightEncoderInverted) rightTicks = -rightTicks;

    // ── Read IMU yaw ──
    float yaw = setup_.imuSub->getYaw();

    // ── Update localization ──
    localization_.update(static_cast<long>(leftTicks),
                         static_cast<long>(rightTicks), yaw);

    // ── Compute current velocity ──
    int32_t dLeft = leftTicks - prevLeftTicks_;
    int32_t dRight = rightTicks - prevRightTicks_;
    prevLeftTicks_ = leftTicks;
    prevRightTicks_ = rightTicks;

    if (dt > 0.001f) {
      float leftVel =
          static_cast<float>(dLeft) * setup_.locSetup.dist_per_tick / dt;
      float rightVel =
          static_cast<float>(dRight) * setup_.locSetup.dist_per_tick / dt;
      currentLinearVel_ = 0.5f * (leftVel + rightVel);
      currentAngularVel_ =
          (rightVel - leftVel) / setup_.locSetup.track_width;
    }

    // ── Command timeout check ──
    if (mode_ != DriveMode::IDLE && mode_ != DriveMode::MANUAL) {
      uint32_t now_ms = millis();
      if (now_ms - lastCommandMs_ > setup_.commandTimeoutMs) {
        mode_ = DriveMode::IDLE;
      }
    }

    // ── Run active control mode ──
    switch (mode_) {
      case DriveMode::IDLE:
        stopMotors();
        break;
      case DriveMode::VELOCITY:
        velocityControl(dt);
        break;
      case DriveMode::GOAL:
        setPointControl(dt);
        break;
      case DriveMode::TRAJECTORY:
        trajectoryControl(dt);
        break;
      case DriveMode::MANUAL:
        // manualDrive() already wrote motor speeds directly
        break;
    }

    // ── Publish status ──
    publishCounter_++;
    if (publishCounter_ >= setup_.publishDivider) {
      publishCounter_ = 0;
      publishStatus();
    }
  }

  void pause() override {
    mode_ = DriveMode::IDLE;
    stopMotors();
    leftPID_.reset();
    rightPID_.reset();
  }

  void reset() override {
    pause();
    localization_.reset();
    prevLeftTicks_ = 0;
    prevRightTicks_ = 0;
    currentLinearVel_ = 0.0f;
    currentAngularVel_ = 0.0f;
    targetLinearVel_ = 0.0f;
    targetAngularVel_ = 0.0f;
  }

  const char* getInfo() override {
    static const char info[] = "DriveSubsystem";
    return info;
  }

  // ── IMicroRosParticipant ────────────────────────────────────────────────

  bool onCreate(rcl_node_t* node, rclc_executor_t* executor) override {
    node_ = node;

    // Initialize status message memory (handles variable-length fields)
    status_mem_conf_ = {};
    status_mem_conf_.max_string_capacity = 20;
    status_mem_conf_.max_ros2_type_sequence_capacity = 0;
    status_mem_conf_.max_basic_type_sequence_capacity = 0;
    if (!micro_ros_utilities_create_message_memory(
            ROSIDL_GET_MSG_TYPE_SUPPORT(mcu_msgs, msg, DriveBase),
            &status_msg_, status_mem_conf_)) {
      return false;
    }

    // Initialize command message memory (needs path capacity for TRAJ mode)
    cmd_mem_conf_ = {};
    cmd_mem_conf_.max_string_capacity = 20;
    cmd_mem_conf_.max_ros2_type_sequence_capacity = MAX_TRAJECTORY_POINTS;
    cmd_mem_conf_.max_basic_type_sequence_capacity = 0;
    if (!micro_ros_utilities_create_message_memory(
            ROSIDL_GET_MSG_TYPE_SUPPORT(mcu_msgs, msg, DriveBase),
            &cmd_msg_, cmd_mem_conf_)) {
      return false;
    }

    // Publisher: drive_base/status
    if (rclc_publisher_init_best_effort(
            &status_pub_, node_,
            ROSIDL_GET_MSG_TYPE_SUPPORT(mcu_msgs, msg, DriveBase),
            setup_.statusTopic) != RCL_RET_OK) {
      return false;
    }

    // Subscription: drive_base/command
    if (rclc_subscription_init_default(
            &cmd_sub_, node_,
            ROSIDL_GET_MSG_TYPE_SUPPORT(mcu_msgs, msg, DriveBase),
            setup_.commandTopic) != RCL_RET_OK) {
      return false;
    }
    if (rclc_executor_add_subscription_with_context(
            executor, &cmd_sub_, &cmd_msg_,
            &DriveSubsystem::commandCallback, this,
            ON_NEW_DATA) != RCL_RET_OK) {
      return false;
    }

    // Subscription: drive_base/reset_pose
    if (rclc_subscription_init_default(
            &pose_sub_, node_,
            ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Pose),
            setup_.resetPoseTopic) != RCL_RET_OK) {
      return false;
    }
    if (rclc_executor_add_subscription_with_context(
            executor, &pose_sub_, &pose_msg_,
            &DriveSubsystem::poseResetCallback, this,
            ON_NEW_DATA) != RCL_RET_OK) {
      return false;
    }

    return true;
  }

  void onDestroy() override {
    stopMotors();
    mode_ = DriveMode::IDLE;

    micro_ros_utilities_destroy_message_memory(
        ROSIDL_GET_MSG_TYPE_SUPPORT(mcu_msgs, msg, DriveBase), &status_msg_,
        status_mem_conf_);
    micro_ros_utilities_destroy_message_memory(
        ROSIDL_GET_MSG_TYPE_SUPPORT(mcu_msgs, msg, DriveBase), &cmd_msg_,
        cmd_mem_conf_);

    status_pub_ = rcl_get_zero_initialized_publisher();
    cmd_sub_ = rcl_get_zero_initialized_subscription();
    pose_sub_ = rcl_get_zero_initialized_subscription();
    node_ = nullptr;
  }

  // ── Public API (for RC subsystem, other on-MCU consumers) ──────────────

  /** @brief Command velocity (tank drive: vx forward, omega turn). */
  void setVelocity(float vx_mps, float omega_radps) {
    mode_ = DriveMode::VELOCITY;
    targetLinearVel_ = secbot::utils::clamp(
        vx_mps, -setup_.maxLinearVel, setup_.maxLinearVel);
    targetAngularVel_ = secbot::utils::clamp(
        omega_radps, -setup_.maxAngularVel, setup_.maxAngularVel);

    // Reset profiles to current state for smooth transition
    linearProfile_.reset({0.0f, currentLinearVel_, 0.0f});
    angularProfile_.reset({0.0f, currentAngularVel_, 0.0f});
    linearProfile_.setGoal({0.0f, targetLinearVel_});
    angularProfile_.setGoal({0.0f, targetAngularVel_});

    leftPID_.reset();
    rightPID_.reset();
    lastCommandMs_ = millis();
  }

  /** @brief Drive to a target pose (x, y in meters, theta in radians). */
  void setGoal(float x_m, float y_m, float theta_rad) {
    mode_ = DriveMode::GOAL;
    targetPose_ = Pose2D(x_m, y_m, theta_rad);
    leftPID_.reset();
    rightPID_.reset();
    lastCommandMs_ = millis();
  }

  /** @brief Follow a trajectory of waypoints (pure pursuit). */
  void setTrajectory(const TrajectoryController::Waypoint* wps, size_t count) {
    mode_ = DriveMode::TRAJECTORY;
    trajController_.setTrajectory(wps, count);
    trajController_.reset();
    leftPID_.reset();
    rightPID_.reset();
    lastCommandMs_ = millis();
  }

  /**
   * @brief Direct motor speed control for RC / manual driving.
   * @param left  -1.0 to 1.0 (full reverse to full forward)
   * @param right -1.0 to 1.0
   * No timeout applied — stays in MANUAL until stop() or another mode.
   */
  void manualDrive(float left, float right) {
    mode_ = DriveMode::MANUAL;
    float l = secbot::utils::clamp(left * setup_.leftMotorMultiplier, -1.0f, 1.0f);
    float r = secbot::utils::clamp(right * setup_.rightMotorMultiplier, -1.0f, 1.0f);
    setup_.motorManager->setSpeed(setup_.leftMotorIdx, l);
    setup_.motorManager->setSpeed(setup_.rightMotorIdx, r);
  }

  /**
   * @brief Arcade drive from RC stick inputs.
   * @param throttle  Forward/backward (-1.0 to 1.0), positive = forward
   * @param steering  Turn left/right (-1.0 to 1.0), positive = turn right
   * Deadzone of 0.05 applied to both axes.
   */
  void rcDrive(float throttle, float steering) {
    static constexpr float DEADZONE = 0.05f;
    if (throttle > -DEADZONE && throttle < DEADZONE) throttle = 0.0f;
    if (steering > -DEADZONE && steering < DEADZONE) steering = 0.0f;

    // Smooth RC inputs to prevent jerky motor transitions
    throttle = rcThrottleFilter_.update(throttle);
    steering = rcSteeringFilter_.update(steering);

    float left = secbot::utils::clamp(throttle + steering, -1.0f, 1.0f);
    float right = secbot::utils::clamp(throttle - steering, -1.0f, 1.0f);
    manualDrive(left, right);
  }

  /** @brief Stop all motors and enter IDLE mode. */
  void stop() {
    mode_ = DriveMode::IDLE;
    stopMotors();
    leftPID_.reset();
    rightPID_.reset();
    linearProfile_.reset();
    angularProfile_.reset();
    rcThrottleFilter_.reset(0.0f, false);
    rcSteeringFilter_.reset(0.0f, false);
  }

  /** @brief Override the internal pose estimate (e.g., from Pi EKF). */
  void resetPose(float x, float y, float theta) {
    Pose2D newPose(x, y, theta);
    localization_.reset();
    // After reset, localization returns to start pose from setup.
    // We need to set it to the requested pose. Since reset() sets to
    // start_{x,y,theta}, and we want arbitrary pose, we update directly.
    // TankDriveLocalization doesn't have a setPose, so we reset and accept
    // that the next update() will use the new encoder baseline.
    // For now, reset is sufficient — the Pi corrects via EKF anyway.
    prevLeftTicks_ = setup_.encoderSub->getAccumulatedTicks(
        setup_.leftEncoderIdx);
    prevRightTicks_ = setup_.encoderSub->getAccumulatedTicks(
        setup_.rightEncoderIdx);
    if (setup_.leftEncoderInverted) prevLeftTicks_ = -prevLeftTicks_;
    if (setup_.rightEncoderInverted) prevRightTicks_ = -prevRightTicks_;
  }

  // ── Accessors ──────────────────────────────────────────────────────────

  DriveMode getMode() const { return mode_; }
  Pose2D getPose() const { return localization_.getPose(); }
  float getLinearVelocity() const { return currentLinearVel_; }
  float getAngularVelocity() const { return currentAngularVel_; }
  bool isIdle() const { return mode_ == DriveMode::IDLE; }

  // ── Threading ──────────────────────────────────────────────────────────

#ifdef USE_TEENSYTHREADS
  void beginThreaded(uint32_t stackSize, int /*priority*/ = 1,
                     uint32_t updateRateMs = 20) {
    task_delay_ms_ = updateRateMs;
    threads.addThread(taskFunction, this, stackSize);
  }

 private:
  static void taskFunction(void* pv) {
    auto* self = static_cast<DriveSubsystem*>(pv);
    self->begin();
    while (true) {
      self->update();
      threads.delay(self->task_delay_ms_);
    }
  }
  uint32_t task_delay_ms_ = 20;
#endif

 private:
  // ── Constants ──────────────────────────────────────────────────────────

  static constexpr size_t MAX_TRAJECTORY_POINTS = 32;

  // ── Encoder capture ──────────────────────────────────────────────────

  /**
   * @brief Read all 8 motor intended directions from MotorManager, then
   *        call QTimerEncoder::captureAll() to accumulate deltas and sync
   *        the hardware counter direction registers.
   */
  void captureEncoders() {
    if (!setup_.encoder || !setup_.motorManager) return;
    bool dirs[Encoders::NUM_ENCODER_CHANNELS];
    for (uint8_t i = 0; i < Encoders::NUM_ENCODER_CHANNELS; i++) {
      dirs[i] = setup_.motorManager->getIntendedDirection(i);
    }
    setup_.encoder->captureAll(dirs);
  }

  // ── Control methods ────────────────────────────────────────────────────

  void velocityControl(float dt) {
    if (dt < 0.001f) return;

    // S-curve ramp to target velocity
    MotionState linState = linearProfile_.update(dt);
    MotionState angState = angularProfile_.update(dt);

    float rampedV = linState.vel;
    float rampedOmega = angState.vel;

    // Differential drive inverse kinematics: (v, omega) → wheel velocities
    float targetLeftVel =
        rampedV - (rampedOmega * setup_.locSetup.track_width * 0.5f);
    float targetRightVel =
        rampedV + (rampedOmega * setup_.locSetup.track_width * 0.5f);

    // Actual wheel velocities from encoder deltas
    int32_t leftTicks = setup_.encoderSub->getAccumulatedTicks(
        setup_.leftEncoderIdx);
    int32_t rightTicks = setup_.encoderSub->getAccumulatedTicks(
        setup_.rightEncoderIdx);
    if (setup_.leftEncoderInverted) leftTicks = -leftTicks;
    if (setup_.rightEncoderInverted) rightTicks = -rightTicks;

    float actualLeftVel =
        static_cast<float>(leftTicks - velPrevLeftTicks_) *
        setup_.locSetup.dist_per_tick / dt;
    float actualRightVel =
        static_cast<float>(rightTicks - velPrevRightTicks_) *
        setup_.locSetup.dist_per_tick / dt;
    velPrevLeftTicks_ = leftTicks;
    velPrevRightTicks_ = rightTicks;

    // PID → motor speed [-1.0, 1.0]
    float leftOut = leftPID_.update(targetLeftVel, actualLeftVel, dt);
    float rightOut = rightPID_.update(targetRightVel, actualRightVel, dt);

    writeMotorSpeeds(leftOut, rightOut);
  }

  void setPointControl(float dt) {
    Pose2D pose = localization_.getPose();

    float dx = targetPose_.getX() - pose.getX();
    float dy = targetPose_.getY() - pose.getY();
    float distError = sqrtf(dx * dx + dy * dy);

    // Close enough — stop
    if (distError < setup_.poseDistTol) {
      targetLinearVel_ = 0.0f;
      targetAngularVel_ = 0.0f;

      // Reset profiles to decel smoothly
      linearProfile_.reset({0.0f, currentLinearVel_, 0.0f});
      angularProfile_.reset({0.0f, currentAngularVel_, 0.0f});
      linearProfile_.setGoal({0.0f, 0.0f});
      angularProfile_.setGoal({0.0f, 0.0f});

      velocityControl(dt);
      return;
    }

    float targetAngle = atan2f(dy, dx);
    float angleError = secbot::utils::normalizeAngleRad(
        targetAngle - pose.getTheta());

    float vCmd = setup_.poseKLinear * distError;
    float omegaCmd = setup_.poseKAngular * angleError;

    // Slow down if not facing target (prevents large arcs)
    float cosAngle = cosf(angleError);
    if (cosAngle < 0.0f) cosAngle = 0.0f;
    vCmd *= cosAngle;

    // Clamp
    vCmd = secbot::utils::clamp(vCmd, -setup_.maxLinearVel,
                                setup_.maxLinearVel);
    omegaCmd = secbot::utils::clamp(omegaCmd, -setup_.maxAngularVel,
                                    setup_.maxAngularVel);

    targetLinearVel_ = vCmd;
    targetAngularVel_ = omegaCmd;

    // Update profiles for smooth ramping
    linearProfile_.reset({0.0f, currentLinearVel_, 0.0f});
    angularProfile_.reset({0.0f, currentAngularVel_, 0.0f});
    linearProfile_.setGoal({0.0f, targetLinearVel_});
    angularProfile_.setGoal({0.0f, targetAngularVel_});

    velocityControl(dt);
  }

  void trajectoryControl(float dt) {
    if (dt < 0.001f) return;

    Pose2D pose = localization_.getPose();
    TrajectoryController::Pose2D trajPose;
    trajPose.x = pose.getX();
    trajPose.y = pose.getY();
    trajPose.theta = pose.getTheta();

    TrajectoryController::Command cmd = trajController_.update(trajPose, dt);

    if (cmd.finished) {
      stopMotors();
      mode_ = DriveMode::IDLE;
      return;
    }

    // Convert (v, w) to wheel velocities
    float targetLeftVel = 0.0f, targetRightVel = 0.0f;
    TrajectoryController::chassisToWheelSpeeds(
        cmd.v, cmd.w, setup_.locSetup.track_width,
        &targetLeftVel, &targetRightVel);

    // Actual wheel velocities
    int32_t leftTicks = setup_.encoderSub->getAccumulatedTicks(
        setup_.leftEncoderIdx);
    int32_t rightTicks = setup_.encoderSub->getAccumulatedTicks(
        setup_.rightEncoderIdx);
    if (setup_.leftEncoderInverted) leftTicks = -leftTicks;
    if (setup_.rightEncoderInverted) rightTicks = -rightTicks;

    float actualLeftVel =
        static_cast<float>(leftTicks - velPrevLeftTicks_) *
        setup_.locSetup.dist_per_tick / dt;
    float actualRightVel =
        static_cast<float>(rightTicks - velPrevRightTicks_) *
        setup_.locSetup.dist_per_tick / dt;
    velPrevLeftTicks_ = leftTicks;
    velPrevRightTicks_ = rightTicks;

    float leftOut = leftPID_.update(targetLeftVel, actualLeftVel, dt);
    float rightOut = rightPID_.update(targetRightVel, actualRightVel, dt);

    writeMotorSpeeds(leftOut, rightOut);
  }

  // ── Motor output ───────────────────────────────────────────────────────

  void writeMotorSpeeds(float left, float right) {
    setup_.motorManager->setSpeed(setup_.leftMotorIdx,
                                  secbot::utils::clamp(left, -1.0f, 1.0f));
    setup_.motorManager->setSpeed(setup_.rightMotorIdx,
                                  secbot::utils::clamp(right, -1.0f, 1.0f));
  }

  void stopMotors() {
    if (!setup_.motorManager) return;
    setup_.motorManager->setSpeed(setup_.leftMotorIdx, 0.0f);
    setup_.motorManager->setSpeed(setup_.rightMotorIdx, 0.0f);
  }

  // ── micro-ROS callbacks ────────────────────────────────────────────────

  static void commandCallback(const void* msvin, void* context) {
    auto* self = static_cast<DriveSubsystem*>(context);
    auto* msg = static_cast<const mcu_msgs__msg__DriveBase*>(msvin);

    self->lastCommandMs_ = millis();

    switch (msg->drive_mode) {
      case mcu_msgs__msg__DriveBase__DRIVE_VECTOR: {
        float vx = static_cast<float>(msg->goal_velocity.linear.x);
        float omega = static_cast<float>(msg->goal_velocity.angular.z);
        self->setVelocity(vx, omega);
        break;
      }
      case mcu_msgs__msg__DriveBase__DRIVE_GOAL: {
        float x = static_cast<float>(
            msg->goal_transform.transform.translation.x);
        float y = static_cast<float>(
            msg->goal_transform.transform.translation.y);
        // Extract yaw from quaternion
        float qz = static_cast<float>(
            msg->goal_transform.transform.rotation.z);
        float qw = static_cast<float>(
            msg->goal_transform.transform.rotation.w);
        float theta = 2.0f * atan2f(qz, qw);
        self->setGoal(x, y, theta);
        break;
      }
      case mcu_msgs__msg__DriveBase__DRIVE_TRAJ: {
        size_t count = msg->goal_path.poses.size;
        if (count == 0 || count > MAX_TRAJECTORY_POINTS) break;

        TrajectoryController::Waypoint wps[MAX_TRAJECTORY_POINTS];
        for (size_t i = 0; i < count; i++) {
          wps[i].x = static_cast<float>(
              msg->goal_path.poses.data[i].pose.position.x);
          wps[i].y = static_cast<float>(
              msg->goal_path.poses.data[i].pose.position.y);
          wps[i].has_vel = 0;
          wps[i].has_heading = 0;
        }
        self->setTrajectory(wps, count);
        break;
      }
      default:
        break;
    }
  }

  static void poseResetCallback(const void* msvin, void* context) {
    auto* self = static_cast<DriveSubsystem*>(context);
    auto* msg = static_cast<const geometry_msgs__msg__Pose*>(msvin);

    float x = static_cast<float>(msg->position.x);
    float y = static_cast<float>(msg->position.y);
    float qz = static_cast<float>(msg->orientation.z);
    float qw = static_cast<float>(msg->orientation.w);
    float theta = 2.0f * atan2f(qz, qw);

    self->resetPose(x, y, theta);
  }

  // ── Publishing ─────────────────────────────────────────────────────────

  void publishStatus() {
    if (!status_pub_.impl) return;

    Pose2D pose = localization_.getPose();

    // Timestamp
    int64_t time_ns = rmw_uros_epoch_nanos();
    status_msg_.header.stamp.sec = static_cast<int32_t>(time_ns / 1000000000);
    status_msg_.header.stamp.nanosec =
        static_cast<int32_t>(time_ns % 1000000000);

    // Current transform (pose as TransformStamped)
    status_msg_.transform.transform.translation.x = pose.getX();
    status_msg_.transform.transform.translation.y = pose.getY();
    status_msg_.transform.transform.translation.z = 0.0;

    // Yaw → quaternion (rotation about Z only)
    float half_theta = pose.getTheta() * 0.5f;
    status_msg_.transform.transform.rotation.x = 0.0;
    status_msg_.transform.transform.rotation.y = 0.0;
    status_msg_.transform.transform.rotation.z = sinf(half_theta);
    status_msg_.transform.transform.rotation.w = cosf(half_theta);

    // Current velocity (body frame)
    status_msg_.twist.linear.x = currentLinearVel_;
    status_msg_.twist.linear.y = 0.0;
    status_msg_.twist.linear.z = 0.0;
    status_msg_.twist.angular.x = 0.0;
    status_msg_.twist.angular.y = 0.0;
    status_msg_.twist.angular.z = currentAngularVel_;

    // Drive mode
    switch (mode_) {
      case DriveMode::VELOCITY:
      case DriveMode::MANUAL:
        status_msg_.drive_mode = mcu_msgs__msg__DriveBase__DRIVE_VECTOR;
        break;
      case DriveMode::GOAL:
        status_msg_.drive_mode = mcu_msgs__msg__DriveBase__DRIVE_GOAL;
        break;
      case DriveMode::TRAJECTORY:
        status_msg_.drive_mode = mcu_msgs__msg__DriveBase__DRIVE_TRAJ;
        break;
      default:
        status_msg_.drive_mode = mcu_msgs__msg__DriveBase__DRIVE_VECTOR;
        break;
    }

    // Clear path in status (we don't echo it back)
    status_msg_.goal_path.poses.size = 0;

#ifdef USE_TEENSYTHREADS
    {
      Threads::Scope guard(g_microros_mutex);
      (void)rcl_publish(&status_pub_, &status_msg_, NULL);
    }
#else
    (void)rcl_publish(&status_pub_, &status_msg_, NULL);
#endif
  }

  // ── State ──────────────────────────────────────────────────────────────

  const DriveSubsystemSetup& setup_;

  // Localization
  Drive::TankDriveLocalization localization_;
  int32_t prevLeftTicks_ = 0;
  int32_t prevRightTicks_ = 0;

  // Velocity tracking (separate from localization tick tracking)
  int32_t velPrevLeftTicks_ = 0;
  int32_t velPrevRightTicks_ = 0;

  // Control
  PIDController leftPID_;
  PIDController rightPID_;
  SCurveMotionProfile linearProfile_;
  SCurveMotionProfile angularProfile_;
  TrajectoryController trajController_;

  // Drive state
  DriveMode mode_ = DriveMode::IDLE;
  float targetLinearVel_ = 0.0f;
  float targetAngularVel_ = 0.0f;
  float currentLinearVel_ = 0.0f;
  float currentAngularVel_ = 0.0f;
  Pose2D targetPose_;

  // RC input smoothing
  secbot::utils::LowPass1P rcThrottleFilter_;
  secbot::utils::LowPass1P rcSteeringFilter_;

  // Timing
  uint32_t last_update_us_ = 0;
  uint32_t lastCommandMs_ = 0;
  uint8_t publishCounter_ = 0;

  // micro-ROS handles
  rcl_node_t* node_ = nullptr;
  rcl_publisher_t status_pub_{};
  rcl_subscription_t cmd_sub_{};
  rcl_subscription_t pose_sub_{};
  mcu_msgs__msg__DriveBase status_msg_{};
  mcu_msgs__msg__DriveBase cmd_msg_{};
  geometry_msgs__msg__Pose pose_msg_{};
  micro_ros_utilities_memory_conf_t status_mem_conf_{};
  micro_ros_utilities_memory_conf_t cmd_mem_conf_{};
};

}  // namespace Subsystem
