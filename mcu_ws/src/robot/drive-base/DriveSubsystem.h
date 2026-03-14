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

#include <Pose2D.h>
#include <TankDriveLocalization.h>
#include <Vector2D.h>
#include <geometry_msgs/msg/pose.h>
#include <filters.h>
#include <math_utils.h>
#include <mcu_msgs/msg/drive_base.h>
#include <mcu_msgs/srv/set_drive_config.h>
#include <micro_ros_utilities/type_utilities.h>
#include <microros_manager_robot.h>
#include <motion_profile.h>
#include <pid_controller.h>
#include <traj_controller.h>

#include "TimedSubsystem.h"
#include "robot/subsystems/EncoderSubsystem.h"
#include "robot/subsystems/ImuSubsystem.h"
#include "robot/subsystems/MotorManagerSubsystem.h"

namespace Subsystem {

// ═══════════════════════════════════════════════════════════════════════════
//  Drive modes
// ═══════════════════════════════════════════════════════════════════════════

enum class DriveMode : uint8_t {
  IDLE,            // Motors off, no active control
  VELOCITY,        // Twist (vx, omega) → S-curve → wheel PIDs (gyro hold)
  VELOCITY_RAW,    // Twist (vx, omega) → S-curve → wheel PIDs (no gyro hold)
  GOAL,            // Drive to (x, y, theta) pose
  TRAJECTORY,      // Follow waypoint path (pure pursuit)
  MANUAL           // Direct motor speed passthrough (for RC)
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
  float maxLinearVel = 0.7f;     // m/s — chassis speed limit
  float maxAngularVel = 6.0f;    // rad/s
  float maxWheelVel = 0.73f;     // m/s — effective wheel limit (min of physical & user)

  // Pose drive gains
  float poseKLinear = 2.0f;
  float poseKAngular = 4.0f;
  float poseDistTol = 0.015f;    // meters
  float poseHeadingTol = 0.05f;  // radians (~3 deg)

  // Gyro heading hold: corrects drift when commanding straight-line velocity.
  // When |targetAngularVel| < threshold, locks current heading and applies
  // a P-correction to omega. Set kp=0 to disable.
  float gyroHoldKp = 2.0f;                 // P gain for heading correction
  float gyroHoldThreshold = 0.05f;         // rad/s — below this, hold heading

  // Motor direction multipliers (1.0 or -1.0 to reverse a motor)
  float leftMotorMultiplier = -1.0f;
  float rightMotorMultiplier = 1.0f;

  // Safety
  uint32_t commandTimeoutMs = 500;

  // Topic names
  const char* statusTopic = "drive_base/status";
  const char* commandTopic = "drive_base/command";
  const char* resetPoseTopic = "drive_base/reset_pose";
  const char* configServiceName = "/mcu_robot/drive/config";

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

    // cmd_vel input smoothing: same tau/dt as RC for consistent feel
    cmdVelLinearFilter_.configureTauDtFast(0.15f, 0.005f);
    cmdVelAngularFilter_.configureTauDtFast(0.15f, 0.005f);

    // Motor output smoothing: tau=0.05s at drive loop dt=0.02s (50 Hz)
    // Snappier than RC (tau=0.15s) — settles in ~3 time constants (~150ms)
    motorLeftFilter_.configureTauDtFast(0.05f, 0.02f);
    motorRightFilter_.configureTauDtFast(0.05f, 0.02f);

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

    // ── Command timeout check (applies to all active modes including MANUAL) ──
    if (mode_ != DriveMode::IDLE) {
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
      {
        Threads::Scope lock(data_mutex_);
        publishStatus();
        data_ready_ = true;
      }
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
    velLinAccel_ = 0.0f;
    velAngAccel_ = 0.0f;
    velRampedLinear_ = 0.0f;
    velRampedAngular_ = 0.0f;
    goalLinAccel_ = 0.0f;
    goalAngAccel_ = 0.0f;
    goalRampedLinear_ = 0.0f;
    goalRampedAngular_ = 0.0f;
  }

  const char* getInfo() override {
    static const char info[] = "DriveSubsystem";
    return info;
  }

  // ── IMicroRosParticipant ────────────────────────────────────────────────

  bool onCreate(rcl_node_t* node, rclc_executor_t* executor) override {
    node_ = node;
    DEBUG_PRINTLN("[DRIVE] onCreate: start");

    // Initialize status message memory (handles variable-length fields)
    status_mem_conf_ = {};
    status_mem_conf_.max_string_capacity = 20;
    status_mem_conf_.max_ros2_type_sequence_capacity = 0;
    status_mem_conf_.max_basic_type_sequence_capacity = 0;
    if (!micro_ros_utilities_create_message_memory(
            ROSIDL_GET_MSG_TYPE_SUPPORT(mcu_msgs, msg, DriveBase),
            &status_msg_, status_mem_conf_)) {
      DEBUG_PRINTLN("[DRIVE] FAIL: status msg memory");
      return false;
    }
    DEBUG_PRINTLN("[DRIVE] onCreate: status msg OK");

    // Initialize command message memory (needs path capacity for TRAJ mode)
    cmd_mem_conf_ = {};
    cmd_mem_conf_.max_string_capacity = 20;
    cmd_mem_conf_.max_ros2_type_sequence_capacity = MAX_TRAJECTORY_POINTS;
    cmd_mem_conf_.max_basic_type_sequence_capacity = 0;
    if (!micro_ros_utilities_create_message_memory(
            ROSIDL_GET_MSG_TYPE_SUPPORT(mcu_msgs, msg, DriveBase),
            &cmd_msg_, cmd_mem_conf_)) {
      DEBUG_PRINTLN("[DRIVE] FAIL: cmd msg memory");
      return false;
    }
    DEBUG_PRINTLN("[DRIVE] onCreate: cmd msg OK");

    // Publisher: drive_base/status
    if (rclc_publisher_init_best_effort(
            &status_pub_, node_,
            ROSIDL_GET_MSG_TYPE_SUPPORT(mcu_msgs, msg, DriveBase),
            setup_.statusTopic) != RCL_RET_OK) {
      DEBUG_PRINTLN("[DRIVE] FAIL: status pub");
      return false;
    }
    DEBUG_PRINTLN("[DRIVE] onCreate: pub OK");

    // Subscription: drive_base/command
    if (rclc_subscription_init_default(
            &cmd_sub_, node_,
            ROSIDL_GET_MSG_TYPE_SUPPORT(mcu_msgs, msg, DriveBase),
            setup_.commandTopic) != RCL_RET_OK) {
      DEBUG_PRINTLN("[DRIVE] FAIL: cmd sub");
      return false;
    }
    if (rclc_executor_add_subscription_with_context(
            executor, &cmd_sub_, &cmd_msg_,
            &DriveSubsystem::commandCallback, this,
            ON_NEW_DATA) != RCL_RET_OK) {
      DEBUG_PRINTLN("[DRIVE] FAIL: cmd sub executor");
      return false;
    }
    DEBUG_PRINTLN("[DRIVE] onCreate: cmd sub OK");

    // Subscription: drive_base/reset_pose
    if (rclc_subscription_init_default(
            &pose_sub_, node_,
            ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Pose),
            setup_.resetPoseTopic) != RCL_RET_OK) {
      DEBUG_PRINTLN("[DRIVE] FAIL: pose sub");
      return false;
    }
    if (rclc_executor_add_subscription_with_context(
            executor, &pose_sub_, &pose_msg_,
            &DriveSubsystem::poseResetCallback, this,
            ON_NEW_DATA) != RCL_RET_OK) {
      DEBUG_PRINTLN("[DRIVE] FAIL: pose sub executor");
      return false;
    }
    DEBUG_PRINTLN("[DRIVE] onCreate: pose sub OK");

    // Service: drive config tuning
    if (rclc_service_init_default(
            &config_srv_, node_,
            ROSIDL_GET_SRV_TYPE_SUPPORT(mcu_msgs, srv, SetDriveConfig),
            setup_.configServiceName) != RCL_RET_OK) {
      DEBUG_PRINTLN("[DRIVE] FAIL: config srv");
      return false;
    }
    if (rclc_executor_add_service_with_context(
            executor, &config_srv_, &config_req_, &config_res_,
            &DriveSubsystem::configCallback, this) != RCL_RET_OK) {
      DEBUG_PRINTLN("[DRIVE] FAIL: config srv executor");
      return false;
    }
    DEBUG_PRINTLN("[DRIVE] onCreate: config srv OK");

    DEBUG_PRINTLN("[DRIVE] onCreate OK");
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
    config_srv_ = rcl_get_zero_initialized_service();
    node_ = nullptr;
  }

  // ── Public API (for RC subsystem, other on-MCU consumers) ──────────────

  /** @brief Command velocity (tank drive: vx forward, omega turn). */
  void setVelocity(float vx_mps, float omega_radps,
                    DriveMode vel_mode = DriveMode::VELOCITY) {
    bool modeChange = (mode_ != vel_mode);
    mode_ = vel_mode;
    targetLinearVel_ = secbot::utils::clamp(
        vx_mps, -setup_.maxLinearVel, setup_.maxLinearVel);
    targetAngularVel_ = secbot::utils::clamp(
        omega_radps, -setup_.maxAngularVel, setup_.maxAngularVel);

    // Reset heading hold so it re-captures on next straight-line command
    gyroHoldActive_ = false;

    if (modeChange) {
      velLinAccel_ = 0.0f;
      velAngAccel_ = 0.0f;
      // Seed ramp at current measured velocity so we don't jump
      velRampedLinear_ = currentLinearVel_;
      velRampedAngular_ = currentAngularVel_;
      syncVelTicks();
      leftPID_.reset();
      rightPID_.reset();
    }
    lastCommandMs_ = millis();
  }

  /** @brief Drive to a target pose (x, y in meters, theta in radians).
   *  @param reverse If true, drive backwards to the goal. */
  void setGoal(float x_m, float y_m, float theta_rad, bool reverse = false) {
    mode_ = DriveMode::GOAL;
    reverse_ = reverse;
    targetPose_ = Pose2D(x_m, y_m, theta_rad);
    goalLinAccel_ = 0.0f;
    goalAngAccel_ = 0.0f;
    goalRampedLinear_ = currentLinearVel_;
    goalRampedAngular_ = currentAngularVel_;
    syncVelTicks();
    leftPID_.reset();
    rightPID_.reset();
    lastCommandMs_ = millis();
  }

  /** @brief Follow a trajectory of waypoints (pure pursuit).
   *  Copies waypoints into internal storage — caller's array can be temporary. */
  void setTrajectory(const TrajectoryController::Waypoint* wps, size_t count) {
    if (!wps || count == 0) return;
    if (count > MAX_TRAJECTORY_POINTS) count = MAX_TRAJECTORY_POINTS;

    // Copy into persistent storage (fixes use-after-free from stack-local arrays)
    for (size_t i = 0; i < count; i++) trajWaypoints_[i] = wps[i];
    trajWaypointCount_ = count;

    mode_ = DriveMode::TRAJECTORY;
    trajController_.setTrajectory(trajWaypoints_, trajWaypointCount_);
    trajController_.reset();
    syncVelTicks();
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
    writeMotorSpeeds(left, right);
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

    float left = throttle + steering;
    float right = throttle - steering;

    // Proportional saturation: if either exceeds [-1,1], scale both to preserve
    // the left/right ratio (curvature). Independent clamping distorts the ratio.
    float maxAbs = fmaxf(fabsf(left), fabsf(right));
    if (maxAbs > 1.0f) {
      float scale = 1.0f / maxAbs;
      left *= scale;
      right *= scale;
    }

    // Scale duty so full stick ≈ configured max velocity (not physical max)
    float dutyScale = fminf(1.0f, setup_.maxLinearVel / setup_.maxWheelVel);
    left *= dutyScale;
    right *= dutyScale;

    // Gyro hold: when steering is zero, correct drift
    applyManualGyroHold(steering, left, right);

    lastCommandMs_ = millis();
    manualDrive(left, right);
  }

  /**
   * @brief Normalized arcade drive (for MCU-side callers, NOT ROS2 commands).
   * @param linear  Forward/backward (-1.0 to 1.0), positive = forward
   * @param angular Turn rate (-1.0 to 1.0), positive = turn left (CCW)
   * Uses low-pass filter smoothing. For ROS2 DRIVE_VECTOR commands (m/s),
   * use setVelocity() instead — the command callback handles this.
   */
  void cmdVelDrive(float linear, float angular) {
    linear = secbot::utils::clamp(linear, -1.0f, 1.0f);
    angular = secbot::utils::clamp(angular, -1.0f, 1.0f);

    static constexpr float DEADZONE = 0.05f;
    if (linear > -DEADZONE && linear < DEADZONE) linear = 0.0f;
    if (angular > -DEADZONE && angular < DEADZONE) angular = 0.0f;

    // Smooth cmd_vel inputs with same low-pass filter as RC
    linear = cmdVelLinearFilter_.update(linear);
    angular = cmdVelAngularFilter_.update(angular);

    // Differential drive mix: positive angular = CCW = left slower, right faster
    float left = linear - angular;
    float right = linear + angular;

    // Proportional saturation to preserve curvature
    float maxAbs = fmaxf(fabsf(left), fabsf(right));
    if (maxAbs > 1.0f) {
      float scale = 1.0f / maxAbs;
      left *= scale;
      right *= scale;
    }

    // Scale duty so full input ≈ configured max velocity (not physical max)
    float dutyScale = fminf(1.0f, setup_.maxLinearVel / setup_.maxWheelVel);
    left *= dutyScale;
    right *= dutyScale;

    // Gyro hold: when angular is zero, correct drift
    applyManualGyroHold(angular, left, right);

    lastCommandMs_ = millis();
    manualDrive(left, right);
  }

  /** @brief Stop all motors and enter IDLE mode. */
  void stop() {
    mode_ = DriveMode::IDLE;
    gyroHoldActive_ = false;
    stopMotors();
    leftPID_.reset();
    rightPID_.reset();
    linearProfile_.reset();
    angularProfile_.reset();
    rcThrottleFilter_.reset(0.0f, false);
    rcSteeringFilter_.reset(0.0f, false);
    cmdVelLinearFilter_.reset(0.0f, false);
    cmdVelAngularFilter_.reset(0.0f, false);
    motorLeftFilter_.reset(0.0f, false);
    motorRightFilter_.reset(0.0f, false);
  }

  /** @brief Override the internal pose estimate (e.g., from Pi EKF). */
  void resetPose(float x, float y, float theta) {
    localization_.setPose(x, y, theta);

    // Sync tick baselines to current values so the next update() computes
    // zero delta (no position jump from stale encoder history).
    int32_t leftTicks = setup_.encoderSub->getAccumulatedTicks(
        setup_.leftEncoderIdx);
    int32_t rightTicks = setup_.encoderSub->getAccumulatedTicks(
        setup_.rightEncoderIdx);
    if (setup_.leftEncoderInverted) leftTicks = -leftTicks;
    if (setup_.rightEncoderInverted) rightTicks = -rightTicks;

    prevLeftTicks_ = leftTicks;
    prevRightTicks_ = rightTicks;
    velPrevLeftTicks_ = leftTicks;
    velPrevRightTicks_ = rightTicks;
  }

  // ── Accessors ──────────────────────────────────────────────────────────

  DriveMode getMode() const { return mode_; }
  Pose2D getPose() const { return localization_.getPose(); }
  float getLinearVelocity() const { return currentLinearVel_; }
  float getAngularVelocity() const { return currentAngularVel_; }
  bool isIdle() const { return mode_ == DriveMode::IDLE; }

 private:
  // ── Constants ──────────────────────────────────────────────────────────

  static constexpr size_t MAX_TRAJECTORY_POINTS = 32;

  // ── Gyro hold for manual/RC modes ──────────────────────────────────

  /**
   * @brief Apply gyro heading hold to manual-mode left/right motor outputs.
   * @param steeringInput The steering/angular input (pre-mix). If ~0, hold heading.
   * @param left  [in/out] Left motor command (-1 to 1).
   * @param right [in/out] Right motor command (-1 to 1).
   *
   * Converts the gyro heading error to a normalized differential correction
   * and applies it to the motor split. Lightweight — no PID, just P on heading.
   */
  void applyManualGyroHold(float steeringInput, float& left, float& right) {
    if (setup_.gyroHoldKp <= 0.0f || !setup_.imuSub) return;

    if (fabsf(steeringInput) < 0.01f) {
      float yaw = setup_.imuSub->getYaw();
      if (yaw != yaw) return;  // NaN guard — IMU fault or startup
      if (!gyroHoldActive_) {
        gyroHoldHeading_ = yaw;
        gyroHoldActive_ = true;
      }
      float headingErr = secbot::utils::normalizeAngleRad(
          gyroHoldHeading_ - yaw);
      // Scale rad/s correction to normalized [-1,1] motor range
      float correction = (setup_.gyroHoldKp * headingErr) /
                          setup_.maxAngularVel;
      correction = secbot::utils::clamp(correction, -0.3f, 0.3f);
      left = secbot::utils::clamp(left - correction, -1.0f, 1.0f);
      right = secbot::utils::clamp(right + correction, -1.0f, 1.0f);
    } else {
      gyroHoldActive_ = false;
    }
  }

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

  /** @brief Sync velocity tick trackers to current ticks (call on mode entry). */
  void syncVelTicks() {
    velPrevLeftTicks_ = prevLeftTicks_;
    velPrevRightTicks_ = prevRightTicks_;
  }

  /**
   * @brief Proportionally scale wheel velocities if either exceeds the
   *        physical motor limit. Preserves the left/right ratio (curvature).
   */
  void saturateWheelSpeeds(float& left, float& right) {
    float maxAbs = fmaxf(fabsf(left), fabsf(right));
    if (maxAbs > setup_.maxWheelVel) {
      float scale = setup_.maxWheelVel / maxAbs;
      left *= scale;
      right *= scale;
    }
  }

  /**
   * @brief Compute velocity feedforward: maps target wheel velocity to
   *        an approximate motor duty [-1, 1].
   *
   * Without feedforward the PID must generate the entire motor command from
   * error alone.  With Kp=0.8, a 0.2 m/s target only produces 0.16 duty —
   * well below the NFPShop motor deadband (~0.3).  Feedforward provides
   * the baseline duty so the PID only corrects small errors.
   *
   * feedforward = targetVel / maxWheelVel  (linear model: duty 1.0 ≈ max speed)
   */
  float velocityFeedforward(float targetWheelVel) const {
    if (setup_.maxWheelVel <= 0.0f) return 0.0f;
    return targetWheelVel / setup_.maxWheelVel;
  }

  /**
   * @brief Jerk-limited velocity ramp (velocity-only, no position logic).
   *
   * The S-curve profile uses position-based braking which is degenerate when
   * used as a velocity shaper (pos=0/goal=0 → always brakes). This helper
   * does pure velocity tracking with jerk and acceleration limits.
   *
   * @param current   Current velocity (m/s or rad/s)
   * @param target    Desired velocity
   * @param accel     [in/out] Current acceleration state (preserved across calls)
   * @param a_max     Max acceleration magnitude
   * @param d_max     Max deceleration magnitude
   * @param j_max     Max jerk (rate of change of acceleration)
   * @param v_max     Max velocity magnitude (clamp output)
   * @param dt        Time step
   * @return          Ramped velocity
   */
  static float rampVelocity(float current, float target, float& accel,
                             float a_max, float d_max, float j_max,
                             float v_max, float dt) {
    float dv = target - current;

    // Choose acceleration or deceleration limit
    float a_target;
    if (fabsf(dv) < 1e-4f) {
      a_target = 0.0f;
    } else if (dv > 0.0f) {
      a_target = a_max;
    } else {
      a_target = -d_max;
    }

    // Jerk-limited approach to a_target
    if (j_max > 0.0f) {
      float da = a_target - accel;
      float maxStep = j_max * dt;
      if (da > maxStep) da = maxStep;
      if (da < -maxStep) da = -maxStep;
      accel += da;
    } else {
      accel = a_target;
    }

    float v_next = current + accel * dt;

    // Don't overshoot target velocity
    if ((target > current && v_next > target) ||
        (target < current && v_next < target)) {
      v_next = target;
      accel = 0.0f;
    }

    // Clamp to max velocity
    if (v_next > v_max) { v_next = v_max; if (accel > 0.0f) accel = 0.0f; }
    if (v_next < -v_max) { v_next = -v_max; if (accel < 0.0f) accel = 0.0f; }

    return v_next;
  }

  // ── Control methods ────────────────────────────────────────────────────

  void velocityControl(float dt) {
    if (dt < 0.001f) return;

    // Jerk-limited velocity ramp using persisted ramp output (not measured
    // velocity).  This lets the ramp accumulate through the motor deadband
    // even when the robot hasn't started moving yet.
    auto& linLimits = setup_.linearProfile.limits;
    auto& angLimits = setup_.angularProfile.limits;
    velRampedLinear_ = rampVelocity(
        velRampedLinear_, targetLinearVel_, velLinAccel_,
        linLimits.a_max, linLimits.d_max, linLimits.j_max,
        linLimits.v_max, dt);
    velRampedAngular_ = rampVelocity(
        velRampedAngular_, targetAngularVel_, velAngAccel_,
        angLimits.a_max, angLimits.d_max, angLimits.j_max,
        angLimits.v_max, dt);
    float rampedV = velRampedLinear_;
    float rampedOmega = velRampedAngular_;

    // Gyro heading hold: when commanding straight-line driving (omega ≈ 0),
    // lock the current heading and apply a small correction to prevent drift.
    // Skipped in VELOCITY_RAW mode (e.g. nudge — no heading correction wanted).
    if (mode_ != DriveMode::VELOCITY_RAW &&
        setup_.gyroHoldKp > 0.0f &&
        fabsf(targetAngularVel_) < setup_.gyroHoldThreshold) {
      float yaw = setup_.imuSub->getYaw();
      if (yaw == yaw) {  // NaN guard — skip if IMU fault
        if (!gyroHoldActive_) {
          gyroHoldHeading_ = yaw;
          gyroHoldActive_ = true;
        }
        float headingErr = secbot::utils::normalizeAngleRad(
            gyroHoldHeading_ - yaw);
        float correction = secbot::utils::clamp(
            setup_.gyroHoldKp * headingErr,
            -setup_.maxAngularVel * 0.3f, setup_.maxAngularVel * 0.3f);
        rampedOmega += correction;
      }
    } else {
      gyroHoldActive_ = false;
    }

    // Differential drive inverse kinematics: (v, omega) → wheel velocities
    float targetLeftVel =
        rampedV - (rampedOmega * setup_.locSetup.track_width * 0.5f);
    float targetRightVel =
        rampedV + (rampedOmega * setup_.locSetup.track_width * 0.5f);

    // Saturate: if either wheel exceeds physical motor limit, scale both
    // proportionally to preserve curvature (turn radius).
    saturateWheelSpeeds(targetLeftVel, targetRightVel);

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

    // PID + feedforward → motor speed [-1.0, 1.0]
    // Feedforward maps target wheel vel to approximate duty so the PID
    // doesn't have to generate the entire command from error alone.
    float leftFF = velocityFeedforward(targetLeftVel);
    float rightFF = velocityFeedforward(targetRightVel);
    float leftOut = leftPID_.update(targetLeftVel, actualLeftVel, dt, leftFF);
    float rightOut = rightPID_.update(targetRightVel, actualRightVel, dt, rightFF);

    writeMotorSpeeds(leftOut, rightOut);
  }

  /**
   * @brief GOAL mode: drive to targetPose_ with jerk-limited velocity ramping.
   *
   * Uses rampVelocity() instead of S-curve profiles because the profiles'
   * position-based braking logic is degenerate when used as a velocity shaper
   * (pos=0/goal_pos=0 → always triggers braking → profile outputs ~0).
   */
  void setPointControl(float dt) {
    if (dt < 0.001f) return;

    Pose2D pose = localization_.getPose();

    float dx = targetPose_.getX() - pose.getX();
    float dy = targetPose_.getY() - pose.getY();
    float distError = sqrtf(dx * dx + dy * dy);

    float vCmd = 0.0f;
    float omegaCmd = 0.0f;

    if (distError < setup_.poseDistTol) {
      // ── Phase: Position reached — align to target heading ──
      float headingError = secbot::utils::normalizeAngleRad(
          targetPose_.getTheta() - pose.getTheta());

      if (fabsf(headingError) > setup_.poseHeadingTol) {
        omegaCmd = setup_.poseKAngular * headingError;
        omegaCmd = secbot::utils::clamp(
            omegaCmd, -setup_.maxAngularVel, setup_.maxAngularVel);
      }
      // else: both within tolerance → vCmd=0, omegaCmd=0 → stop
    } else {
      // ── Phase: Drive toward goal position ──
      float targetAngle = atan2f(dy, dx);
      if (reverse_) targetAngle += static_cast<float>(M_PI);
      float angleError = secbot::utils::normalizeAngleRad(
          targetAngle - pose.getTheta());

      vCmd = setup_.poseKLinear * distError;
      if (reverse_) vCmd = -vCmd;
      omegaCmd = setup_.poseKAngular * angleError;

      // Squared angular rampdown near goal (prevents atan2 noise oscillation)
      float angularScale = fminf(distError / 0.15f, 1.0f);
      angularScale *= angularScale;
      omegaCmd *= angularScale;

      // Slow down if not facing target (prevents large arcs)
      float cosAngle = cosf(angleError);
      if (cosAngle < 0.0f) cosAngle = 0.0f;
      vCmd *= cosAngle;

      // Clamp to configured limits
      vCmd = secbot::utils::clamp(
          vCmd, -setup_.maxLinearVel, setup_.maxLinearVel);
      omegaCmd = secbot::utils::clamp(
          omegaCmd, -setup_.maxAngularVel, setup_.maxAngularVel);
    }

    // Jerk-limited velocity ramp using persisted ramp output
    auto& linLimits = setup_.linearProfile.limits;
    auto& angLimits = setup_.angularProfile.limits;
    goalRampedLinear_ = rampVelocity(
        goalRampedLinear_, vCmd, goalLinAccel_,
        linLimits.a_max, linLimits.d_max, linLimits.j_max,
        linLimits.v_max, dt);
    float rampedV = goalRampedLinear_;
    goalRampedAngular_ = rampVelocity(
        goalRampedAngular_, omegaCmd, goalAngAccel_,
        angLimits.a_max, angLimits.d_max, angLimits.j_max,
        angLimits.v_max, dt);
    float rampedOmega = goalRampedAngular_;

    // IK → wheel velocities
    float targetLeftVel =
        rampedV - (rampedOmega * setup_.locSetup.track_width * 0.5f);
    float targetRightVel =
        rampedV + (rampedOmega * setup_.locSetup.track_width * 0.5f);

    saturateWheelSpeeds(targetLeftVel, targetRightVel);

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

    float leftFF = velocityFeedforward(targetLeftVel);
    float rightFF = velocityFeedforward(targetRightVel);
    float leftOut = leftPID_.update(targetLeftVel, actualLeftVel, dt, leftFF);
    float rightOut = rightPID_.update(targetRightVel, actualRightVel, dt, rightFF);

    writeMotorSpeeds(leftOut, rightOut);
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

    // Saturate: scale both wheels proportionally if either exceeds motor limit
    saturateWheelSpeeds(targetLeftVel, targetRightVel);

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

    float leftFF = velocityFeedforward(targetLeftVel);
    float rightFF = velocityFeedforward(targetRightVel);
    float leftOut = leftPID_.update(targetLeftVel, actualLeftVel, dt, leftFF);
    float rightOut = rightPID_.update(targetRightVel, actualRightVel, dt, rightFF);

    writeMotorSpeeds(leftOut, rightOut);
  }

  // ── Motor output ───────────────────────────────────────────────────────

  void writeMotorSpeeds(float left, float right) {
    float l = secbot::utils::clamp(
        left * setup_.leftMotorMultiplier, -1.0f, 1.0f);
    float r = secbot::utils::clamp(
        right * setup_.rightMotorMultiplier, -1.0f, 1.0f);
    // Smooth motor output to prevent jerky transitions from PID spikes
    l = motorLeftFilter_.update(l);
    r = motorRightFilter_.update(r);
    setup_.motorManager->setSpeed(setup_.leftMotorIdx, l);
    setup_.motorManager->setSpeed(setup_.rightMotorIdx, r);
  }

  void stopMotors() {
    if (!setup_.motorManager) return;
    // Snap filters to zero (don't ramp — safety stop)
    motorLeftFilter_.reset(0.0f);
    motorRightFilter_.reset(0.0f);
    setup_.motorManager->setSpeed(setup_.leftMotorIdx, 0.0f);
    setup_.motorManager->setSpeed(setup_.rightMotorIdx, 0.0f);
  }

  // ── micro-ROS callbacks ────────────────────────────────────────────────

  static void commandCallback(const void* msvin, void* context) {
    auto* self = static_cast<DriveSubsystem*>(context);
    auto* msg = static_cast<const mcu_msgs__msg__DriveBase*>(msvin);

    self->lastCommandMs_ = millis();

    switch (msg->drive_mode) {
      case mcu_msgs__msg__DriveBase__DRIVE_VECTOR:
      case mcu_msgs__msg__DriveBase__DRIVE_VECTOR_RAW: {
        // goal_velocity is in SI units: m/s (linear.x) and rad/s (angular.z)
        float vx = static_cast<float>(msg->goal_velocity.linear.x);
        float omega = static_cast<float>(msg->goal_velocity.angular.z);
        // Clamp to physical limits
        vx = secbot::utils::clamp(
            vx, -self->setup_.maxLinearVel, self->setup_.maxLinearVel);
        omega = secbot::utils::clamp(
            omega, -self->setup_.maxAngularVel, self->setup_.maxAngularVel);
        // Pick mode based on whether gyro hold is desired
        DriveMode target_mode = (msg->drive_mode == mcu_msgs__msg__DriveBase__DRIVE_VECTOR_RAW)
            ? DriveMode::VELOCITY_RAW : DriveMode::VELOCITY;
        // Only reset profiles/PID if target actually changed
        if (self->mode_ != target_mode ||
            fabsf(vx - self->targetLinearVel_) > 0.01f ||
            fabsf(omega - self->targetAngularVel_) > 0.01f) {
          self->setVelocity(vx, omega, target_mode);
        } else {
          self->lastCommandMs_ = millis();  // refresh timeout
        }
        break;
      }
      case mcu_msgs__msg__DriveBase__DRIVE_GOAL: {
        float x = static_cast<float>(
            msg->goal_transform.transform.translation.x);
        float y = static_cast<float>(
            msg->goal_transform.transform.translation.y);
        bool reverse = msg->goal_transform.transform.translation.z < 0.0;
        // Extract yaw from quaternion
        float qz = static_cast<float>(
            msg->goal_transform.transform.rotation.z);
        float qw = static_cast<float>(
            msg->goal_transform.transform.rotation.w);
        float theta = 2.0f * atan2f(qz, qw);
        // Only reset PID/profile if goal actually changed
        if (self->mode_ != DriveMode::GOAL ||
            fabsf(x - self->targetPose_.getX()) > 0.001f ||
            fabsf(y - self->targetPose_.getY()) > 0.001f ||
            fabsf(theta - self->targetPose_.getTheta()) > 0.01f ||
            reverse != self->reverse_) {
          self->setGoal(x, y, theta, reverse);
        }
        break;
      }
      case mcu_msgs__msg__DriveBase__DRIVE_TRAJ: {
        size_t count = msg->goal_path.poses.size;
        if (count == 0 || count > MAX_TRAJECTORY_POINTS) break;

        TrajectoryController::Waypoint wps[MAX_TRAJECTORY_POINTS];
        for (size_t i = 0; i < count; i++) {
          auto& pose = msg->goal_path.poses.data[i].pose;
          wps[i].x = static_cast<float>(pose.position.x);
          wps[i].y = static_cast<float>(pose.position.y);
          wps[i].has_vel = 0;
          wps[i].has_heading = 0;
        }

        // Extract final heading from last waypoint's quaternion (if non-identity)
        auto& last_orient = msg->goal_path.poses.data[count - 1].pose.orientation;
        float qz = static_cast<float>(last_orient.z);
        float qw = static_cast<float>(last_orient.w);
        if (fabsf(qz) > 1e-4f || fabsf(qw - 1.0f) > 1e-4f) {
          wps[count - 1].heading = 2.0f * atan2f(qz, qw);
          wps[count - 1].has_heading = 1;
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

  static void configCallback(const void* req, void* res, void* ctx) {
    auto* self = static_cast<DriveSubsystem*>(ctx);
    auto* r = static_cast<const mcu_msgs__srv__SetDriveConfig_Request*>(req);
    auto* rsp = static_cast<mcu_msgs__srv__SetDriveConfig_Response*>(res);
    auto& s = const_cast<DriveSubsystemSetup&>(self->setup_);

    // Helper: apply non-zero PID fields
    auto applyPID = [](PIDController& pid, float kp, float ki, float kd,
                        float i_min, float i_max, float d_filter) {
      PIDController::Config cfg = pid.config();
      if (kp != 0.0f) cfg.gains.kp = kp;
      if (ki != 0.0f) cfg.gains.ki = ki;
      if (kd != 0.0f) cfg.gains.kd = kd;
      if (i_min != 0.0f) cfg.limits.i_min = i_min;
      if (i_max != 0.0f) cfg.limits.i_max = i_max;
      if (d_filter != 0.0f) cfg.d_filter_alpha = d_filter;
      pid.configure(cfg);
    };

    // ── Wheel velocity PID (both wheels get same gains) ──
    if (r->wheel_kp != 0.0f || r->wheel_ki != 0.0f || r->wheel_kd != 0.0f ||
        r->wheel_i_min != 0.0f || r->wheel_i_max != 0.0f ||
        r->wheel_d_filter != 0.0f) {
      applyPID(self->leftPID_, r->wheel_kp, r->wheel_ki, r->wheel_kd,
               r->wheel_i_min, r->wheel_i_max, r->wheel_d_filter);
      applyPID(self->rightPID_, r->wheel_kp, r->wheel_ki, r->wheel_kd,
               r->wheel_i_min, r->wheel_i_max, r->wheel_d_filter);
    }

    // ── Velocity limits ──
    if (r->max_linear_vel != 0.0f) s.maxLinearVel = r->max_linear_vel;
    if (r->max_angular_vel != 0.0f) s.maxAngularVel = r->max_angular_vel;

    // ── Acceleration limits ──
    if (r->max_linear_accel != 0.0f) {
      s.linearProfile.limits.a_max = r->max_linear_accel;
      s.linearProfile.limits.d_max = r->max_linear_accel;
    }
    if (r->max_angular_accel != 0.0f) {
      s.angularProfile.limits.a_max = r->max_angular_accel;
      s.angularProfile.limits.d_max = r->max_angular_accel;
    }

    // ── Jerk limits ──
    if (r->max_linear_jerk != 0.0f) s.linearProfile.limits.j_max = r->max_linear_jerk;
    if (r->max_angular_jerk != 0.0f) s.angularProfile.limits.j_max = r->max_angular_jerk;

    // ── Pose drive gains ──
    if (r->pose_k_linear != 0.0f) s.poseKLinear = r->pose_k_linear;
    if (r->pose_k_angular != 0.0f) s.poseKAngular = r->pose_k_angular;
    if (r->pose_dist_tol != 0.0f) s.poseDistTol = r->pose_dist_tol;
    if (r->pose_heading_tol != 0.0f) s.poseHeadingTol = r->pose_heading_tol;

    // ── Gyro heading hold ──
    if (r->gyro_hold_kp != 0.0f) s.gyroHoldKp = r->gyro_hold_kp;
    if (r->gyro_hold_threshold != 0.0f) s.gyroHoldThreshold = r->gyro_hold_threshold;

    // ── Trajectory controller ──
    {
      auto cfg = self->trajController_.config();
      if (r->traj_lookahead != 0.0f) cfg.lookahead_dist = r->traj_lookahead;
      if (r->traj_cruise_v != 0.0f) cfg.cruise_v = r->traj_cruise_v;
      if (r->traj_max_v != 0.0f) cfg.max_v = r->traj_max_v;
      if (r->traj_max_w != 0.0f) cfg.max_w = r->traj_max_w;
      if (r->traj_slowdown_dist != 0.0f) cfg.slowdown_dist = r->traj_slowdown_dist;
      if (r->traj_min_v != 0.0f) cfg.min_v_near_goal = r->traj_min_v;
      if (r->traj_pos_tol != 0.0f) cfg.pos_tol = r->traj_pos_tol;
      if (r->traj_heading_tol != 0.0f) cfg.heading_tol = r->traj_heading_tol;
      if (r->traj_k_heading != 0.0f) cfg.k_heading = r->traj_k_heading;
      if (r->traj_advance_tol != 0.0f) cfg.advance_tol = r->traj_advance_tol;
      self->trajController_.configure(cfg);
    }

    // ── Input filter time constants ──
    if (r->cmd_vel_filter_tau != 0.0f) {
      self->cmdVelLinearFilter_.configureTauDtFast(r->cmd_vel_filter_tau, 0.005f);
      self->cmdVelAngularFilter_.configureTauDtFast(r->cmd_vel_filter_tau, 0.005f);
      self->rcThrottleFilter_.configureTauDtFast(r->cmd_vel_filter_tau, 0.005f);
      self->rcSteeringFilter_.configureTauDtFast(r->cmd_vel_filter_tau, 0.005f);
    }
    if (r->motor_filter_tau != 0.0f) {
      self->motorLeftFilter_.configureTauDtFast(r->motor_filter_tau, 0.02f);
      self->motorRightFilter_.configureTauDtFast(r->motor_filter_tau, 0.02f);
    }

    // ── Command timeout ──
    if (r->command_timeout_ms != 0) s.commandTimeoutMs = r->command_timeout_ms;

    rsp->success = true;
    static char buf[256];
    int len = snprintf(buf, sizeof(buf),
             "PID kp=%.3f ki=%.3f kd=%.3f | vel=%.2f/%.1f | "
             "accel=%.2f/%.1f | jerk=%.2f/%.1f | traj la=%.3f cv=%.2f",
             self->leftPID_.config().gains.kp,
             self->leftPID_.config().gains.ki,
             self->leftPID_.config().gains.kd,
             s.maxLinearVel, s.maxAngularVel,
             s.linearProfile.limits.a_max, s.angularProfile.limits.a_max,
             s.linearProfile.limits.j_max, s.angularProfile.limits.j_max,
             self->trajController_.config().lookahead_dist,
             self->trajController_.config().cruise_v);
    rsp->message.data = buf;
    rsp->message.size = static_cast<size_t>(len);
    rsp->message.capacity = sizeof(buf);
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
      case DriveMode::VELOCITY_RAW:
        status_msg_.drive_mode = mcu_msgs__msg__DriveBase__DRIVE_VECTOR_RAW;
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
  }

 public:
  void publishAll() override {
    Threads::Scope lock(data_mutex_);
    if (!data_ready_ || !status_pub_.impl) return;
    (void)rcl_publish(&status_pub_, &status_msg_, NULL);
    data_ready_ = false;
  }

 private:

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
  bool reverse_ = false;

  // Velocity ramp state (persists across ticks within each mode).
  // The ramp tracks its own output so it accumulates properly even when
  // the robot is stalled (measured velocity = 0).  Without this, the ramp
  // re-anchors to measured velocity each cycle and can never exceed one
  // dt-step above actual — trapping the feedforward below the motor deadband.
  float velLinAccel_ = 0.0f;   // VELOCITY mode accel state
  float velAngAccel_ = 0.0f;
  float velRampedLinear_ = 0.0f;   // VELOCITY mode ramp output (persisted)
  float velRampedAngular_ = 0.0f;
  float goalLinAccel_ = 0.0f;  // GOAL mode accel state
  float goalAngAccel_ = 0.0f;
  float goalRampedLinear_ = 0.0f;  // GOAL mode ramp output (persisted)
  float goalRampedAngular_ = 0.0f;

  // Gyro heading hold state
  float gyroHoldHeading_ = 0.0f;  // locked heading when driving straight
  bool gyroHoldActive_ = false;   // true when heading is being held

  // Persistent waypoint storage (trajectory controller borrows pointer, must outlive it)
  TrajectoryController::Waypoint trajWaypoints_[MAX_TRAJECTORY_POINTS] = {};
  size_t trajWaypointCount_ = 0;

  // RC input smoothing
  secbot::utils::LowPass1P rcThrottleFilter_;
  secbot::utils::LowPass1P rcSteeringFilter_;

  // cmd_vel input smoothing (same filter type as RC)
  secbot::utils::LowPass1P cmdVelLinearFilter_;
  secbot::utils::LowPass1P cmdVelAngularFilter_;

  // Motor output smoothing (prevents PID spikes from jerking wheels)
  // Snappier than RC input filters (tau=0.05s vs 0.15s)
  secbot::utils::LowPass1P motorLeftFilter_;
  secbot::utils::LowPass1P motorRightFilter_;

  // Timing
  uint32_t last_update_us_ = 0;
  uint32_t lastCommandMs_ = 0;
  uint8_t publishCounter_ = 0;

  // micro-ROS handles
  rcl_node_t* node_ = nullptr;
  rcl_publisher_t status_pub_{};
  rcl_subscription_t cmd_sub_{};
  rcl_subscription_t pose_sub_{};
  rcl_service_t config_srv_{};
  mcu_msgs__msg__DriveBase status_msg_{};
  mcu_msgs__msg__DriveBase cmd_msg_{};
  geometry_msgs__msg__Pose pose_msg_{};
  mcu_msgs__srv__SetDriveConfig_Request config_req_{};
  mcu_msgs__srv__SetDriveConfig_Response config_res_{};
  micro_ros_utilities_memory_conf_t status_mem_conf_{};
  micro_ros_utilities_memory_conf_t cmd_mem_conf_{};

  bool data_ready_ = false;
  Threads::Mutex data_mutex_;
};

}  // namespace Subsystem
