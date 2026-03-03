/**
 * @file IntakeSubsystem.h
 * @brief Combined linear rail + spinning intake motor subsystem.
 * @date 2026-02-28
 *
 * Manages two mechanisms:
 *   1. Linear rail — encoder-based P control with TCA9555 limit switch hard
 *      stops and automatic calibration sequence.
 *   2. Spinning intake motor — simple signed speed control.
 *
 * Both motors are driven via PCA9685 MotorManagerSubsystem.  The rail encoder
 * is read via EncoderSubsystem (QTimerEncoder hardware).  Limit switches are
 * read via ButtonSubsystem (TCA9555 GPIO expander).
 *
 * State machine (linear rail):
 *   UNINITIALIZED -> CALIBRATING -> IDLE <-> MOVING
 *   Any state may transition to FAULT on unrecoverable error.
 *
 * -- ROS2 interface (defaults) ------------------------------------------------
 *   /mcu_robot/intake/state    publisher     mcu_msgs/msg/IntakeState   (20 Hz)
 *   /mcu_robot/intake/command  subscription  mcu_msgs/msg/IntakeCommand
 */

#pragma once

#include <BaseSubsystem.h>
#include <microros_manager_robot.h>

#ifdef USE_TEENSYTHREADS
#include <TeensyThreads.h>
#endif

#include "TimedSubsystem.h"
#include "mcu_msgs/msg/intake_command.h"
#include "mcu_msgs/msg/intake_state.h"
#include "robot/subsystems/ButtonSubsystem.h"
#include "robot/subsystems/EncoderSubsystem.h"
#include "robot/subsystems/MotorManagerSubsystem.h"

namespace Subsystem {

enum class IntakeState : uint8_t {
  UNINITIALIZED = 0,
  CALIBRATING = 1,
  IDLE = 2,
  MOVING = 3,
  FAULT = 4
};

class IntakeSubsystemSetup : public Classes::BaseSetup {
 public:
  IntakeSubsystemSetup(const char* _id, MotorManagerSubsystem* motor_mgr,
                       EncoderSubsystem* encoder_sub,
                       ButtonSubsystem* btn_sub, uint8_t rail_motor_idx,
                       uint8_t intake_motor_idx, uint8_t rail_encoder_idx,
                       uint8_t retract_limit_btn, uint8_t extend_limit_btn,
                       float kp = 2.0f, float max_speed = 0.8f,
                       float calibration_speed = 0.3f,
                       float position_tolerance = 0.02f,
                       uint32_t stall_timeout_ms = 500,
                       const char* state_topic = "/mcu_robot/intake/state",
                       const char* cmd_topic = "/mcu_robot/intake/command")
      : Classes::BaseSetup(_id),
        motor_mgr_(motor_mgr),
        encoder_sub_(encoder_sub),
        btn_sub_(btn_sub),
        rail_motor_idx_(rail_motor_idx),
        intake_motor_idx_(intake_motor_idx),
        rail_encoder_idx_(rail_encoder_idx),
        retract_limit_btn_(retract_limit_btn),
        extend_limit_btn_(extend_limit_btn),
        kp_(kp),
        max_speed_(max_speed),
        calibration_speed_(calibration_speed),
        position_tolerance_(position_tolerance),
        stall_timeout_ms_(stall_timeout_ms),
        state_topic_(state_topic),
        cmd_topic_(cmd_topic) {}

  MotorManagerSubsystem* motor_mgr_;
  EncoderSubsystem* encoder_sub_;
  ButtonSubsystem* btn_sub_;

  uint8_t rail_motor_idx_;
  uint8_t intake_motor_idx_;
  uint8_t rail_encoder_idx_;
  uint8_t retract_limit_btn_;
  uint8_t extend_limit_btn_;

  float kp_;
  float max_speed_;
  float calibration_speed_;
  float position_tolerance_;
  uint32_t stall_timeout_ms_;

  const char* state_topic_;
  const char* cmd_topic_;
};

class IntakeSubsystem : public IMicroRosParticipant,
                        public Subsystem::TimedSubsystem {
 public:
  explicit IntakeSubsystem(const IntakeSubsystemSetup& setup)
      : Subsystem::TimedSubsystem(setup), setup_(setup) {}

  // ── Lifecycle ──────────────────────────────────────────────────────────
  bool init() override;
  void begin() override;
  void update() override;
  void pause() override;
  void reset() override;
  const char* getInfo() override;

  // ── IMicroRosParticipant ───────────────────────────────────────────────
  bool onCreate(rcl_node_t* node, rclc_executor_t* executor) override;
  void onDestroy() override;

  // ── Linear Rail API ────────────────────────────────────────────────────
  void initialize();
  void setPosition(float normalized);
  void extend() { setPosition(1.0f); }
  void retract() { setPosition(0.0f); }
  void stopRail();

  bool isCalibrated() const { return calibrated_; }
  float getPosition() const;
  float getSetpoint() const { return setpoint_; }
  IntakeState getState() const { return state_; }

  // ── Intake Motor API ──────────────────────────────────────────────────
  void setIntakeSpeed(float speed);
  float getIntakeSpeed() const { return intake_speed_; }
  void stopIntake() { setIntakeSpeed(0.0f); }

  // ── TeensyThreads ─────────────────────────────────────────────────────
#ifdef USE_TEENSYTHREADS
  void beginThreaded(uint32_t stackSize, int /*priority*/ = 1,
                     uint32_t updateRateMs = 20) {
    task_delay_ms_ = updateRateMs;
    threads.addThread(taskFunction, this, stackSize);
  }

 private:
  static void taskFunction(void* pvParams) {
    auto* self = static_cast<IntakeSubsystem*>(pvParams);
    self->begin();
    while (true) {
      self->update();
      threads.delay(self->task_delay_ms_);
    }
  }
  uint32_t task_delay_ms_ = 20;
#endif

 private:
  // State machine
  void transitionTo(IntakeState new_state);
  void updateCalibration();
  void updateMoving();
  void enforceHardStops();

  // ROS
  void publishState();
  static void cmdCallback(const void* msg, void* ctx);

  const IntakeSubsystemSetup setup_;

  // Rail state
  IntakeState state_ = IntakeState::UNINITIALIZED;
  bool calibrated_ = false;
  float setpoint_ = 0.0f;
  int32_t min_ticks_ = 0;
  int32_t max_ticks_ = 0;

  // Calibration sub-state
  enum class CalibPhase : uint8_t {
    SEEK_RETRACT,
    SEEK_EXTEND,
    RETURN_HOME
  };
  CalibPhase calib_phase_ = CalibPhase::SEEK_RETRACT;
  bool calib_tried_reverse_ = false;
  int32_t calib_last_ticks_ = 0;
  uint32_t calib_last_change_ms_ = 0;

  // Intake motor
  float intake_speed_ = 0.0f;

  // Timing
  uint32_t state_entry_ms_ = 0;

  // ROS entities
  rcl_publisher_t state_pub_{};
  rcl_subscription_t cmd_sub_{};
  mcu_msgs__msg__IntakeState state_msg_{};
  mcu_msgs__msg__IntakeCommand cmd_msg_{};
  rcl_node_t* node_ = nullptr;
};

}  // namespace Subsystem
