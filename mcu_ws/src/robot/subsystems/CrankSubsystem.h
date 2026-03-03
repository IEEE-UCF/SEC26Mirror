/**
 * @file CrankSubsystem.h
 * @brief Crank mechanism control via a single PCA9685 servo channel.
 * @date 2026-02-28
 *
 * Open-loop, time-based servo control for spinning a field crank element.
 * The servo is driven through ServoSubsystem::setAngle() — no direct PCA9685
 * access.  The servo does NOT move at startup; an explicit SPIN or RESET
 * command is required.
 *
 * State machine:
 *   IDLE → CRANKING → IDLE   (spin command: 0° → 180°)
 *   IDLE → RESETTING → IDLE  (reset command: 180° → 0°)
 *
 * -- ROS2 interface (defaults) ------------------------------------------------
 *   /mcu_robot/crank/state    publisher     std_msgs/msg/UInt8   (5 Hz)
 *   /mcu_robot/crank/command  subscription  std_msgs/msg/UInt8
 *     command 1 = SPIN, command 2 = RESET
 */

#pragma once

#include <BaseSubsystem.h>
#include <microros_manager_robot.h>
#include <std_msgs/msg/u_int8.h>

#include "robot/subsystems/ServoSubsystem.h"

#ifdef USE_TEENSYTHREADS
#include <TeensyThreads.h>
#endif

namespace Subsystem {

enum class CrankState : uint8_t {
  IDLE = 0,
  CRANKING = 1,
  RESETTING = 2,
};

enum class CrankCommand : uint8_t {
  NONE = 0,
  SPIN = 1,
  RESET = 2,
};

class CrankSubsystemSetup : public Classes::BaseSetup {
 public:
  /**
   * @param _id            Subsystem identifier.
   * @param servo          Pointer to the ServoSubsystem that owns the PCA9685.
   * @param servoIndex     PCA9685 channel index for the crank servo.
   * @param spinAngle      Target angle for the spin action (degrees).
   * @param homeAngle      Home/reset angle (degrees).
   * @param spinDurationMs Time to hold CRANKING state before returning to IDLE.
   * @param resetDurationMs Time to hold RESETTING state before returning to IDLE.
   * @param stateTopic     ROS2 topic for state publishing.
   * @param commandTopic   ROS2 topic for command subscription.
   */
  CrankSubsystemSetup(const char* _id, ServoSubsystem* servo,
                       uint8_t servoIndex = 0, float spinAngle = 180.0f,
                       float homeAngle = 0.0f, uint32_t spinDurationMs = 1500,
                       uint32_t resetDurationMs = 1500,
                       const char* stateTopic = "/mcu_robot/crank/state",
                       const char* commandTopic = "/mcu_robot/crank/command")
      : Classes::BaseSetup(_id),
        servo_(servo),
        servoIndex_(servoIndex),
        spinAngle_(spinAngle),
        homeAngle_(homeAngle),
        spinDurationMs_(spinDurationMs),
        resetDurationMs_(resetDurationMs),
        stateTopic_(stateTopic),
        commandTopic_(commandTopic) {}

  ServoSubsystem* servo_;
  uint8_t servoIndex_;
  float spinAngle_;
  float homeAngle_;
  uint32_t spinDurationMs_;
  uint32_t resetDurationMs_;
  const char* stateTopic_;
  const char* commandTopic_;
};

class CrankSubsystem : public IMicroRosParticipant,
                        public Classes::BaseSubsystem {
 public:
  explicit CrankSubsystem(const CrankSubsystemSetup& setup)
      : Classes::BaseSubsystem(setup), setup_(setup) {}

  // ── Lifecycle ──────────────────────────────────────────────────────────

  bool init() override { return setup_.servo_ != nullptr; }

  void begin() override {}

  void update() override {
    uint32_t now = millis();

    // Process pending command
    CrankCommand cmd = pending_command_;
    pending_command_ = CrankCommand::NONE;

    switch (state_) {
      case CrankState::IDLE:
        if (cmd == CrankCommand::SPIN) {
          spin();
        } else if (cmd == CrankCommand::RESET) {
          resetCrank();
        }
        break;

      case CrankState::CRANKING:
        if (now - state_entry_ms_ >= setup_.spinDurationMs_) {
          state_ = CrankState::IDLE;
        }
        break;

      case CrankState::RESETTING:
        if (now - state_entry_ms_ >= setup_.resetDurationMs_) {
          state_ = CrankState::IDLE;
        }
        break;
    }

    // Publish state at ~5 Hz
    if (state_pub_.impl && now - last_publish_ms_ >= PUBLISH_INTERVAL_MS) {
      last_publish_ms_ = now;
      publishState();
    }
  }

  void pause() override {}

  void reset() override { state_ = CrankState::IDLE; }

  const char* getInfo() override {
    static const char info[] = "CrankSubsystem";
    return info;
  }

  // ── micro-ROS ─────────────────────────────────────────────────────────

  bool onCreate(rcl_node_t* node, rclc_executor_t* executor) override {
    node_ = node;

    if (rclc_publisher_init_best_effort(
            &state_pub_, node_,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8),
            setup_.stateTopic_) != RCL_RET_OK) {
      return false;
    }

    if (rclc_subscription_init_best_effort(
            &cmd_sub_, node_,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8),
            setup_.commandTopic_) != RCL_RET_OK) {
      return false;
    }

    if (rclc_executor_add_subscription_with_context(
            executor, &cmd_sub_, &cmd_msg_, &CrankSubsystem::cmdCallback, this,
            ON_NEW_DATA) != RCL_RET_OK) {
      return false;
    }

    return true;
  }

  void onDestroy() override {
    state_pub_ = rcl_get_zero_initialized_publisher();
    cmd_sub_ = rcl_get_zero_initialized_subscription();
    node_ = nullptr;
  }

  // ── Public API (callable from other subsystems) ────────────────────────

  void spin() {
    if (state_ != CrankState::IDLE) return;
    setup_.servo_->setAngle(setup_.servoIndex_, setup_.spinAngle_);
    state_ = CrankState::CRANKING;
    state_entry_ms_ = millis();
  }

  void resetCrank() {
    if (state_ != CrankState::IDLE) return;
    setup_.servo_->setAngle(setup_.servoIndex_, setup_.homeAngle_);
    state_ = CrankState::RESETTING;
    state_entry_ms_ = millis();
  }

  CrankState getState() const { return state_; }

  // ── TeensyThreads ─────────────────────────────────────────────────────

#ifdef USE_TEENSYTHREADS
  void beginThreaded(uint32_t stackSize, int /*priority*/ = 1,
                     uint32_t updateRateMs = 50) {
    task_delay_ms_ = updateRateMs;
    threads.addThread(taskFunction, this, stackSize);
  }

 private:
  static void taskFunction(void* pv) {
    auto* self = static_cast<CrankSubsystem*>(pv);
    self->begin();
    while (true) {
      self->update();
      threads.delay(self->task_delay_ms_);
    }
  }
  uint32_t task_delay_ms_ = 50;
#endif

 private:
  void publishState() {
    state_msg_.data = static_cast<uint8_t>(state_);
#ifdef USE_TEENSYTHREADS
    {
      Threads::Scope guard(g_microros_mutex);
      (void)rcl_publish(&state_pub_, &state_msg_, NULL);
    }
#else
    (void)rcl_publish(&state_pub_, &state_msg_, NULL);
#endif
  }

  static void cmdCallback(const void* msg, void* ctx) {
    auto* self = static_cast<CrankSubsystem*>(ctx);
    auto* m = static_cast<const std_msgs__msg__UInt8*>(msg);
    if (m->data == static_cast<uint8_t>(CrankCommand::SPIN) ||
        m->data == static_cast<uint8_t>(CrankCommand::RESET)) {
      self->pending_command_ = static_cast<CrankCommand>(m->data);
    }
  }

  static constexpr uint32_t PUBLISH_INTERVAL_MS = 200;  // 5 Hz

  const CrankSubsystemSetup setup_;
  CrankState state_ = CrankState::IDLE;
  CrankCommand pending_command_ = CrankCommand::NONE;
  uint32_t state_entry_ms_ = 0;
  uint32_t last_publish_ms_ = 0;

  rcl_publisher_t state_pub_{};
  rcl_subscription_t cmd_sub_{};
  std_msgs__msg__UInt8 state_msg_{};
  std_msgs__msg__UInt8 cmd_msg_{};
  rcl_node_t* node_ = nullptr;
};

}  // namespace Subsystem
