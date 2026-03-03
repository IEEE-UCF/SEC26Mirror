/**
 * @file KeypadSubsystem.h
 * @brief Keypad field element interaction via servo key selection and drive press.
 * @date 2026-02-28
 *
 * Uses a 270-degree servo (PCA9685 channel 1) to select one of four keys,
 * then drives the robot forward at low speed to press the key (stall against
 * the keypad), and backs up.
 *
 * The servo does NOT move at startup — an explicit command is required.
 *
 * 270-degree servo angle mapping (physical → ServoSubsystem 0-180 range):
 *   Key 1:   0° physical →   0° servo
 *   Key 2:  90° physical →  60° servo
 *   Key 3: 180° physical → 120° servo
 *   Key 4: 270° physical → 180° servo
 *
 * State machine:
 *   IDLE → SWITCHING → IDLE       (select key 1-4)
 *   IDLE → PRESSING → RETREATING → IDLE   (press current key)
 *
 * -- ROS2 interface (defaults) ------------------------------------------------
 *   /mcu_robot/keypad/state    publisher     std_msgs/msg/UInt8   (5 Hz)
 *   /mcu_robot/keypad/command  subscription  std_msgs/msg/UInt8
 *     command 1-4 = switch to key, command 5 = press key
 */

#pragma once

#include <BaseSubsystem.h>
#include <microros_manager_robot.h>
#include <std_msgs/msg/u_int8.h>

#include "robot/drive-base/DriveSubsystem.h"
#include "robot/subsystems/ServoSubsystem.h"

#ifdef USE_TEENSYTHREADS
#include <TeensyThreads.h>
#endif

namespace Subsystem {

enum class KeypadState : uint8_t {
  IDLE = 0,
  SWITCHING = 1,
  PRESSING = 2,
  RETREATING = 3,
};

enum class KeypadCommand : uint8_t {
  NONE = 0,
  KEY1 = 1,
  KEY2 = 2,
  KEY3 = 3,
  KEY4 = 4,
  PRESS = 5,
};

class KeypadSubsystemSetup : public Classes::BaseSetup {
 public:
  static constexpr uint8_t NUM_KEYS = 4;

  /**
   * @param _id              Subsystem identifier.
   * @param servo            Pointer to the ServoSubsystem that owns the PCA9685.
   * @param servoIndex       PCA9685 channel index for the keypad servo.
   * @param drive            Pointer to DriveSubsystem for press/retreat motion.
   * @param switchDurationMs Time for servo to reach target position.
   * @param pressSpeed       Forward velocity during press (m/s, slow enough to stall).
   * @param pressDurationMs  How long to drive forward into the keypad.
   * @param retreatSpeed     Reverse velocity during retreat (m/s).
   * @param retreatDurationMs How long to drive backward after pressing.
   * @param stateTopic       ROS2 topic for state publishing.
   * @param commandTopic     ROS2 topic for command subscription.
   */
  KeypadSubsystemSetup(const char* _id, ServoSubsystem* servo,
                        uint8_t servoIndex, DriveSubsystem* drive,
                        uint32_t switchDurationMs = 500,
                        float pressSpeed = 0.08f,
                        uint32_t pressDurationMs = 1500,
                        float retreatSpeed = 0.15f,
                        uint32_t retreatDurationMs = 1000,
                        const char* stateTopic = "/mcu_robot/keypad/state",
                        const char* commandTopic = "/mcu_robot/keypad/command")
      : Classes::BaseSetup(_id),
        servo_(servo),
        servoIndex_(servoIndex),
        drive_(drive),
        switchDurationMs_(switchDurationMs),
        pressSpeed_(pressSpeed),
        pressDurationMs_(pressDurationMs),
        retreatSpeed_(retreatSpeed),
        retreatDurationMs_(retreatDurationMs),
        stateTopic_(stateTopic),
        commandTopic_(commandTopic) {
    // Default key angles for a 270° servo mapped to ServoSubsystem 0-180 range
    keyAngles_[0] = 0.0f;    // Key 1:   0° physical
    keyAngles_[1] = 60.0f;   // Key 2:  90° physical
    keyAngles_[2] = 120.0f;  // Key 3: 180° physical
    keyAngles_[3] = 180.0f;  // Key 4: 270° physical
  }

  ServoSubsystem* servo_;
  uint8_t servoIndex_;
  DriveSubsystem* drive_;
  float keyAngles_[NUM_KEYS];
  uint32_t switchDurationMs_;
  float pressSpeed_;
  uint32_t pressDurationMs_;
  float retreatSpeed_;
  uint32_t retreatDurationMs_;
  const char* stateTopic_;
  const char* commandTopic_;
};

class KeypadSubsystem : public IMicroRosParticipant,
                         public Classes::BaseSubsystem {
 public:
  explicit KeypadSubsystem(const KeypadSubsystemSetup& setup)
      : Classes::BaseSubsystem(setup), setup_(setup) {}

  // ── Lifecycle ──────────────────────────────────────────────────────────

  bool init() override {
    return setup_.servo_ != nullptr && setup_.drive_ != nullptr;
  }

  void begin() override {}

  void update() override {
    uint32_t now = millis();

    // Process pending command
    KeypadCommand cmd = pending_command_;
    pending_command_ = KeypadCommand::NONE;

    switch (state_) {
      case KeypadState::IDLE:
        if (cmd >= KeypadCommand::KEY1 && cmd <= KeypadCommand::KEY4) {
          switchToKey(static_cast<uint8_t>(cmd) - 1);
        } else if (cmd == KeypadCommand::PRESS) {
          pressKey();
        }
        break;

      case KeypadState::SWITCHING:
        if (now - state_entry_ms_ >= setup_.switchDurationMs_) {
          state_ = KeypadState::IDLE;
        }
        break;

      case KeypadState::PRESSING:
        // Re-issue velocity command every cycle to prevent drive timeout
        setup_.drive_->setVelocity(setup_.pressSpeed_, 0.0f);
        if (now - state_entry_ms_ >= setup_.pressDurationMs_) {
          // Transition to retreat
          setup_.drive_->setVelocity(-setup_.retreatSpeed_, 0.0f);
          state_ = KeypadState::RETREATING;
          state_entry_ms_ = now;
        }
        break;

      case KeypadState::RETREATING:
        // Re-issue velocity command every cycle to prevent drive timeout
        setup_.drive_->setVelocity(-setup_.retreatSpeed_, 0.0f);
        if (now - state_entry_ms_ >= setup_.retreatDurationMs_) {
          setup_.drive_->stop();
          state_ = KeypadState::IDLE;
        }
        break;
    }

    // Publish state at ~5 Hz
    if (state_pub_.impl && now - last_publish_ms_ >= PUBLISH_INTERVAL_MS) {
      last_publish_ms_ = now;
      publishState();
    }
  }

  void pause() override {
    if (state_ == KeypadState::PRESSING || state_ == KeypadState::RETREATING) {
      setup_.drive_->stop();
    }
    state_ = KeypadState::IDLE;
  }

  void reset() override { pause(); }

  const char* getInfo() override {
    static const char info[] = "KeypadSubsystem";
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
            executor, &cmd_sub_, &cmd_msg_, &KeypadSubsystem::cmdCallback, this,
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

  /** @brief Switch to a key position (0-3). */
  void switchToKey(uint8_t keyIndex) {
    if (state_ != KeypadState::IDLE) return;
    if (keyIndex >= KeypadSubsystemSetup::NUM_KEYS) return;
    currentKey_ = keyIndex;
    setup_.servo_->setAngle(setup_.servoIndex_, setup_.keyAngles_[keyIndex]);
    state_ = KeypadState::SWITCHING;
    state_entry_ms_ = millis();
  }

  /** @brief Press the currently selected key (drives forward then retreats). */
  void pressKey() {
    if (state_ != KeypadState::IDLE) return;
    setup_.drive_->setVelocity(setup_.pressSpeed_, 0.0f);
    state_ = KeypadState::PRESSING;
    state_entry_ms_ = millis();
  }

  KeypadState getState() const { return state_; }
  uint8_t getCurrentKey() const { return currentKey_; }

  // ── TeensyThreads ─────────────────────────────────────────────────────

#ifdef USE_TEENSYTHREADS
  void beginThreaded(uint32_t stackSize, int /*priority*/ = 1,
                     uint32_t updateRateMs = 50) {
    task_delay_ms_ = updateRateMs;
    threads.addThread(taskFunction, this, stackSize);
  }

 private:
  static void taskFunction(void* pv) {
    auto* self = static_cast<KeypadSubsystem*>(pv);
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
    auto* self = static_cast<KeypadSubsystem*>(ctx);
    auto* m = static_cast<const std_msgs__msg__UInt8*>(msg);
    if (m->data >= static_cast<uint8_t>(KeypadCommand::KEY1) &&
        m->data <= static_cast<uint8_t>(KeypadCommand::PRESS)) {
      self->pending_command_ = static_cast<KeypadCommand>(m->data);
    }
  }

  static constexpr uint32_t PUBLISH_INTERVAL_MS = 200;  // 5 Hz

  const KeypadSubsystemSetup setup_;
  KeypadState state_ = KeypadState::IDLE;
  KeypadCommand pending_command_ = KeypadCommand::NONE;
  uint8_t currentKey_ = 0;
  uint32_t state_entry_ms_ = 0;
  uint32_t last_publish_ms_ = 0;

  rcl_publisher_t state_pub_{};
  rcl_subscription_t cmd_sub_{};
  std_msgs__msg__UInt8 state_msg_{};
  std_msgs__msg__UInt8 cmd_msg_{};
  rcl_node_t* node_ = nullptr;
};

}  // namespace Subsystem
