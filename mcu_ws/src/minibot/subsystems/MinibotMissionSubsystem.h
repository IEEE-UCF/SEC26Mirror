/**
 * @file MinibotMissionSubsystem.h
 * @brief Service-triggered crater navigation state machine for the minibot.
 *
 * Non-blocking state machine driven by millis() in update(). Two micro-ROS
 * services trigger missions; motor work happens in the subsystem's FreeRTOS
 * task, never in the service callback.
 *
 * -- ROS2 interface -----------------------------------------------------------
 *   /mcu_minibot/state          pub   mcu_msgs/msg/MiniRobotState (best-effort)
 *   /mcu_minibot/enter_crater   srv   mcu_msgs/srv/Reset
 *   /mcu_minibot/exit_crater    srv   mcu_msgs/srv/Reset
 */
#pragma once

#include <Arduino.h>
#include <RTOSSubsystem.h>
#include <mcu_msgs/msg/mini_robot_state.h>
#include <mcu_msgs/srv/reset.h>
#include <microros_manager_robot.h>

#include "../MinibotMotorDriver.h"

namespace Subsystem {

class MinibotMissionSubsystem : public IMicroRosParticipant,
                                public Subsystem::RTOSSubsystem {
 public:
  MinibotMissionSubsystem(const char* id,
                          Drivers::MinibotMotorDriver& left,
                          Drivers::MinibotMotorDriver& right)
      : Subsystem::RTOSSubsystem(Classes::BaseSetup(id)),
        left_(left),
        right_(right) {}

  bool init() override {
    stopMotors();
    return true;
  }

  void begin() override {}

  void update() override {
    // Process pending commands from service callbacks (runs in this thread
    // only, avoiding cross-thread state/motor writes)
    uint8_t cmd = pending_cmd_;
    if (cmd == CMD_ENTER) {
      pending_cmd_ = CMD_NONE;
      current_task_ = mcu_msgs__msg__MiniRobotState__TASK_ENTER_CRATER;
      phase_start_ms_ = millis();
      phase_ = Phase::ENTER_WAIT;
    } else if (cmd == CMD_EXIT) {
      pending_cmd_ = CMD_NONE;
      current_task_ = mcu_msgs__msg__MiniRobotState__TASK_EXIT_CRATER;
      phase_start_ms_ = millis();
      left_.setPWM(EXIT_SPEED);
      right_.setPWM(EXIT_SPEED);
      phase_ = Phase::EXIT_FORWARD;
    }

    uint32_t now = millis();
    uint32_t elapsed = now - phase_start_ms_;

    switch (phase_) {
      case Phase::IDLE:
        break;

      case Phase::ENTER_WAIT:
        if (elapsed >= ENTER_WAIT_MS) {
          phase_ = Phase::ENTER_FORWARD;
          phase_start_ms_ = now;
          left_.setPWM(ENTER_SPEED);
          right_.setPWM(ENTER_SPEED);
        }
        break;

      case Phase::ENTER_FORWARD:
        if (elapsed >= ENTER_FORWARD_MS) {
          phase_ = Phase::ENTER_TURN_LEFT;
          phase_start_ms_ = now;
          left_.setPWM((int)(TURN_SPEED * TURN_FACTOR));
          right_.setPWM(TURN_SPEED);
        }
        break;

      case Phase::ENTER_TURN_LEFT:
        if (elapsed >= ENTER_TURN_MS) {
          stopMotors();
          phase_ = Phase::IDLE;
          current_task_ = mcu_msgs__msg__MiniRobotState__TASK_NONE;
        }
        break;

      case Phase::EXIT_FORWARD:
        if (elapsed >= EXIT_FORWARD_MS) {
          stopMotors();
          phase_ = Phase::IDLE;
          current_task_ = mcu_msgs__msg__MiniRobotState__TASK_NONE;
        }
        break;
    }

    // Drive motors every tick
    left_.update();
    right_.update();

    // Publish state at 10 Hz
    if (everyMs(100)) {
      publishState();
    }
  }

  void pause() override { stopMotors(); }
  void reset() override {
    stopMotors();
    phase_ = Phase::IDLE;
    current_task_ = mcu_msgs__msg__MiniRobotState__TASK_NONE;
  }

  const char* getInfo() override {
    static const char info[] = "MinibotMission";
    return info;
  }

  // ── IMicroRosParticipant ──────────────────────────────────────────────

  bool onCreate(rcl_node_t* node, rclc_executor_t* executor) override {
    node_ = node;

    mcu_msgs__msg__MiniRobotState__init(&state_msg_);

    // State publisher (best-effort)
    if (rclc_publisher_init_best_effort(
            &state_pub_, node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(mcu_msgs, msg, MiniRobotState),
            "/mcu_minibot/state") != RCL_RET_OK) {
      return false;
    }

    // Enter crater service
    if (rclc_service_init_default(
            &enter_srv_, node,
            ROSIDL_GET_SRV_TYPE_SUPPORT(mcu_msgs, srv, Reset),
            "/mcu_minibot/enter_crater") != RCL_RET_OK) {
      return false;
    }
    if (rclc_executor_add_service_with_context(
            executor, &enter_srv_, &enter_req_, &enter_res_,
            &MinibotMissionSubsystem::enterCallback, this) != RCL_RET_OK) {
      return false;
    }

    // Exit crater service
    if (rclc_service_init_default(
            &exit_srv_, node,
            ROSIDL_GET_SRV_TYPE_SUPPORT(mcu_msgs, srv, Reset),
            "/mcu_minibot/exit_crater") != RCL_RET_OK) {
      return false;
    }
    if (rclc_executor_add_service_with_context(
            executor, &exit_srv_, &exit_req_, &exit_res_,
            &MinibotMissionSubsystem::exitCallback, this) != RCL_RET_OK) {
      return false;
    }

    return true;
  }

  void onDestroy() override {
    state_pub_ = rcl_get_zero_initialized_publisher();
    enter_srv_ = rcl_get_zero_initialized_service();
    exit_srv_ = rcl_get_zero_initialized_service();
    node_ = nullptr;
  }

  void publishAll() override {
#if defined(USE_FREERTOS)
    Threads::Scope lock(data_mutex_);
    if (!data_ready_ || !state_pub_.impl) return;
    rcl_publish(&state_pub_, &state_msg_, NULL);
    data_ready_ = false;
#endif
  }

 private:
  // ── Timing constants (from MinibotCorrectCode.ino) ──
  static constexpr uint32_t ENTER_WAIT_MS = 5000;
  static constexpr uint32_t ENTER_FORWARD_MS = 3000;
  static constexpr uint32_t ENTER_TURN_MS = 2000;
  static constexpr uint32_t EXIT_FORWARD_MS = 3000;

  static constexpr int ENTER_SPEED = 90;
  static constexpr int TURN_SPEED = 60;
  static constexpr float TURN_FACTOR = 0.5f;
  static constexpr int EXIT_SPEED = 220;

  // Pending command IDs (set by service callback, consumed by update)
  static constexpr uint8_t CMD_NONE = 0;
  static constexpr uint8_t CMD_ENTER = 1;
  static constexpr uint8_t CMD_EXIT = 2;

  enum class Phase : uint8_t {
    IDLE,
    ENTER_WAIT,
    ENTER_FORWARD,
    ENTER_TURN_LEFT,
    EXIT_FORWARD,
  };

  void stopMotors() {
    left_.setPWM(0);
    right_.setPWM(0);
    left_.update();
    right_.update();
  }

  void publishState() {
    if (!state_pub_.impl) return;

#if defined(USE_FREERTOS)
    Threads::Scope lock(data_mutex_);
#endif

    uint32_t now_ms = millis();
    state_msg_.header.stamp.sec = (int32_t)(now_ms / 1000);
    state_msg_.header.stamp.nanosec = (uint32_t)((now_ms % 1000) * 1000000);
    state_msg_.state = (phase_ == Phase::IDLE)
                           ? mcu_msgs__msg__MiniRobotState__ARMED
                           : mcu_msgs__msg__MiniRobotState__RUNNING;
    state_msg_.current_task = current_task_;
    data_ready_ = true;
  }

  // ── Service callbacks ──

  static void enterCallback(const void* req, void* res, void* ctx) {
    (void)req;
    auto* self = static_cast<MinibotMissionSubsystem*>(ctx);
    auto* rsp = static_cast<mcu_msgs__srv__Reset_Response*>(res);

    if (self->phase_ != Phase::IDLE || self->pending_cmd_ != CMD_NONE) {
      rsp->success = false;
      return;
    }

    self->pending_cmd_ = CMD_ENTER;
    rsp->success = true;
  }

  static void exitCallback(const void* req, void* res, void* ctx) {
    (void)req;
    auto* self = static_cast<MinibotMissionSubsystem*>(ctx);
    auto* rsp = static_cast<mcu_msgs__srv__Reset_Response*>(res);

    if (self->phase_ != Phase::IDLE || self->pending_cmd_ != CMD_NONE) {
      rsp->success = false;
      return;
    }

    self->pending_cmd_ = CMD_EXIT;
    rsp->success = true;
  }

  // ── State ──
  Drivers::MinibotMotorDriver& left_;
  Drivers::MinibotMotorDriver& right_;

  volatile Phase phase_ = Phase::IDLE;
  volatile uint8_t pending_cmd_ = CMD_NONE;
  uint32_t phase_start_ms_ = 0;
  uint8_t current_task_ = mcu_msgs__msg__MiniRobotState__TASK_NONE;

  // ── micro-ROS entities ──
  rcl_publisher_t state_pub_{};
  mcu_msgs__msg__MiniRobotState state_msg_{};

  rcl_service_t enter_srv_{};
  mcu_msgs__srv__Reset_Request enter_req_{};
  mcu_msgs__srv__Reset_Response enter_res_{};

  rcl_service_t exit_srv_{};
  mcu_msgs__srv__Reset_Request exit_req_{};
  mcu_msgs__srv__Reset_Response exit_res_{};

  rcl_node_t* node_ = nullptr;
  bool data_ready_ = false;
#if defined(USE_FREERTOS)
  Threads::Mutex data_mutex_;
#endif
};

}  // namespace Subsystem
