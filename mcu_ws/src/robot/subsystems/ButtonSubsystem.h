/**
 * @file ButtonSubsystem.h
 * @brief Reads TCA9555 Port 1 (8 push buttons), publishes state, fires
 *        callbacks on press events.
 * @date 2026-02-24
 *
 * Publishes an 8-bit bitmask (bit N = button N).
 * Callbacks are invoked on the rising edge of each button (press).
 *
 * -- ROS2 interface (defaults) ---------------------------------------------
 *   /mcu_robot/buttons   topic   std_msgs/msg/UInt8   (10 Hz)
 */

#pragma once

#include <BaseSubsystem.h>
#include <TCA9555Driver.h>
#include "DebugLog.h"
#include <microros_manager_robot.h>
#include <std_msgs/msg/u_int8.h>

#include <functional>

#ifdef USE_TEENSYTHREADS
#include <TeensyThreads.h>
#endif

namespace Subsystem {

class ButtonSubsystemSetup : public Classes::BaseSetup {
 public:
  /**
   * @param _id    Subsystem identifier.
   * @param driver TCA9555 driver (port 1 used for buttons).
   * @param topic  ROS2 topic name for publishing button state.
   */
  ButtonSubsystemSetup(const char* _id, Drivers::TCA9555Driver* driver,
                       const char* topic = "/mcu_robot/buttons")
      : Classes::BaseSetup(_id), driver_(driver), topic_(topic) {}
  Drivers::TCA9555Driver* driver_ = nullptr;
  const char* topic_ = "/mcu_robot/buttons";
};

class ButtonSubsystem : public IMicroRosParticipant,
                        public Classes::BaseSubsystem {
 public:
  static constexpr uint8_t NUM_BUTTONS = 8;

  explicit ButtonSubsystem(const ButtonSubsystemSetup& setup)
      : Classes::BaseSubsystem(setup), setup_(setup) {}

  bool init() override {
    if (!setup_.driver_) {
      DEBUG_PRINTLN("[BTN] init FAIL: no driver");
      return false;
    }
    setup_.driver_->configurePort(1, 0xFF);
    DEBUG_PRINTLN("[BTN] init OK (port 1, 8 buttons)");
    return true;
  }

  void begin() override {}

  void update() override {
    if (!setup_.driver_) return;
    setup_.driver_->update();

    uint8_t current = setup_.driver_->readPort(1);

    // Detect rising edges (not-pressed -> pressed)
    uint8_t pressed = current & ~prev_state_;
    for (uint8_t i = 0; i < NUM_BUTTONS; i++) {
      if ((pressed >> i) & 0x01) {
        DEBUG_PRINTF("[BTN] press: button %d\n", i);
        if (callbacks_[i]) callbacks_[i]();
      }
    }
    prev_state_ = current;

    if (!pub_.impl) return;
    uint32_t now = millis();
    if (now - last_publish_ms_ >= PUBLISH_INTERVAL_MS) {
      last_publish_ms_ = now;
      msg_.data = current;
#ifdef USE_TEENSYTHREADS
      {
        Threads::Scope guard(g_microros_mutex);
        (void)rcl_publish(&pub_, &msg_, NULL);
      }
#else
      (void)rcl_publish(&pub_, &msg_, NULL);
#endif
    }
  }

  void pause() override {}
  void reset() override { pause(); }
  const char* getInfo() override {
    static const char info[] = "ButtonSubsystem";
    return info;
  }

  bool onCreate(rcl_node_t* node, rclc_executor_t* executor) override {
    (void)executor;
    node_ = node;
    bool ok = rclc_publisher_init_best_effort(
                  &pub_, node_, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8),
                  setup_.topic_) == RCL_RET_OK;
    DEBUG_PRINTF("[BTN] onCreate %s\n", ok ? "OK" : "FAIL");
    return ok;
  }

  void onDestroy() override {
    pub_ = rcl_get_zero_initialized_publisher();
    node_ = nullptr;
  }

  void onPress(uint8_t button, std::function<void()> cb) {
    if (button < NUM_BUTTONS) callbacks_[button] = cb;
  }

  uint8_t getState() const { return prev_state_; }

  bool isPressed(uint8_t button) const {
    return button < NUM_BUTTONS && ((prev_state_ >> button) & 0x01);
  }

#ifdef USE_TEENSYTHREADS
  void beginThreaded(uint32_t stackSize, int /*priority*/ = 1,
                     uint32_t updateRateMs = 20) {
    task_delay_ms_ = updateRateMs;
    threads.addThread(taskFunction, this, stackSize);
  }

 private:
  static void taskFunction(void* pv) {
    auto* self = static_cast<ButtonSubsystem*>(pv);
    self->begin();
    while (true) {
      self->update();
      threads.delay(self->task_delay_ms_);
    }
  }
  uint32_t task_delay_ms_ = 20;
#endif

 private:
  static constexpr uint32_t PUBLISH_INTERVAL_MS = 100;
  const ButtonSubsystemSetup setup_;
  rcl_publisher_t pub_{};
  std_msgs__msg__UInt8 msg_{};
  rcl_node_t* node_ = nullptr;
  uint32_t last_publish_ms_ = 0;
  uint8_t prev_state_ = 0;
  std::function<void()> callbacks_[NUM_BUTTONS];
};

}  // namespace Subsystem
