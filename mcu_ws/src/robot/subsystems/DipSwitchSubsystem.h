/**
 * @file DipSwitchSubsystem.h
 * @brief Reads TCA9555 Port 0 (8 DIP switches) and publishes state.
 * @date 2026-02-24
 *
 * Publishes an 8-bit bitmask where each bit represents one DIP switch
 * (bit 0 = switch 0, etc.).  Hardware is active-low and bit-reversed;
 * this subsystem corrects both so consumers see active-HIGH, bit 0 = switch 0.
 *
 * -- ROS2 interface (defaults) ---------------------------------------------
 *   /mcu_robot/dip_switches   topic   std_msgs/msg/UInt8   (1 Hz)
 */

#pragma once

#include <BaseSubsystem.h>
#include <TCA9555Driver.h>
#include "DebugLog.h"
#include <microros_manager_robot.h>
#include <std_msgs/msg/u_int8.h>

#ifdef USE_TEENSYTHREADS
#include <TeensyThreads.h>
#endif

namespace Subsystem {

class DipSwitchSubsystemSetup : public Classes::BaseSetup {
 public:
  /**
   * @param _id    Subsystem identifier.
   * @param driver TCA9555 driver (port 0 used for DIP switches).
   * @param topic  ROS2 topic name for publishing switch state.
   */
  DipSwitchSubsystemSetup(const char* _id, Drivers::TCA9555Driver* driver,
                          const char* topic = "/mcu_robot/dip_switches")
      : Classes::BaseSetup(_id), driver_(driver), topic_(topic) {}
  Drivers::TCA9555Driver* driver_ = nullptr;
  const char* topic_ = "/mcu_robot/dip_switches";
};

class DipSwitchSubsystem : public IMicroRosParticipant,
                           public Classes::BaseSubsystem {
 public:
  explicit DipSwitchSubsystem(const DipSwitchSubsystemSetup& setup)
      : Classes::BaseSubsystem(setup), setup_(setup) {}

  bool init() override {
    if (!setup_.driver_) {
      DEBUG_PRINTLN("[DIP] init FAIL: no driver");
      return false;
    }
    setup_.driver_->configurePort(0, 0xFF);
    DEBUG_PRINTLN("[DIP] init OK (port 0, 8 switches)");
    return true;
  }

  void begin() override {}

  void update() override {
    if (!setup_.driver_) return;
    setup_.driver_->update();

    if (!pub_.impl) return;
    uint32_t now = millis();
    if (now - last_publish_ms_ >= PUBLISH_INTERVAL_MS) {
      last_publish_ms_ = now;
      msg_.data = fixGPIO(setup_.driver_->readPort(0));
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
    static const char info[] = "DipSwitchSubsystem";
    return info;
  }

  bool onCreate(rcl_node_t* node, rclc_executor_t* executor) override {
    (void)executor;
    node_ = node;
    bool ok = rclc_publisher_init_best_effort(
                  &pub_, node_, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8),
                  setup_.topic_) == RCL_RET_OK;
    DEBUG_PRINTF("[DIP] onCreate %s\n", ok ? "OK" : "FAIL");
    return ok;
  }

  void onDestroy() override {
    pub_ = rcl_get_zero_initialized_publisher();
    node_ = nullptr;
  }

  uint8_t getState() const {
    return setup_.driver_ ? fixGPIO(setup_.driver_->readPort(0)) : 0;
  }

#ifdef USE_TEENSYTHREADS
  void beginThreaded(uint32_t stackSize, int /*priority*/ = 1,
                     uint32_t updateRateMs = 500) {
    task_delay_ms_ = updateRateMs;
    threads.addThread(taskFunction, this, stackSize);
  }

 private:
  static void taskFunction(void* pv) {
    auto* self = static_cast<DipSwitchSubsystem*>(pv);
    self->begin();
    while (true) {
      self->update();
      threads.delay(self->task_delay_ms_);
    }
  }
  uint32_t task_delay_ms_ = 500;
#endif

 private:
  // Hardware is active-low and bit-reversed (switch 0 appears at bit 7).
  // Reverse bits and invert so bit 0 = switch 0, 1 = ON.
  static uint8_t fixGPIO(uint8_t b) {
    b = ((b & 0xF0) >> 4) | ((b & 0x0F) << 4);
    b = ((b & 0xCC) >> 2) | ((b & 0x33) << 2);
    b = ((b & 0xAA) >> 1) | ((b & 0x55) << 1);
    return ~b;
  }

  static constexpr uint32_t PUBLISH_INTERVAL_MS = 1000;
  const DipSwitchSubsystemSetup setup_;
  rcl_publisher_t pub_{};
  std_msgs__msg__UInt8 msg_{};
  rcl_node_t* node_ = nullptr;
  uint32_t last_publish_ms_ = 0;
};

}  // namespace Subsystem
