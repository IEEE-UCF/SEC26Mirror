/**
 * @file LightSubsystem.h
 * @brief BH1750 ambient light sensor subsystem.
 *
 * Reads lux from a BH1750 (optionally via TCA9548A mux) and publishes
 * to micro-ROS.
 *
 * -- ROS2 interface (defaults) ------------------------------------------------
 *   /mcu_robot/light   topic   std_msgs/msg/Float32   (update rate)
 */

#pragma once

#include <RTOSSubsystem.h>
#include <BH1750Driver.h>
#include <microros_manager_robot.h>
#include <std_msgs/msg/float32.h>
#include "DebugLog.h"

namespace Subsystem {

class LightSubsystemSetup : public Classes::BaseSetup {
 public:
  LightSubsystemSetup(const char* _id, Drivers::BH1750Driver* driver,
                      const char* topic = "/mcu_robot/light")
      : Classes::BaseSetup(_id), driver_(driver), topic_(topic) {}
  Drivers::BH1750Driver* driver_ = nullptr;
  const char* topic_ = "/mcu_robot/light";
};

class LightSubsystem : public IMicroRosParticipant,
                       public Subsystem::RTOSSubsystem {
 public:
  explicit LightSubsystem(const LightSubsystemSetup& setup)
      : Subsystem::RTOSSubsystem(setup), setup_(setup) {}

  bool init() override {
    if (!setup_.driver_) {
      DEBUG_PRINTLN("[LIGHT] init FAIL: no driver");
      return false;
    }
    bool ok = setup_.driver_->init();
    DEBUG_PRINTF("[LIGHT] init: %s\n", ok ? "OK" : "FAIL");
    return ok;
  }

  void begin() override {}

  void update() override {
    if (!setup_.driver_) return;
    setup_.driver_->update();

    if (!pub_.impl) return;
    {
      Threads::Scope lock(data_mutex_);
      msg_.data = setup_.driver_->getLux();
      data_ready_ = true;
    }
  }

  void pause() override {}
  void reset() override {}

  const char* getInfo() override {
    static const char info[] = "LightSubsystem";
    return info;
  }

  bool onCreate(rcl_node_t* node, rclc_executor_t* executor) override {
    (void)executor;
    node_ = node;
    bool ok = rclc_publisher_init_best_effort(
                  &pub_, node_,
                  ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
                  setup_.topic_) == RCL_RET_OK;
    DEBUG_PRINTF("[LIGHT] onCreate %s\n", ok ? "OK" : "FAIL");
    return ok;
  }

  void onDestroy() override {
    pub_ = rcl_get_zero_initialized_publisher();
    node_ = nullptr;
  }

  void publishAll() override {
    Threads::Scope lock(data_mutex_);
    if (!data_ready_ || !pub_.impl) return;
    (void)rcl_publish(&pub_, &msg_, NULL);
    data_ready_ = false;
  }

  float getLux() const {
    return setup_.driver_ ? setup_.driver_->getLux() : 0.0f;
  }

 private:
  const LightSubsystemSetup setup_;
  rcl_publisher_t pub_{};
  std_msgs__msg__Float32 msg_{};
  rcl_node_t* node_ = nullptr;
  bool data_ready_ = false;
  Threads::Mutex data_mutex_;
};

}  // namespace Subsystem
