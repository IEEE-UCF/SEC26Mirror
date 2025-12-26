/**
 * @file RCSubsystem.h
 * @brief Manages FlySky RC receiver and publishes RC data to ROS 2.
 * @author Generated for SEC26
 * @date 12/21/2025
 */
#pragma once

#include <Arduino.h>
#include <BaseSubsystem.h>
#include <IBusBM.h>
#include <elapsedMillis.h>
#include <mcu_msgs/msg/rc.h>
#include <microros_manager_robot.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>

#include "TimedSubsystem.h"

namespace Subsystem {

/**
 * @brief Data structure containing RC channel values and state.
 */
class RCSubsystemData {
 public:
  int32_t channels[10];  // Raw channel values from receiver (-255 to 255)

  RCSubsystemData() {
    // Initialize all channels to 0
    for (int i = 0; i < 10; i++) {
      channels[i] = 0;
    }
  }
};

/**
 * @brief Setup configuration for RCSubsystem.
 */
class RCSubsystemSetup : public Classes::BaseSetup {
 public:
  HardwareSerial* serial;

  RCSubsystemSetup(const char* _id, HardwareSerial* _serial)
      : Classes::BaseSetup(_id), serial(_serial) {}
};

/**
 * @brief RC Subsystem that reads FlySky receiver data and publishes to ROS 2.
 *
 * This subsystem manages communication with a FlySky RC receiver using the
 * IBUS protocol. It periodically reads channel values and publishes them
 * to the "mcu_robot/rc" topic. No input commands from Raspberry Pi are
 * required.
 */
class RCSubsystem : public IMicroRosParticipant,
                    public Subsystem::TimedSubsystem {
 public:
  explicit RCSubsystem(const RCSubsystemSetup& setup)
      : Subsystem::TimedSubsystem(setup), setup_(setup), updateTimer_(0) {}

  bool init() override;
  void begin() override;
  void update() override;
  void pause() override;
  void reset() override;
  const char* getInfo() override {
    static const char info[] = "RCSubsystem";
    return info;
  }

  // IMicroRosParticipant interface
  bool onCreate(rcl_node_t* node, rclc_executor_t* executor) override;
  void onDestroy() override;

 private:
  void updateRCData();
  void publishRC();
  void updateRCMessage();

  static constexpr uint32_t UPDATE_DELAY_MS = 2;  // 2ms delay for IBUS updates
  static constexpr uint32_t PUBLISH_INTERVAL_MS = 50;  // Publish at 20 Hz

  const RCSubsystemSetup setup_;
  RCSubsystemData data_;

  IBusBM ibus_;
  elapsedMillis updateTimer_;

  rcl_publisher_t pub_{};
  mcu_msgs__msg__RC msg_{};
  rcl_node_t* node_ = nullptr;
};

}  // namespace Subsystem
