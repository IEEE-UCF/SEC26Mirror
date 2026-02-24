/**
 * @file UWBSubsystem.h
 * @brief UWB subsystem for DW3000 ranging (Tag/Anchor modes)
 * @author SEC26 Team
 * @date 12/22/2025
 */
#pragma once

#include <Arduino.h>
#include <BaseSubsystem.h>
#include <mcu_msgs/msg/uwb_range.h>
#include <mcu_msgs/msg/uwb_ranging.h>
#include <microros_manager_robot.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <stdio.h>

#include "TimedSubsystem.h"
#include "UWBDriver.h"

namespace Subsystem {

/**
 * @brief Setup configuration for UWBSubsystem.
 */
class UWBSubsystemSetup : public Classes::BaseSetup {
 public:
  Drivers::UWBDriver* driver;

  UWBSubsystemSetup(const char* _id, Drivers::UWBDriver* _driver)
      : Classes::BaseSetup(_id), driver(_driver) {}
};

/**
 * @brief UWB Subsystem for ranging and positioning.
 *
 * This subsystem manages UWB ranging using the DW3000 module.
 * - TAG mode: Initiates ranging to anchors and publishes range data
 * - ANCHOR mode: Responds to ranging requests (no publishing)
 */
class UWBSubsystem : public IMicroRosParticipant,
                     public Subsystem::TimedSubsystem {
 public:
  explicit UWBSubsystem(const UWBSubsystemSetup& setup)
      : Subsystem::TimedSubsystem(setup), setup_(setup) {}

  bool init() override;
  void begin() override;
  void update() override;
  void pause() override;
  void reset() override;
  const char* getInfo() override {
    static const char info[] = "UWBSubsystem";
    return info;
  }

  // IMicroRosParticipant interface
  bool onCreate(rcl_node_t* node, rclc_executor_t* executor) override;
  void onDestroy() override;

 private:
  void publishRanging();
  void updateRangingMessage();
  void publishPeerRanges();

  static constexpr uint32_t PUBLISH_INTERVAL_MS = 100;       // 10 Hz (TAG mode)
  static constexpr uint32_t RANGING_INTERVAL_MS = 50;        // 20 Hz (TAG mode)
  static constexpr uint32_t PEER_PUBLISH_INTERVAL_MS = 500;  // 2 Hz (inter-beacon)
  static constexpr uint8_t MAX_PEER_PUBS = 2;  // max peers per beacon

  const UWBSubsystemSetup setup_;

  // TAG mode publisher
  rcl_publisher_t pub_{};
  mcu_msgs__msg__UWBRanging msg_{};
  rcl_node_t* node_ = nullptr;

  bool is_tag_mode_ = false;
  elapsedMillis ranging_timer_;

  // Inter-beacon peer publishers (ANCHOR mode with peers)
  bool has_peer_ranging_ = false;
  rcl_publisher_t peer_pubs_[MAX_PEER_PUBS] = {};
  mcu_msgs__msg__UWBRange peer_msgs_[MAX_PEER_PUBS] = {};
  char peer_topic_names_[MAX_PEER_PUBS][32] = {};
  uint8_t num_peer_pubs_ = 0;
};

}  // namespace Subsystem
