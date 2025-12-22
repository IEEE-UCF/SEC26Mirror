/**
 * @file UWBSubsystem.cpp
 * @brief Implementation of UWB subsystem
 * @author SEC26 Team
 * @date 12/22/2025
 */

#include "UWBSubsystem.h"

namespace Subsystem {

bool UWBSubsystem::init() {
  if (!setup_.driver) {
    return false;
  }

  // Initialize the UWB driver
  if (!setup_.driver->init()) {
    return false;
  }

  // Check if we're in tag mode (only tags publish data)
  const auto& driver_data = setup_.driver->getData();
  is_tag_mode_ = (driver_data.mode == Drivers::UWBMode::TAG);

  return true;
}

void UWBSubsystem::begin() {
  if (setup_.driver) {
    setup_.driver->begin();
  }
  ranging_timer_ = 0;
}

void UWBSubsystem::update() {
  if (!setup_.driver) {
    return;
  }

  // Update the driver
  setup_.driver->update();

  // If in tag mode, initiate ranging periodically
  if (is_tag_mode_ && ranging_timer_ >= RANGING_INTERVAL_MS) {
    setup_.driver->startRanging();
    ranging_timer_ = 0;
  }

  // Publish ranging data if in tag mode
  if (is_tag_mode_ && pub_.impl) {
    if (everyMs(PUBLISH_INTERVAL_MS)) {
      publishRanging();
    }
  }
}

void UWBSubsystem::pause() {
  if (setup_.driver) {
    setup_.driver->pause();
  }
}

void UWBSubsystem::reset() {
  if (setup_.driver) {
    setup_.driver->reset();
  }
  pause();
}

bool UWBSubsystem::onCreate(rcl_node_t* node, rclc_executor_t* executor) {
  (void)executor;
  node_ = node;

  // Only create publisher if in tag mode
  if (!is_tag_mode_) {
    return true;  // Anchors don't publish
  }

  // Initialize the UWBRanging message
  mcu_msgs__msg__UWBRanging__init(&msg_);

  // Allocate memory for ranges array (max 4 anchors)
  msg_.ranges.capacity = 4;
  msg_.ranges.size = 0;
  msg_.ranges.data = (mcu_msgs__msg__UWBRange*)malloc(
      msg_.ranges.capacity * sizeof(mcu_msgs__msg__UWBRange));

  if (!msg_.ranges.data) {
    return false;
  }

  // Initialize each range message
  for (size_t i = 0; i < msg_.ranges.capacity; i++) {
    mcu_msgs__msg__UWBRange__init(&msg_.ranges.data[i]);
  }

  // Create publisher for UWB ranging data
  if (rclc_publisher_init_best_effort(
          &pub_, node, ROSIDL_GET_MSG_TYPE_SUPPORT(mcu_msgs, msg, UWBRanging),
          "mcu_uwb/ranging") != RCL_RET_OK) {
    free(msg_.ranges.data);
    msg_.ranges.data = nullptr;
    return false;
  }

  return true;
}

void UWBSubsystem::onDestroy() {
  if (pub_.impl) {
    (void)rcl_publisher_fini(&pub_, node_);
  }

  // Free ranges array
  if (msg_.ranges.data) {
    free(msg_.ranges.data);
    msg_.ranges.data = nullptr;
  }

  mcu_msgs__msg__UWBRanging__fini(&msg_);
  node_ = nullptr;
}

void UWBSubsystem::updateRangingMessage() {
  if (!setup_.driver) {
    return;
  }

  const auto& driver_data = setup_.driver->getData();

  // Update header timestamp
  msg_.header.stamp.sec = (int32_t)(millis() / 1000);
  msg_.header.stamp.nanosec = (uint32_t)((millis() % 1000) * 1000000);

  // Set tag ID
  msg_.tag_id = driver_data.device_id;

  // Set temperature
  msg_.temperature = driver_data.temperature;

  // Copy range measurements
  msg_.ranges.size = 0;
  for (size_t i = 0; i < Drivers::UWBDriverData::MAX_ANCHORS; i++) {
    const auto& range = driver_data.ranges[i];
    if (range.valid && msg_.ranges.size < msg_.ranges.capacity) {
      auto& msg_range = msg_.ranges.data[msg_.ranges.size];

      // Update header
      msg_range.header.stamp = msg_.header.stamp;

      // Set IDs
      msg_range.tag_id = driver_data.device_id;
      msg_range.anchor_id = range.peer_id;

      // Set measurement data
      msg_range.distance = range.distance_cm;
      msg_range.signal_strength = 0.0f;  // Not available from DW3000 in this mode
      msg_range.clock_offset = range.clock_offset;
      msg_range.tx_timestamp = range.tx_timestamp;
      msg_range.rx_timestamp = range.rx_timestamp;

      // Set status
      msg_range.valid = range.valid;
      msg_range.error_code = range.error_code;

      msg_.ranges.size++;
    }
  }

  msg_.num_anchors = msg_.ranges.size;
}

void UWBSubsystem::publishRanging() {
  if (!pub_.impl || !is_tag_mode_) {
    return;
  }

  updateRangingMessage();
  (void)rcl_publish(&pub_, &msg_, NULL);
}

}  // namespace Subsystem
