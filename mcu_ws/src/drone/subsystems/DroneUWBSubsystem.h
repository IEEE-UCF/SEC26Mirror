#pragma once
/**
 * @file DroneUWBSubsystem.h
 * @brief Thin ROS2 publisher wrapper for /mcu_drone/uwb/ranging.
 *        Publishes UWBRanging data from the UWBDriver (TAG mode).
 */

#include <microros_manager_robot.h>
#include <mcu_msgs/msg/uwb_range.h>
#include <mcu_msgs/msg/uwb_ranging.h>
#include <UWBDriver.h>

namespace Drone {

class DroneUWBSubsystem : public Subsystem::IMicroRosParticipant {
 public:
  explicit DroneUWBSubsystem(Drivers::UWBDriver& uwb) : uwb_(uwb) {}

  bool onCreate(rcl_node_t* node, rclc_executor_t* /*executor*/) override {
    // Initialize msg with dynamic ranges array
    mcu_msgs__msg__UWBRanging__init(&msg_);
    msg_.ranges.capacity = Drivers::UWBDriverData::MAX_ANCHORS;
    msg_.ranges.size = 0;
    msg_.ranges.data = (mcu_msgs__msg__UWBRange*)malloc(
        msg_.ranges.capacity * sizeof(mcu_msgs__msg__UWBRange));
    if (!msg_.ranges.data) return false;
    for (size_t i = 0; i < msg_.ranges.capacity; i++) {
      mcu_msgs__msg__UWBRange__init(&msg_.ranges.data[i]);
    }

    if (rclc_publisher_init_best_effort(
            &pub_, node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(mcu_msgs, msg, UWBRanging),
            "/mcu_drone/uwb/ranging") != RCL_RET_OK) {
      free(msg_.ranges.data);
      msg_.ranges.data = nullptr;
      return false;
    }
    return true;
  }

  void onDestroy() override {
    pub_.impl = nullptr;
    if (msg_.ranges.data) {
      free(msg_.ranges.data);
      msg_.ranges.data = nullptr;
    }
    mcu_msgs__msg__UWBRanging__fini(&msg_);
  }

  // Call from UWB FreeRTOS task after ranging completes
  void publish() {
    if (!pub_.impl) return;

    const auto& data = uwb_.getData();
    msg_.header.stamp.sec = (int32_t)(millis() / 1000);
    msg_.header.stamp.nanosec = (uint32_t)((millis() % 1000) * 1000000);
    msg_.tag_id = data.device_id;
    msg_.temperature = data.temperature;

    msg_.ranges.size = 0;
    for (size_t i = 0; i < Drivers::UWBDriverData::MAX_ANCHORS; i++) {
      const auto& range = data.ranges[i];
      if (range.valid && msg_.ranges.size < msg_.ranges.capacity) {
        auto& mr = msg_.ranges.data[msg_.ranges.size];
        mr.header.stamp = msg_.header.stamp;
        mr.tag_id = data.device_id;
        mr.anchor_id = range.peer_id;
        mr.distance = range.distance_cm;
        mr.signal_strength = 0.0f;
        mr.clock_offset = range.clock_offset;
        mr.tx_timestamp = range.tx_timestamp;
        mr.rx_timestamp = range.rx_timestamp;
        mr.valid = range.valid;
        mr.error_code = range.error_code;
        msg_.ranges.size++;
      }
    }
    msg_.num_anchors = msg_.ranges.size;

    rcl_publish(&pub_, &msg_, NULL);
  }

 private:
  Drivers::UWBDriver& uwb_;
  rcl_publisher_t pub_{};
  mcu_msgs__msg__UWBRanging msg_{};
};

}  // namespace Drone
