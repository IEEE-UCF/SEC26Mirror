#include "SensorSubsystem.h"

#include <micro_ros_utilities/type_utilities.h>

namespace Subsystem {

bool SensorSubsystem::init() {
  // Initialize all TOF drivers
  for (auto* driver : setup_.drivers_) {
    if (driver && !driver->init()) {
      return false;
    }
  }
  return true;
}

void SensorSubsystem::update() {
  if (!pub_.impl) return;
  publishData();
}

void SensorSubsystem::reset() { pause(); }

const char* SensorSubsystem::getInfo() {
  static const char info[] = "SensorSubsystem";
  return info;
}

bool SensorSubsystem::onCreate(rcl_node_t* node, rclc_executor_t* executor) {
  (void)executor;
  node_ = node;

  if (rclc_publisher_init_best_effort(
          &pub_, node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
          "/mcu_robot/tof_distances") != RCL_RET_OK) {
    return false;
  }

  // Allocate memory for the float array
  size_t num_sensors = setup_.drivers_.size();
  msg_.data.capacity = num_sensors;
  msg_.data.size = num_sensors;
  msg_.data.data = (float*)malloc(num_sensors * sizeof(float));

  if (!msg_.data.data) {
    return false;
  }

  // Initialize to zero
  for (size_t i = 0; i < num_sensors; i++) {
    msg_.data.data[i] = 0.0f;
  }

  return true;
}

void SensorSubsystem::onDestroy() {
  if (msg_.data.data) {
    free(msg_.data.data);
    msg_.data.data = nullptr;
    msg_.data.size = 0;
    msg_.data.capacity = 0;
  }
  // destroy_entities() finalises the rcl_node before calling onDestroy, so
  // rcl_*_fini would leave impl non-NULL on error; reset local state only.
  pub_ = rcl_get_zero_initialized_publisher();
  node_ = nullptr;
}

void SensorSubsystem::publishData() {
  if (!pub_.impl || setup_.drivers_.empty()) return;

  // Read all TOF sensors and convert to meters
  for (size_t i = 0; i < setup_.drivers_.size(); i++) {
    if (setup_.drivers_[i]) {
      setup_.drivers_[i]->update();
      Drivers::TOFDriverData data = setup_.drivers_[i]->read();
      // Convert from mm to meters
      msg_.data.data[i] = data.range / 1000.0f;
    } else {
      msg_.data.data[i] = 0.0f;
    }
  }

  (void)rcl_publish(&pub_, &msg_, NULL);
}

}  // namespace Subsystem