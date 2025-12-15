#pragma once
#include "microros_manager_robot.h"
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/string.h>
#include <micro_ros_utilities/string_utilities.h>

namespace Subsystem {

class ExampleSubsystem : public IMicroRosParticipant {
 public:
  bool onCreate(rcl_node_t* node, rclc_executor_t* executor) override {
    (void)executor; // not used in this simple example
    node_ = node;
    if (rclc_publisher_init_best_effort(
          &pub_,
          node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
          "example_subsystem/status") != RCL_RET_OK) {
      return false;
    }
    msg_.data = micro_ros_string_utilities_set(msg_.data, "READY");
    return true;
  }
  void onDestroy() override {
    if (pub_.impl) {
      rcl_ret_t ret = rcl_publisher_fini(&pub_, node_);
      (void)ret;
    }
    if (msg_.data.data) {
      micro_ros_string_utilities_destroy(&msg_.data);
    }
  }
  // Simple API to publish a status update
  void publishStatus(const char* text) {
    if (!pub_.impl) return;
    msg_.data = micro_ros_string_utilities_set(msg_.data, text);
    rcl_ret_t ret = rcl_publish(&pub_, &msg_, NULL);
    (void)ret;
  }
 private:
  rcl_publisher_t pub_{};
  std_msgs__msg__String msg_{};
  rcl_node_t* node_ = nullptr;
};

} // namespace Subsystem
