#include <Arduino.h>
#include "microros_manager_robot.h"
#include <micro_ros_utilities/type_utilities.h>
#include <micro_ros_utilities/string_utilities.h>

namespace Subsystem {

MicrorosManager* MicrorosManager::s_instance_ = nullptr;

// (TF quaternion utility removed)

bool MicrorosManager::create_entities() {
  allocator_ = rcl_get_default_allocator();
  if (rclc_support_init(&support_, 0, NULL, &allocator_) != RCL_RET_OK) return false;
  if (rclc_node_init_default(&node_, "robot_manager", "", &support_) != RCL_RET_OK) return false;
  // (legacy Int32 test publisher removed)
  // Heartbeat publisher
  if (rclc_publisher_init_best_effort(
    &heartbeat_pub_,
    &node_,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "robot_manager/heartbeat") != RCL_RET_OK) return false;
  // State publisher
  if (rclc_publisher_init_best_effort(
    &state_pub_,
    &node_,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "robot_manager/state") != RCL_RET_OK) return false;
  // TF publisher
  // (TF publisher removed)
  const unsigned int timer_timeout = 1000;
    if (rclc_timer_init_default2(
      &timer_,
      &support_,
      RCL_MS_TO_NS(timer_timeout),
      MicrorosManager::timer_callback,
      true) != RCL_RET_OK) return false;
  executor_ = rclc_executor_get_zero_initialized_executor();
  if (rclc_executor_init(&executor_, &support_.context, 1, &allocator_) != RCL_RET_OK) return false;
  if (rclc_executor_add_timer(&executor_, &timer_) != RCL_RET_OK) return false;
  // (TF message allocation removed)
  // Initialize state string
  state_msg_.data = micro_ros_string_utilities_set(state_msg_.data, "INIT");
  return true;
}

void MicrorosManager::destroy_entities() {
  rmw_context_t* rmw_context = rcl_context_get_rmw_context(&support_.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);
  // (legacy Int32 publisher fini removed)
  rcl_ret_t _ret_pub2 = rcl_publisher_fini(&heartbeat_pub_, &node_);
  (void)_ret_pub2;
  rcl_ret_t _ret_pub3 = rcl_publisher_fini(&state_pub_, &node_);
  (void)_ret_pub3;
  // (TF publisher fini removed)
  rcl_ret_t _ret_timer = rcl_timer_fini(&timer_);
  (void)_ret_timer;
  rclc_executor_fini(&executor_);
  rcl_ret_t _ret_node = rcl_node_fini(&node_);
  (void)_ret_node;
  rclc_support_fini(&support_);
  // (TF message memory destroy removed)
}

void MicrorosManager::timer_callback(rcl_timer_t* timer, int64_t last_call_time) {
  (void)last_call_time;
  if (timer != NULL && s_instance_ != nullptr) {
    // Heartbeat tick
    s_instance_->heartbeat_msg_.data++;
    rcl_ret_t _ret_hb = rcl_publish(&s_instance_->heartbeat_pub_, &s_instance_->heartbeat_msg_, NULL);
    (void)_ret_hb;
    // (legacy Int32 publish removed)
    // Publish current state
    rcl_ret_t _ret_state = rcl_publish(&s_instance_->state_pub_, &s_instance_->state_msg_, NULL);
    (void)_ret_state;
    // (TF publish removed)
  }
}

bool MicrorosManager::init() {
  state_ = WAITING_AGENT;
  return true;
}

void MicrorosManager::begin() {
  s_instance_ = this;
  set_microros_transports();
}

void MicrorosManager::update() {
  switch (state_) {
    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(500, state_ = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
      break;
    case AGENT_AVAILABLE:
      state_ = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
      if (state_ == WAITING_AGENT) {
        destroy_entities();
      }
      break;
    case AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(200, state_ = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
      if (state_ == AGENT_CONNECTED) {
        rclc_executor_spin_some(&executor_, RCL_MS_TO_NS(100));
      }
      break;
    case AGENT_DISCONNECTED:
      destroy_entities();
      state_ = WAITING_AGENT;
      break;
    default:
      break;
  }
}

void MicrorosManager::pause() {
  if (state_ == AGENT_CONNECTED) {
    destroy_entities();
  }
  state_ = WAITING_AGENT;
}

void MicrorosManager::reset() {
  pause();
}

const char* MicrorosManager::getInfo() {
  static const char info[] = "MicrorosManager";
  return info;
}

// (TF update function removed)

// (Pose setter removed)

void MicrorosManager::setState(const char* state) {
  state_msg_.data = micro_ros_string_utilities_set(state_msg_.data, state);
}

}  // namespace Subsystem
