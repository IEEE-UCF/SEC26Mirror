#include <Arduino.h>
#include "microros_manager_example.h"

namespace Subsystem {

MicrorosManager* MicrorosManager::s_instance_ = nullptr;

bool MicrorosManager::create_entities() {
  allocator_ = rcl_get_default_allocator();
  if (rclc_support_init(&support_, 0, NULL, &allocator_) != RCL_RET_OK) return false;
  if (rclc_node_init_default(&node_, "int32_publisher_rclc", "", &support_) != RCL_RET_OK) return false;
  if (rclc_publisher_init_best_effort(
          &publisher_,
          &node_,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
          "std_msgs_msg_Int32") != RCL_RET_OK) return false;
  const unsigned int timer_timeout = 1000;
  if (rclc_timer_init_default(
          &timer_,
          &support_,
          RCL_MS_TO_NS(timer_timeout),
          MicrorosManager::timer_callback) != RCL_RET_OK) return false;
  executor_ = rclc_executor_get_zero_initialized_executor();
  if (rclc_executor_init(&executor_, &support_.context, 1, &allocator_) != RCL_RET_OK) return false;
  if (rclc_executor_add_timer(&executor_, &timer_) != RCL_RET_OK) return false;
  return true;
}

void MicrorosManager::destroy_entities() {
  rmw_context_t* rmw_context = rcl_context_get_rmw_context(&support_.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);
  rcl_publisher_fini(&publisher_, &node_);
  rcl_timer_fini(&timer_);
  rclc_executor_fini(&executor_);
  rcl_node_fini(&node_);
  rclc_support_fini(&support_);
}

void MicrorosManager::timer_callback(rcl_timer_t* timer, int64_t last_call_time) {
  (void)last_call_time;
  if (timer != NULL && s_instance_ != nullptr) {
    std_msgs__msg__Int32* m = &s_instance_->msg_;
    rcl_publish(&s_instance_->publisher_, m, NULL);
    m->data++;
  }
}

bool MicrorosManager::init() {
  msg_.data = 0;
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
  msg_.data = 0;
}

const char* MicrorosManager::getInfo() {
  static const char info[] = "MicrorosManager";
  return info;
}

}  // namespace Subsystem
