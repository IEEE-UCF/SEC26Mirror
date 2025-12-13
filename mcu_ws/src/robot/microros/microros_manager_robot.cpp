#include <Arduino.h>
#include "microros_manager_robot.h"
#include <micro_ros_utilities/type_utilities.h>
#include <micro_ros_utilities/string_utilities.h>
#include <uxr/client/util/time.h>

namespace Subsystem {

MicrorosManager* MicrorosManager::s_instance_ = nullptr;

static void euler_to_quat(float x, float y, float z, double* q) {
  float c1 = cos((y*3.14/180.0f)/2);
  float c2 = cos((z*3.14/180.0f)/2);
  float c3 = cos((x*3.14/180.0f)/2);
  float s1 = sin((y*3.14/180.0f)/2);
  float s2 = sin((z*3.14/180.0f)/2);
  float s3 = sin((x*3.14/180.0f)/2);
  q[0] = c1 * c2 * c3 - s1 * s2 * s3;
  q[1] = s1 * s2 * c3 + c1 * c2 * s3;
  q[2] = s1 * c2 * c3 + c1 * s2 * s3;
  q[3] = c1 * s2 * c3 - s1 * c2 * s3;
}

bool MicrorosManager::create_entities() {
  allocator_ = rcl_get_default_allocator();
  if (rclc_support_init(&support_, 0, NULL, &allocator_) != RCL_RET_OK) return false;
  if (rclc_node_init_default(&node_, "robot_manager", "", &support_) != RCL_RET_OK) return false;
  // Int32 test publisher (legacy)
  if (rclc_publisher_init_best_effort(
    &publisher_,
    &node_,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "robot_manager/int32") != RCL_RET_OK) return false;
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
  if (rclc_publisher_init_default(
    &tf_pub_,
    &node_,
    ROSIDL_GET_MSG_TYPE_SUPPORT(tf2_msgs, msg, TFMessage),
    "/tf") != RCL_RET_OK) return false;
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
  // Allocate TF message with two transforms (base and inertial)
  if (!micro_ros_utilities_create_message_memory(
    ROSIDL_GET_MSG_TYPE_SUPPORT(tf2_msgs, msg, TFMessage),
    &tf_message_,
    (micro_ros_utilities_memory_conf_t){})) {
    return false;
  }
  tf_message_->transforms.size = 2;
  tf_message_->transforms.data[0].header.frame_id =
      micro_ros_string_utilities_set(tf_message_->transforms.data[0].header.frame_id, "/base_link");
  tf_message_->transforms.data[1].header.frame_id =
      micro_ros_string_utilities_set(tf_message_->transforms.data[1].header.frame_id, "/inertial_unit");
  // Initialize state string
  state_msg_.data = micro_ros_string_utilities_set(state_msg_.data, "INIT");
  return true;
}

void MicrorosManager::destroy_entities() {
  rmw_context_t* rmw_context = rcl_context_get_rmw_context(&support_.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);
  rcl_ret_t _ret_pub1 = rcl_publisher_fini(&publisher_, &node_);
  (void)_ret_pub1;
  rcl_ret_t _ret_pub2 = rcl_publisher_fini(&heartbeat_pub_, &node_);
  (void)_ret_pub2;
  rcl_ret_t _ret_pub3 = rcl_publisher_fini(&state_pub_, &node_);
  (void)_ret_pub3;
  rcl_ret_t _ret_pub4 = rcl_publisher_fini(&tf_pub_, &node_);
  (void)_ret_pub4;
  rcl_ret_t _ret_timer = rcl_timer_fini(&timer_);
  (void)_ret_timer;
  rclc_executor_fini(&executor_);
  rcl_ret_t _ret_node = rcl_node_fini(&node_);
  (void)_ret_node;
  rclc_support_fini(&support_);
  if (tf_message_) {
    micro_ros_utilities_destroy_message_memory(
        ROSIDL_GET_MSG_TYPE_SUPPORT(tf2_msgs, msg, TFMessage), tf_message_,
        (micro_ros_utilities_memory_conf_t){});
    tf_message_ = nullptr;
  }
}

void MicrorosManager::timer_callback(rcl_timer_t* timer, int64_t last_call_time) {
  (void)last_call_time;
  if (timer != NULL && s_instance_ != nullptr) {
    // Heartbeat tick
    s_instance_->heartbeat_msg_.data++;
    rcl_ret_t _ret_hb = rcl_publish(&s_instance_->heartbeat_pub_, &s_instance_->heartbeat_msg_, NULL);
    (void)_ret_hb;
    // Publish current state
    rcl_ret_t _ret_state = rcl_publish(&s_instance_->state_pub_, &s_instance_->state_msg_, NULL);
    (void)_ret_state;
    // Update and publish TF
    s_instance_->update_tf_message();
    rcl_ret_t _ret_tf = rcl_publish(&s_instance_->tf_pub_, s_instance_->tf_message_, NULL);
    (void)_ret_tf;
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

void MicrorosManager::update_tf_message() {
  if (!tf_message_) return;
  // Transform 0: base_link position and orientation from cached pose
  geometry_msgs__msg__TransformStamped* t0 = &tf_message_->transforms.data[0];
  t0->transform.translation.x = pos_x_;
  t0->transform.translation.y = pos_y_;
  t0->transform.translation.z = pos_z_;
  double q[4];
  euler_to_quat(roll_, pitch_, yaw_, q);
  t0->transform.rotation.x = (double)q[1];
  t0->transform.rotation.y = (double)q[2];
  t0->transform.rotation.z = (double)q[3];
  t0->transform.rotation.w = (double)q[0];
  // Timestamp
  int64_t ms = uxr_millis();
  t0->header.stamp.sec = (int32_t)(ms / 1000LL);
  t0->header.stamp.nanosec = (uint32_t)((ms % 1000LL) * 1000000UL);
}

void MicrorosManager::setPose(float x, float y, float z, float roll, float pitch, float yaw) {
  pos_x_ = x; pos_y_ = y; pos_z_ = z;
  roll_ = roll; pitch_ = pitch; yaw_ = yaw;
}

void MicrorosManager::setState(const char* state) {
  state_msg_.data = micro_ros_string_utilities_set(state_msg_.data, state);
}

}  // namespace Subsystem
