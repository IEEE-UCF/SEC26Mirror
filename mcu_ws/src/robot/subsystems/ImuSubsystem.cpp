#include "ImuSubsystem.h"

#include <micro_ros_utilities/type_utilities.h>

#include "DebugLog.h"

namespace Subsystem {

bool ImuSubsystem::init() {
  if (!setup_.driver_) {
    DEBUG_PRINTLN("[IMU] init FAIL: no driver");
    return false;
  }
  if (!setup_.driver_->init()) {
    DEBUG_PRINTLN("[IMU] init FAIL: driver init");
    return false;
  }

  return true;
}

void ImuSubsystem::update() {
  if (!setup_.driver_) return;

  setup_.driver_->update();

  // Cache yaw for cross-thread access (e.g., DriveSubsystem localization)
  yaw_rad_.store(setup_.driver_->getData().yaw, std::memory_order_relaxed);

  ++diag_update_count_;
  diag_last_update_ms_ = millis();

  if (!pub_.impl) return;

  // Populate message under data_mutex_ for deferred publishing
  {
    Threads::Scope lock(data_mutex_);
    publishData();
    data_ready_ = true;
  }
}

void ImuSubsystem::reset() { pause(); }

const char* ImuSubsystem::getInfo() {
  static const char info[] = "ImuSubsystem";
  return info;
}

bool ImuSubsystem::onCreate(rcl_node_t* node, rclc_executor_t* executor) {
  (void)executor;
  node_ = node;

  // Initialize the IMU message
  sensor_msgs__msg__Imu__init(&msg_);

  // Initialize covariance matrices
  initCovariances();

  // frame_id will be set in publishData() using a static string pointer.
  // See onDestroy() for why we null it before __fini.

  // Then we create a publisher with best-effort QoS for real-time sensor data
  if (rclc_publisher_init_best_effort(
          &pub_, node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
          "/mcu_robot/imu/data") != RCL_RET_OK) {
    DEBUG_PRINTLN("[IMU] onCreate FAIL: publisher init");
    return false;
  }

  DEBUG_PRINTLN("[IMU] onCreate OK");
  return true;
}

void ImuSubsystem::onDestroy() {
  pub_ = rcl_get_zero_initialized_publisher();
  // Null out the frame_id string BEFORE calling __fini.  publishData() sets
  // frame_id.data to a static char[], and __fini would try to free() it,
  // causing heap corruption.
  msg_.header.frame_id.data = NULL;
  msg_.header.frame_id.size = 0;
  msg_.header.frame_id.capacity = 0;
  sensor_msgs__msg__Imu__fini(&msg_);
  node_ = nullptr;
}

void ImuSubsystem::initCovariances() {
  // Initialize all covariance arrays to zero first
  for (int i = 0; i < 9; i++) {
    msg_.orientation_covariance[i] = 0.0;
    msg_.angular_velocity_covariance[i] = 0.0;
    msg_.linear_acceleration_covariance[i] = 0.0;
  }

  // Set diagonal elements with variance values
  // BNO085 provides orientation via Game Rotation Vector
  msg_.orientation_covariance[0] = kOrientationVariance;  // x
  msg_.orientation_covariance[4] = kOrientationVariance;  // y
  msg_.orientation_covariance[8] = kOrientationVariance;  // z

  // Gyro and accel reports are not enabled — mark as unknown per ROS convention
  msg_.angular_velocity_covariance[0] = -1.0;
  msg_.linear_acceleration_covariance[0] = -1.0;
}

void ImuSubsystem::publishData() {
  if (!pub_.impl || !setup_.driver_) return;

  Drivers::BNO085DriverData data = setup_.driver_->getData();

  // Timestamp using micro-ROS epoch time
  int64_t time_ns = rmw_uros_epoch_nanos();
  msg_.header.stamp.sec = (int32_t)(time_ns / 1000000000);
  msg_.header.stamp.nanosec = (int32_t)(time_ns % 1000000000);

  // Frame ID — points to file-scope static string.  onDestroy() nulls this
  // before calling __fini to prevent freeing a static pointer.
  static char frame_id[] = "imu_link";
  msg_.header.frame_id.data = frame_id;
  msg_.header.frame_id.size = 8;
  msg_.header.frame_id.capacity = 9;

  // Orientation quaternion from Game Rotation Vector
  msg_.orientation.x = data.qx;
  msg_.orientation.y = data.qy;
  msg_.orientation.z = data.qz;
  msg_.orientation.w = data.qw;
}

void ImuSubsystem::publishAll() {
  Threads::Scope lock(data_mutex_);
  if (!data_ready_ || !pub_.impl) return;
  rcl_ret_t rc = rcl_publish(&pub_, &msg_, NULL);
  data_ready_ = false;
  ++diag_pub_count_;
  if (rc != RCL_RET_OK) {
    DEBUG_PRINTF("[IMU] pub FAIL: rc=%d\n", (int)rc);
  }
}

}  // namespace Subsystem
