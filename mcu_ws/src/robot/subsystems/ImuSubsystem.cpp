#include "ImuSubsystem.h"

#include <micro_ros_utilities/type_utilities.h>

#include "DebugLog.h"

namespace Subsystem {

// Frame ID for TF and robot_localization compatibility
static char frame_id[] = "imu_link";

bool ImuSubsystem::init() {
  if (!setup_.driver_) {
    DEBUG_PRINTLN("[IMU] init FAIL: no driver");
    return false;
  }
  if (!setup_.driver_->init()) {
    DEBUG_PRINTLN("[IMU] init FAIL: driver init");
    return false;
  }

  // Calibrates the gyroscope by averaging samples at rest
  // This removes static bias from gyro readings!!!
  DEBUG_PRINTF("[IMU] Calibrating gyro (%d samples)...\n", kCalibrationSamples);
  float sum_x = 0.0f, sum_y = 0.0f, sum_z = 0.0f;

  for (int i = 0; i < kCalibrationSamples; i++) {
    setup_.driver_->update();
    Drivers::BNO085DriverData data = setup_.driver_->getData();
    sum_x += data.gyro_x;
    sum_y += data.gyro_y;
    sum_z += data.gyro_z;
    delay(kCalibrationDelayMs);
  }

  gyro_offset_x_ = sum_x / kCalibrationSamples;
  gyro_offset_y_ = sum_y / kCalibrationSamples;
  gyro_offset_z_ = sum_z / kCalibrationSamples;
  calibrated_ = true;

  DEBUG_PRINTF("[IMU] Calibration done: offset=(%.4f, %.4f, %.4f)\n",
               gyro_offset_x_, gyro_offset_y_, gyro_offset_z_);
  return true;
}

void ImuSubsystem::update() {
  if (!setup_.driver_) return;

  setup_.driver_->update();

  if (!pub_.impl) return;
  publishData();
}

void ImuSubsystem::reset() { pause(); }

const char* ImuSubsystem::getInfo() {
  static const char info[] = "ImuSubsystem";
  return info;
}

bool ImuSubsystem::onCreate(rcl_node_t* node, rclc_executor_t* executor) {
  (void)executor;
  node_ = node;

  // Initialize the IMU message ofc..
  sensor_msgs__msg__Imu__init(&msg_);

  // Initialize covariance matrices
  initCovariances();

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
  // destroy_entities() finalises the rcl_node before calling onDestroy, so
  // rcl_*_fini would leave impl non-NULL on error; reset local state only.
  pub_ = rcl_get_zero_initialized_publisher();
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
  // BNO085 provides orientation, so we set valid covariances!
  msg_.orientation_covariance[0] = kOrientationVariance;  // x
  msg_.orientation_covariance[4] = kOrientationVariance;  // y
  msg_.orientation_covariance[8] = kOrientationVariance;  // z

  msg_.angular_velocity_covariance[0] = kAngularVelocityVariance;  // x
  msg_.angular_velocity_covariance[4] = kAngularVelocityVariance;  // y
  msg_.angular_velocity_covariance[8] = kAngularVelocityVariance;  // z

  msg_.linear_acceleration_covariance[0] = kLinearAccelerationVariance;  // x
  msg_.linear_acceleration_covariance[4] = kLinearAccelerationVariance;  // y
  msg_.linear_acceleration_covariance[8] = kLinearAccelerationVariance;  // z
}

void ImuSubsystem::publishData() {
  if (!pub_.impl || !setup_.driver_) return;

  Drivers::BNO085DriverData data = setup_.driver_->getData();

  // Timestamp using micro-ROS epoch time
  int64_t time_ns = rmw_uros_epoch_nanos();
  msg_.header.stamp.sec = (int32_t)(time_ns / 1000000000);
  msg_.header.stamp.nanosec = (int32_t)(time_ns % 1000000000);

  // Frame ID
  msg_.header.frame_id.data = frame_id;
  msg_.header.frame_id.size = strlen(frame_id);
  msg_.header.frame_id.capacity = strlen(frame_id) + 1;

  // Orientation quaternion (Looking at the documentation and I think the BNO085
  // provides this directly)
  msg_.orientation.x = data.qx;
  msg_.orientation.y = data.qy;
  msg_.orientation.z = data.qz;
  msg_.orientation.w = data.qw;

  // Angular velocity with calibration offset and dead zone filter
  // BNO085 already outputs in rad/s (SI units)
  float gyro_x = data.gyro_x - gyro_offset_x_;
  float gyro_y = data.gyro_y - gyro_offset_y_;
  float gyro_z = data.gyro_z - gyro_offset_z_;

  // Apply dead zone filter to reduce noise near zero
  msg_.angular_velocity.x =
      (gyro_x > -kGyroDeadZone && gyro_x < kGyroDeadZone) ? 0.0f : gyro_x;
  msg_.angular_velocity.y =
      (gyro_y > -kGyroDeadZone && gyro_y < kGyroDeadZone) ? 0.0f : gyro_y;
  msg_.angular_velocity.z =
      (gyro_z > -kGyroDeadZone && gyro_z < kGyroDeadZone) ? 0.0f : gyro_z;

  // Linear acceleration (BNO085 already outputs in m/s^2...)
  msg_.linear_acceleration.x = data.accel_x;
  msg_.linear_acceleration.y = data.accel_y;
  msg_.linear_acceleration.z = data.accel_z;

#ifdef USE_TEENSYTHREADS
  {
    Threads::Scope guard(g_microros_mutex);
    (void)rcl_publish(&pub_, &msg_, NULL);
  }
#else
  (void)rcl_publish(&pub_, &msg_, NULL);
#endif
}

}  // namespace Subsystem
