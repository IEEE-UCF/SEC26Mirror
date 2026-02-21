#include "MiniBotSubsystem.h"

namespace Subsystem {

const char[] imu_frame = "imu_link";

void MiniBotSubsystem::begin() {}

void MiniBotSubsystem::update() {}

void MiniBotSubsystem::reset() {}

void MiniBotSubsystem::pause() {}

bool MiniBotSubsystem::onCreate(rcl_node_t* node, rclc_executor_t* executor) {
  node_ = node;
  executor_ = executor;

  sensor_msgs__msg__Imu_init(&mpu_msg_);
  mcu_msgs__msg__UWBRanging_init(&uwb_msg_);

  if (RCL_RET_OK != rclc_publisher_init_best_effort(
                        &mpu_pub_, node_,
                        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
                        "/minibot/imu/data")) {
    return false;
  }

  if (RCL_RET_OK != rclc_publisher_init_best_effort(
                        &uwb_pub_, node_,
                        ROSIDL_GET_MSG_TYPE_SUPPORT(mcu_msgs, msg, UWBRanging),
                        "/minibot/uwb/data")) {
    return false;
  }

  if (RCL_RET_OK !=
      rclc_publisher_init_best_effort(
          &batt_pub_, node_,
          ROSIDL_GET_MSG_TYPE_SUPPORT(mcu_msgs, msg, BatteryHealth),
          "/minibot/battery_health")) {
    return false;
  }

  /* if (RCL_RET_OK != rclc_executor_add_subscription_with_context(
                        executor_, &drive_sub_, &drive_msg_,
                        &MiniBotSubsystem::drive_callback, this, ON_NEW_DATA)) {
    return false;
  } */

  if (RCL_RET_OK != rclc_executor_add_subscription_with_context(
                        executor_, &led_sub_, &led_msg_,
                        &MiniBotSubsystem::led_callback, this, ON_NEW_DATA)) {
    return false;
  }

  return true;
}

void MiniBotSubsystem::imu_pub() {
  mpu_data_ = mpu_.getData();

  int64_t time_ns = rmw_uros_epoch_nanos();
  mpu_msg_.header.frame_id.data = imu_framee;
  mpu_msg_.header.frame_id.size = strlen(imu_frame);
  mpu_msg_.header.frame_id.capacity = strlen(imu_frame) + 1;

  mpu_msg_.header.stamp.sec = (int32_t)(time_ns / 1000000000);
  mpu_msg_.header.stamp.nanosec = (int32_t)(time_ns % 1000000000);

  // quaternion calculation from roll pitch yaw?
  mpu_msg_.orientation.x;
  mpu_msg_.orientation.y;
  mpu_msg_.orientation.z;
  mpu_msg_.orientation.w;

  mpu_msg_.angular_velocity.x = data_.g_x - g_x_off_;
  mpu_msg_.angular_velocity.y = data_.g_y - g_y_off_;
  mpu_msg_.angular_velocity.z = data_.g_z - g_z_off_;

  mpu_msg_.orientation_covariance[0] = 0.001f;
  mpu_msg_.orientation_covariance[4] = 0.001f;
  mpu_msg_.orientation_covariance[8] = 0.001f;

  mpu_msg_.linear_acceleration.x = data_.a_x - a_x_off_;
  mpu_msg_.linear_acceleration.y = data_.a_y - a_y_off_;
  mpu_msg_.linear_acceleration.z = data_.a_z - a_z_off_;

  mpu_msg_.linear_acceleration_covariance[0] = 0.001f;
  mpu_msg_.linear_acceleration_covariance[4] = 0.001f;
  mpu_msg_.linear_acceleration_covariance[8] = 0.001f;

  rclc_publish(&imu_pub_, &mpu_msg_, NULL);
}

void MiniBotSubsystem::drive_callback(const void* msgin, void* context) {}

void MiniBotSubsystem::led_callback(const void* msgin, void* context) {}

void MiniBotSubsystem::calibrate_imu() {
  float sum_a_x = 0.0f, sum_a_y = 0.0f, sum_a_z = 0.0f;
  float sum_g_x = 0.0f, sum_g_y = 0.0f, sum_g_z = 0.0f;

  for (int i = 0; i < CALIBRATE_SAMPLES; i++) {
    mpu_.update();
    Drivers::MPU6050DriverData data = mpu_.getData();
    sum_a_x += data.a_x;
    sum_a_y += data.a_y;
    sum_a_z += data.a_z;

    sum_g_x += data.g_x;
    sum_g_y += data.g_y;
    sum_g_z += data.g_z;

    delay(50);
  }

  a_x_off_ = sum_a_x / CALIBRATE_SAMPLES;
  a_y_off_ = sum_a_y / CALIBRATE_SAMPLES;
  a_z_off_ = sum_a_z / CALIBRATE_SAMPLES;

  g_x_off_ = sum_g_x / CALIBRATE_SAMPLES;
  g_y_off_ = sum_g_y / CALIBRATE_SAMPLES;
  g_z_off_ = sum_g_z / CALIBRATE_SAMPLES;

  calibrated_ = true;
}

}  // namespace Subsystem