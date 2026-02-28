#include "MiniBotIMUSubsystem.h"

namespace Subsystem {

// change these later if needed
static char frame_name[] = "imu_link";

void MiniBotIMUSubsystem::begin() {
  if (!mpu_.init())
    return;
  calibrate();
}

void MiniBotIMUSubsystem::update() { mpu_.update(); }

void MiniBotIMUSubsystem::pause() {}

void MiniBotIMUSubsystem::reset() { pause(); }

const char *MiniBotIMUSubsystem::getInfo() {
  const char info[] = "MiniBotIMUSubsystem";
  return info;
}

bool MiniBotIMUSubsystem::onCreate(rcl_node_t *node,
                                   rclc_executor_t *executor) {
  node_ = node;
  executor_ = executor;

  sensor_msgs__msg__Imu_init(&mpu_msg_);

  if (RCL_RET_OK != rclc_publisher_init_best_effort(
                        &mpu_pub_, node_,
                        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
                        "/mini_robot/imu/data")) {
    return false;
  }

  return true;
}
void MiniBotIMUSubsystem::onDestroy() {
  rcl_publisher_fini(&mpu_pub_, node_);
  mcu_msgs__msg__DriveBase__fini(&mpu_msg_);
  sensor_msgs__msg__Imu__fini(&mpu_msg_);
  node_ = nullptr;
}

void MiniBotIMUSubsystem::publishData() {
  data_ = mpu_.getData();

  int64_t time_ns = rmw_uros_epoch_nanos();
  mpu_msg_.header.frame_id.data = frame_name;
  mpu_msg_.header.frame_id.size = strlen(frame_name);
  mpu_msg_.header.frame_id.capacity = strlen(frame_name) + 1;

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

  rclc_publish(&mpu_pub_, &mpu_msg_, NULL);
}

void MiniBotIMUSubsystem::calibrate() {
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

} // namespace Subsystem