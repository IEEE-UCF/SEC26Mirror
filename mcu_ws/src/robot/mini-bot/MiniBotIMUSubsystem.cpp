#include "MiniBotIMUSubsystem.h"

namespace Subsystem {

// change these later if needed
static char frame_name[] = "imu_link";

void MiniBotIMUSubsystem::begin() {
  if (!mpu_.init()) return;
  // probably calibrate MPU here as well
}

void MiniBotIMUSubsystem::update() { mpu_.update(); }

void MiniBotIMUSubsystem::pause() {}

void MiniBotIMUSubsystem::reset() { pause(); }

const char* MiniBotIMUSubsystem::getInfo() {
  const char info[] = "MiniBotIMUSubsystem";
  return info;
}

bool MiniBotIMUSubsystem::onCreate(rcl_node_t* node,
                                   rclc_executor_t* executor) {
  node_ = node;
  executor_ = executor;

  sensor_msgs__msg__Imu_init(&mpu_msg_);

  if (RCL_RET_OK != rclc_publisher_init_best_effort(
                        &mpu_pub_, node_,
                        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
                        "/minibot/imu/data")) {
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

  // quaternion calculation from roll pitch yaw
  // maybe also angular velocity?

  mpu_msg_.orientation.x;
  mpu_msg_.orientation.y;
  mpu_msg_.orientation.z;
  mpu_msg_.orientation.w;

  mpu_msgs_.angular_velocity.x;
  mpu_msgs_.angular_velocity.y;
  mpu_msgs_.angular_velocity.z;

  mpu_msgs_.linear_acceleration.x;
  mpu_msgs_.linear_acceleration.y;
  mpu_msgs_.linear_acceleration.z;

  rclc_publish(&mpu_pub_, &mpu_msg_, NULL);
}

}  // namespace Subsystem