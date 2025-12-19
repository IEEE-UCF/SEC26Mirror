#include "DriveSubsystem.h"

namespace Subsystem {

static char frame_name[] = "odom";
static char child_frame[] = "base_link";

void DriveSubsystem::update() {
  if (everyMs(100)) driveBase_.update();
  if (everyMs(1000)) publishData();
}

void DriveSubsystem::begin() { update(); }

void DriveSubsystem::pause() {}

void DriveSubsystem::reset() { pause(); }

const char* DriveSubsystem::getInfo() {
  const char info[] = "DriveSubsystem";
  return info;
}

bool DriveSubsystem::onCreate(rcl_node_t* node, rclc_executor* executor) {
  node_ = node;
  executor_ = executor;

  rcl_allocator_t allocator = rcl_get_default_allocator();

  mcu_msgs__msg__DriveBase__init(&drive_msg_);

  if (RCL_RET_OK != rclc_publisher_init_best_effort(
                        &drive_pub_, node_,
                        ROSIDL_GET_MSG_TYPE_SUPPORT(mcu_msgs, msg, DriveBase),
                        "drive_base/status")) {
    return false;
  }

  if (RCL_RET_OK != rclc_subscription_init_best_effort(
                        &drive_sub_, node_,
                        ROSIDL_GET_MSG_TYPE_SUPPORT(mcu_msgs, msg, DriveBase),
                        "drive_base/command")) {
    return false;
  }

  if (RCL_RET_OK != rclc_executor_add_subscription_with_context(
                        executor_, &drive_sub_, &drive_msg_,
                        &DriveSubsystem::drive_callback, this, ON_NEW_DATA)) {
    return false;
  }
  return true;
}

bool DriveSubsystem::onDestroy() {
  rcl_publisher_fini(&drive_pub_, node_);
  rcl_subscription_fini(&drive_sub_, node_);
  mcu_msgs__msg__DriveBase__fini(&drive_msg_, node_);

  node_ = nullptr;

  return true;
}

void DriveSubsystem::publishData() {
  Pose2D currentPose = driveBase_.getCurrentPose();
  Vector2D currentVelocity = driveBase_.getCurrentVelocity();
  int64_t time_ns = rmw_uros_epoch_nanos();

  drive_msg_.header.frame_id.data = frame_name;
  drive_msg_.header.frame_id.size = strlen(frame_name);
  drive_msg_.header.frame_id.capacity = strlen(frame_name) + 1;

  drive_msg_.header.stamp.sec = (int32_t)(time_ns / 1000000000);
  drive_msg_.header.stamp.nanosec = (int32_t)(time_ns % 1000000000);

  drive_msg_.transform.child_frame_id.data = child_frame;
  drive_msg_.transform.child_frame_id.size = strlen(child_frame);
  drive_msg_.transform.child_frame_id.capacity = strlen(child_frame) + 1;
  drive_msg_.transform.transform.translation.x = currentPose.getX();
  drive_msg_.transform.transform.translation.y = currentPose.getY();
  drive_msg_.transform.transform.translation.z = 0.0f;

  float yaw = currentPose.getTheta();
  drive_msg_.transform.transform.rotation.x = 0.0;
  drive_msg_.transform.transform.rotation.y = 0.0;
  drive_msg_.transform.transform.rotation.z = sin(yaw / 2.0);
  drive_msg_.transform.transform.rotation.w = cos(yaw / 2.0);

  drive_msg_.twist.linear.x = currentVelocity.getX();
  drive_msg_.twist.linear.y = currentVelocity.getY();
  drive_msg_.twist.angular.z = currentVelocity.getTheta();

  drive_msg_.transform.header.stamp = drive_msg_.header.stamp;

  rcl_publish(&drive_pub_, &drive_msg_, NULL);
}

// void* context is the pointer to the current subsystem instance
static void DriveSubsystem::drive_callback(const void* msvin, void* context) {
  DriveSubsystem* instance = (DriveSubsystem*)context;
  const mcu_msgs__msg__DriveBase* msg = (const mcu_msgs__msg__DriveBase*)msvin;

  switch (msg->drive_mode) {
    case mcu_msgs__msg__DriveBase__DRIVE_VECTOR:
      instance->driveBase_.driveVelocity(Vector2D(
          msg->goal_velocity.linear.x, 0, msg->goal_velocity.angular.z));
      break;
    case mcu_msgs__msg__DriveBase__DRIVE_GOAL:
      double qz = msg->goal_transform.transform.rotation.z;
      double qw = msg->goal_transform.transform.rotation.w;
      float targetYaw = atan2(2.0 * (qw * qz), 1.0 - 2.0 * (qz * qz));
      instance->driveBase_.driveSetPoint(
          Pose2D(msg->goal_transform.transform.translation.x,
                 msg->goal_transform.transform.translation.y, targetYaw));
      break;
    default:
      instance->driveBase_.driveVelocity(Vector2D(0, 0, 0));
      break;
  }
}
}  // namespace Subsystem
