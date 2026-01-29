/**
 * @file MinibotDriveSubsystem.h
 * @author Rafeed Khan
 * @brief Micro-ROS subsystem that receives Twist commands and drives minibot motors
 */
#pragma once

#include <Arduino.h>
#include <geometry_msgs/msg/twist.h>
#include <mcu_msgs/msg/mini_robot_state.h>
#include <microros_manager_robot.h>

#include "MotorDriver.h"
#include "TimedSubsystem.h"

namespace Subsystem {

class MinibotDriveSubsystemSetup : public Classes::BaseSetup {
 public:
  Drivers::MotorDriver* left_motor;
  Drivers::MotorDriver* right_motor;
  float speed_scale;

  MinibotDriveSubsystemSetup(const char* _id,
                              Drivers::MotorDriver* _left,
                              Drivers::MotorDriver* _right,
                              float _speed_scale = 255.0f)
      : Classes::BaseSetup(_id),
        left_motor(_left),
        right_motor(_right),
        speed_scale(_speed_scale) {}
};

class MinibotDriveSubsystem : public IMicroRosParticipant,
                               public Subsystem::TimedSubsystem {
 public:
  explicit MinibotDriveSubsystem(const MinibotDriveSubsystemSetup& setup)
      : Subsystem::TimedSubsystem(setup), setup_(setup) {}

  bool init() override {
    stopMotors();
    return true;
  }

  void begin() override {}

  void update() override {
    // Drive the motors (handles PWM output timing)
    setup_.left_motor->update();
    setup_.right_motor->update();

    // Safety timeout slop, stops if no command received in 500ms
    if (last_cmd_ms_ > 0 && (millis() - last_cmd_ms_) > 500) {
      stopMotors();
      mission_state_ = mcu_msgs__msg__MiniRobotState__ARMED;
    }

    if (everyMs(100)) {
      publishState();
    }
  }

  void pause() override { stopMotors(); }
  void reset() override { stopMotors(); }

  const char* getInfo() override {
    static const char info[] = "MinibotDrive";
    return info;
  }

  // IMicroRosParticipant
  bool onCreate(rcl_node_t* node, rclc_executor_t* executor) override {
    node_ = node;

    geometry_msgs__msg__Twist__init(&twist_msg_);
    mcu_msgs__msg__MiniRobotState__init(&state_msg_);

    if (RCL_RET_OK != rclc_publisher_init_best_effort(
                          &state_pub_, node,
                          ROSIDL_GET_MSG_TYPE_SUPPORT(mcu_msgs, msg,
                                                      MiniRobotState),
                          "/mcu_minibot/state")) {
      return false;
    }

    if (RCL_RET_OK != rclc_subscription_init_best_effort(
                          &twist_sub_, node,
                          ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
                          "/mcu_minibot/cmd_vel")) {
      return false;
    }

    if (RCL_RET_OK != rclc_executor_add_subscription_with_context(
                          executor, &twist_sub_, &twist_msg_,
                          &MinibotDriveSubsystem::twistCallback, this,
                          ON_NEW_DATA)) {
      return false;
    }

    return true;
  }

  void onDestroy() override {
    if (state_pub_.impl) rcl_publisher_fini(&state_pub_, node_);
    if (twist_sub_.impl) rcl_subscription_fini(&twist_sub_, node_);
    geometry_msgs__msg__Twist__fini(&twist_msg_);
    mcu_msgs__msg__MiniRobotState__fini(&state_msg_);
    node_ = nullptr;
  }

 private:
  void stopMotors() {
    setup_.left_motor->setPWM(0);
    setup_.right_motor->setPWM(0);
  }

  static void twistCallback(const void* msvin, void* context) {
    auto* self = (MinibotDriveSubsystem*)context;
    auto* msg = (const geometry_msgs__msg__Twist*)msvin;

    float linear = msg->linear.x;
    float angular = msg->angular.z;

    // Differential drive mix
    int left = constrain((int)((linear - angular) * self->setup_.speed_scale),
                         -255, 255);
    int right = constrain((int)((linear + angular) * self->setup_.speed_scale),
                          -255, 255);

    self->setup_.left_motor->setPWM(left);
    self->setup_.right_motor->setPWM(right);

    self->last_cmd_ms_ = millis();
    self->mission_state_ = mcu_msgs__msg__MiniRobotState__RUNNING;
  }

  void publishState() {
    if (!state_pub_.impl) return;

    state_msg_.header.stamp.sec = (int32_t)(millis() / 1000);
    state_msg_.header.stamp.nanosec =
        (uint32_t)((millis() % 1000) * 1000000);
    state_msg_.state = mission_state_;
    state_msg_.current_task = current_task_;

    rcl_publish(&state_pub_, &state_msg_, NULL);
  }

  const MinibotDriveSubsystemSetup setup_;

  // micro-ROS entities
  rcl_publisher_t state_pub_{};
  rcl_subscription_t twist_sub_{};
  geometry_msgs__msg__Twist twist_msg_{};
  mcu_msgs__msg__MiniRobotState state_msg_{};
  rcl_node_t* node_ = nullptr;

  // State
  uint8_t mission_state_ = mcu_msgs__msg__MiniRobotState__ARMED;
  uint8_t current_task_ = mcu_msgs__msg__MiniRobotState__TASK_NONE;
  uint32_t last_cmd_ms_ = 0;
};

}  // namespace Subsystem
