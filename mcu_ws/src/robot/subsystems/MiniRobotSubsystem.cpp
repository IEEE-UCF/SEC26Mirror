#include "MiniRobotSubsystem.h"

#include <Arduino.h>
#include <Wire.h>

#include <cmath>

namespace Subsystem {

// Lifecycle Hooks

bool MiniRobotSubsystem::init() {
  // Initialize state
  mission_state_ = MiniRobotMissionState::IDLE;
  state_entry_time_ms_ = millis();
  last_comms_time_ms_ = 0;
  last_position_update_ms_ = 0;

  // Initialize positions to origin
  current_position_ = {0.0f, 0.0f};
  target_position_ = {0.0f, 0.0f};
  home_position_ = {0.0f, 0.0f};

  // Comms starts disconnected until we hear from ESP32
  comms_status_ = CommsStatus::DISCONNECTED;

  return true;
}

void MiniRobotSubsystem::begin() {
  // Record home position as current position at startup
  home_position_ = current_position_;
  transitionTo(MiniRobotMissionState::IDLE);
}

void MiniRobotSubsystem::update() {
  // These run ALWAYS regardless of ROS
  updateCommsStatus();
  updateStateMachine();

  // Publish state to ROS (only if connected, non-blocking)
  if (everyMs(100)) {
    publishState();
  }
}

void MiniRobotSubsystem::pause() {
  sendStopCommand();
  transitionTo(MiniRobotMissionState::IDLE);
}

void MiniRobotSubsystem::reset() {
  sendStopCommand();
  transitionTo(MiniRobotMissionState::IDLE);
  pending_command_ = MiniRobotCommand::NONE;
  mission_count_ = 0;
  error_count_ = 0;
  comms_status_ = CommsStatus::DISCONNECTED;
}

const char* MiniRobotSubsystem::getInfo() {
  static const char info[] = "MiniRobotSubsystem";
  return info;
}

// Micro-ROS Hooks

bool MiniRobotSubsystem::onCreate(rcl_node_t* node, rclc_executor_t* executor) {
  (void)executor;
  node_ = node;

  mcu_msgs__msg__MiniRobotState__init(&state_msg_);

  if (rclc_publisher_init_best_effort(
          &state_pub_, node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(mcu_msgs, msg, MiniRobotState),
          "/mcu_robot/mini_robot/state") != RCL_RET_OK) {
    return false;
  }

  return true;
}

void MiniRobotSubsystem::onDestroy() {
  // destroy_entities() finalises the rcl_node before calling onDestroy, so
  // rcl_*_fini would leave impl non-NULL on error; reset local state only.
  state_pub_ = rcl_get_zero_initialized_publisher();
  mcu_msgs__msg__MiniRobotState__fini(&state_msg_);
  node_ = nullptr;
}

// Public Commands

void MiniRobotSubsystem::startMission(float target_x, float target_y) {
  pending_target_.x = target_x;
  pending_target_.y = target_y;
  pending_command_ = MiniRobotCommand::START_MISSION;
}

void MiniRobotSubsystem::startMission(const Pose2D& target) {
  startMission(target.x, target.y);
}

void MiniRobotSubsystem::stop() { pending_command_ = MiniRobotCommand::STOP; }

void MiniRobotSubsystem::returnHome() {
  pending_command_ = MiniRobotCommand::RETURN_HOME;
}

// External Data Input

void MiniRobotSubsystem::updatePosition(float x, float y) {
  current_position_.x = x;
  current_position_.y = y;
  last_position_update_ms_ = millis();
}

void MiniRobotSubsystem::updatePosition(const Pose2D& pos) {
  updatePosition(pos.x, pos.y);
}

// Position Queries

float MiniRobotSubsystem::getDistanceToTarget() const {
  return calculateDistance(current_position_, target_position_);
}

// State Machine Logic

void MiniRobotSubsystem::updateStateMachine() {
  uint32_t now = millis();
  uint32_t time_in_state = now - state_entry_time_ms_;

  // Process pending commands first
  if (pending_command_ != MiniRobotCommand::NONE) {
    switch (pending_command_) {
      case MiniRobotCommand::START_MISSION:
        if (mission_state_ == MiniRobotMissionState::IDLE ||
            mission_state_ == MiniRobotMissionState::AT_TARGET) {
          target_position_ = pending_target_;
          sendDriveCommand(target_position_.x, target_position_.y);
          mission_count_++;
          transitionTo(MiniRobotMissionState::DRIVING_TO_TARGET);
        }
        break;

      case MiniRobotCommand::STOP:
        sendStopCommand();
        transitionTo(MiniRobotMissionState::IDLE);
        break;

      case MiniRobotCommand::RETURN_HOME:
        if (mission_state_ != MiniRobotMissionState::ERROR) {
          target_position_ = home_position_;
          sendReturnCommand();
          transitionTo(MiniRobotMissionState::RETURNING);
        }
        break;

      default:
        break;
    }
    pending_command_ = MiniRobotCommand::NONE;
  }

  // State-specific logic
  switch (mission_state_) {
    case MiniRobotMissionState::IDLE:
      break;

    case MiniRobotMissionState::DRIVING_TO_TARGET:
      if (hasArrivedAtTarget()) {
        sendStopCommand();
        transitionTo(MiniRobotMissionState::AT_TARGET);
      } else if (time_in_state >= setup_.mission_timeout_ms_) {
        sendStopCommand();
        error_count_++;
        transitionTo(MiniRobotMissionState::ERROR);
      } else if (comms_status_ == CommsStatus::TIMEOUT ||
                 comms_status_ == CommsStatus::ERROR) {
        sendStopCommand();
        error_count_++;
        transitionTo(MiniRobotMissionState::ERROR);
      }
      break;

    case MiniRobotMissionState::AT_TARGET:
      // Mission complete, wait for next command
      break;

    case MiniRobotMissionState::RETURNING:
      if (hasArrivedAtHome()) {
        sendStopCommand();
        transitionTo(MiniRobotMissionState::IDLE);
      } else if (time_in_state >= setup_.mission_timeout_ms_) {
        sendStopCommand();
        error_count_++;
        transitionTo(MiniRobotMissionState::ERROR);
      } else if (comms_status_ == CommsStatus::TIMEOUT ||
                 comms_status_ == CommsStatus::ERROR) {
        sendStopCommand();
        error_count_++;
        transitionTo(MiniRobotMissionState::ERROR);
      }
      break;

    case MiniRobotMissionState::ERROR:
      // Require explicit stop() to return to IDLE
      break;
  }
}

void MiniRobotSubsystem::transitionTo(MiniRobotMissionState new_state) {
  if (mission_state_ != new_state) {
    mission_state_ = new_state;
    state_entry_time_ms_ = millis();
  }
}

// Communication with ESP32

void MiniRobotSubsystem::sendDriveCommand(float target_x, float target_y) {
  // Pack: [CMD_DRIVE, target_x (4 bytes), target_y (4 bytes)]
  uint8_t buffer[9];
  buffer[0] = 0x01;  // CMD_DRIVE
  memcpy(&buffer[1], &target_x, sizeof(float));
  memcpy(&buffer[5], &target_y, sizeof(float));

  Wire.beginTransmission(setup_.esp32_i2c_addr_);
  Wire.write(buffer, sizeof(buffer));
  Wire.endTransmission();

  esp32_acknowledged_ = false;
}

void MiniRobotSubsystem::sendStopCommand() {
  uint8_t cmd = 0x02;  // CMD_STOP
  Wire.beginTransmission(setup_.esp32_i2c_addr_);
  Wire.write(cmd);
  Wire.endTransmission();
}

void MiniRobotSubsystem::sendReturnCommand() {
  uint8_t buffer[9];
  buffer[0] = 0x03;  // CMD_RETURN
  memcpy(&buffer[1], &home_position_.x, sizeof(float));
  memcpy(&buffer[5], &home_position_.y, sizeof(float));

  Wire.beginTransmission(setup_.esp32_i2c_addr_);
  Wire.write(buffer, sizeof(buffer));
  Wire.endTransmission();

  esp32_acknowledged_ = false;
}

void MiniRobotSubsystem::updateCommsStatus() {
  uint32_t now = millis();

  if (pollEsp32Status()) {
    last_comms_time_ms_ = now;
    comms_status_ = CommsStatus::CONNECTED;
  }

  if (last_comms_time_ms_ > 0 &&
      (now - last_comms_time_ms_) >= setup_.comms_timeout_ms_) {
    comms_status_ = CommsStatus::TIMEOUT;
  }
}

bool MiniRobotSubsystem::pollEsp32Status() {
  uint8_t bytes_received = Wire.requestFrom(setup_.esp32_i2c_addr_, (uint8_t)1);

  if (bytes_received > 0 && Wire.available()) {
    uint8_t status = Wire.read();
    esp32_acknowledged_ = (status != 0xFF);
    return true;
  }

  return false;
}

// Distance Calculations

float MiniRobotSubsystem::calculateDistance(const Pose2D& a,
                                            const Pose2D& b) const {
  float dx = b.x - a.x;
  float dy = b.y - a.y;
  return sqrtf(dx * dx + dy * dy);
}

bool MiniRobotSubsystem::hasArrivedAtTarget() const {
  return calculateDistance(current_position_, target_position_) <=
         setup_.arrival_threshold_m_;
}

bool MiniRobotSubsystem::hasArrivedAtHome() const {
  return calculateDistance(current_position_, home_position_) <=
         setup_.arrival_threshold_m_;
}

// ROS Publishing

void MiniRobotSubsystem::publishState() {
  if (!state_pub_.impl) return;

  state_msg_.header.stamp.sec = (int32_t)(millis() / 1000);
  state_msg_.header.stamp.nanosec = (uint32_t)((millis() % 1000) * 1000000);

  // Map mission state to existing message constants
  switch (mission_state_) {
    case MiniRobotMissionState::IDLE:
      state_msg_.current_task = mcu_msgs__msg__MiniRobotState__TASK_NONE;
      state_msg_.state = mcu_msgs__msg__MiniRobotState__ARMED;
      break;
    case MiniRobotMissionState::DRIVING_TO_TARGET:
      state_msg_.current_task =
          mcu_msgs__msg__MiniRobotState__TASK_ENTER_CRATER;
      state_msg_.state = mcu_msgs__msg__MiniRobotState__RUNNING;
      break;
    case MiniRobotMissionState::AT_TARGET:
      state_msg_.current_task = mcu_msgs__msg__MiniRobotState__TASK_COMPLETE;
      state_msg_.state = mcu_msgs__msg__MiniRobotState__RUNNING;
      break;
    case MiniRobotMissionState::RETURNING:
      state_msg_.current_task = mcu_msgs__msg__MiniRobotState__TASK_EXIT_CRATER;
      state_msg_.state = mcu_msgs__msg__MiniRobotState__RUNNING;
      break;
    case MiniRobotMissionState::ERROR:
      state_msg_.current_task = mcu_msgs__msg__MiniRobotState__TASK_NONE;
      state_msg_.state = mcu_msgs__msg__MiniRobotState__EMERGENCY_STOP;
      break;
  }

  (void)rcl_publish(&state_pub_, &state_msg_, NULL);
}

}  // namespace Subsystem
