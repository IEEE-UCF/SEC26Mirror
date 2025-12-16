#include "McuSubsystem.h"

namespace Subsystem {
bool MCUSubsystem::init() {
    // Invoke init callback if provided and map result to state
    // change state from pre_init to init
    data_.mcu_state_ = McuState::INIT; 
    publishStatus(); // send pre_init
    bool success = true;
    if (cb_.mcu_init_) {
      success = cb_.mcu_init_();
    }
    // set state
    data_.mcu_state_ = success ? McuState::INIT_SUCCESS : McuState::INIT_FAIL;
    publishStatus();
    return success;
}

void MCUSubsystem::arm() {
    bool completion = false;
    if (cb_.mcu_arm_) {
        completion = cb_.mcu_arm_();
    }
    data_.mcu_state_ = completion ? McuState::ARMED : data_.mcu_state_;
}

void MCUSubsystem::begin() {
    bool completion = false;
    if (cb_.mcu_begin_) {
      completion = cb_.mcu_begin_();
    }
    data_.mcu_state_ = completion ? McuState::RUNNING : data_.mcu_state_;
    publishStatus();
}

void MCUSubsystem::update() {
  if (cb_.mcu_update_) {
    cb_.mcu_update_();
  }
  publishStatus();
}

void MCUSubsystem::stop() {
    if(cb_.mcu_stop_) {
        cb_.mcu_stop_();
    }
    data_.mcu_state_ = McuState::STOPPED;
    publishStatus();
}

void MCUSubsystem::reset() {
  data_.mcu_state_ = McuState::RESET;
  publishStatus();
  // Goodbye world! (Assumed to restart the microcontroller)
  if (cb_.mcu_reset_) {
    cb_.mcu_reset_();
  }
}

const char* MCUSubsystem::getInfo() { return "MCUSubsystem"; }

bool MCUSubsystem::onCreate(rcl_node_t* node, rclc_executor_t* executor) {
  (void)executor;
  node_ = node;
  // Initialize publisher for mcu state message
    rcl_ret_t rc = rclc_publisher_init_best_effort(
      &pub_, node_, ROSIDL_GET_MSG_TYPE_SUPPORT(mcu_msgs, msg, McuState),
      "/mcu_robot/mcu_state");
  // Default to PRE_INIT
  msg_.state = 0; // PRE_INIT per McuState.msg
  return rc == RCL_RET_OK;
}

void MCUSubsystem::onDestroy() {
  if (pub_.impl) {
    (void)rcl_publisher_fini(&pub_, node_);
  }
  node_ = nullptr;
}

void MCUSubsystem::publishStatus() {
  if (!pub_.impl) return;
  msg_.state = uint8_t(data_.mcu_state_);
  (void)rcl_publish(&pub_, &msg_, NULL);
}

}  // namespace Subsystem