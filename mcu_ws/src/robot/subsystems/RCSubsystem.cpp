/**
 * @file RCSubsystem.cpp
 * @brief Implementation of RCSubsystem for FlySky RC receiver.
 * @author Generated for SEC26
 * @date 12/21/2025
 */

#include "RCSubsystem.h"

namespace Subsystem {

bool RCSubsystem::init() {
  return true;
}

void RCSubsystem::begin() {
  // Initialize IBUS communication on the specified serial port
  // IBUSBM_NOTIMER means we handle timing ourselves
  ibus_.begin(*setup_.serial, IBUSBM_NOTIMER);
}

void RCSubsystem::update() {
  // Update RC data from receiver
  updateRCData();

  // Publish RC data at regular intervals
  if (!pub_.impl) return;
  if (everyMs(PUBLISH_INTERVAL_MS)) {
    publishRC();
  }
}

void RCSubsystem::pause() {
  // No special pause behavior needed
}

void RCSubsystem::reset() {
  pause();
}

bool RCSubsystem::onCreate(rcl_node_t* node, rclc_executor_t* executor) {
  (void)executor;
  node_ = node;

  // Initialize the RC message
  mcu_msgs__msg__RC__init(&msg_);

  // Create publisher for RC data
  if (rclc_publisher_init_best_effort(
          &pub_, node, ROSIDL_GET_MSG_TYPE_SUPPORT(mcu_msgs, msg, RC),
          "mcu_robot/rc") != RCL_RET_OK) {
    return false;
  }

  return true;
}

void RCSubsystem::onDestroy() {
  if (pub_.impl) {
    (void)rcl_publisher_fini(&pub_, node_);
  }
  mcu_msgs__msg__RC__fini(&msg_);
  node_ = nullptr;
}

void RCSubsystem::updateRCData() {
  if (updateTimer_ >= UPDATE_DELAY_MS) {
    updateTimer_ -= UPDATE_DELAY_MS;
    ibus_.loop();  // Process incoming IBUS data

    // Read and map all 10 channels
    int32_t newChannels[10];
    for (int i = 0; i < 10; i++) {
      // Read raw channel data (1000-2000), constrain, then map to -255 to 255
      newChannels[i] = map(constrain(ibus_.readChannel(i), 1000, 2000) - 1000, 0, 1000, -255, 255);
    }

    // Failsafe/mode check: if channel 9 is at its minimum (-255 after mapping),
    // zero out specific control channels (kill switch behavior)
    if (newChannels[9] == -255) {
      data_.channels[0] = 0;  // e.g., Roll/Aileron
      data_.channels[1] = 0;  // e.g., Pitch/Elevator
      data_.channels[2] = 0;  // e.g., Throttle
      data_.channels[3] = 0;  // e.g., Yaw/Rudder
      data_.channels[5] = 0;  // e.g., Auxiliary channel
      data_.channels[9] = -255;  // Keep failsafe switch state
    } else {
      // If not in failsafe, update all channels with new readings
      for (int i = 0; i < 10; i++) {
        data_.channels[i] = newChannels[i];
      }
    }
  }
}

void RCSubsystem::updateRCMessage() {
  // Update timestamp
  msg_.header.stamp.sec = (int32_t)(millis() / 1000);
  msg_.header.stamp.nanosec = (uint32_t)((millis() % 1000) * 1000000);

  // Copy all channel values
  for (int i = 0; i < 10; i++) {
    msg_.channels[i] = data_.channels[i];
  }

  // Extract joystick values (channels 0-3)
  msg_.lx = data_.channels[0];  // left stick X
  msg_.ly = data_.channels[1];  // left stick Y
  msg_.rx = data_.channels[2];  // right stick X
  msg_.ry = data_.channels[3];  // right stick Y

  // Extract knob values (channels 4-5)
  msg_.knobl = data_.channels[4];  // left knob
  msg_.knobr = data_.channels[5];  // right knob

  // Extract switch states (channels 6-9) by thresholding
  // Switches are typically at -255 (off) or 255 (on)
  msg_.swa = (data_.channels[6] > 0);   // switch A
  msg_.swb = (data_.channels[7] > 0);   // switch B
  msg_.swc = (data_.channels[8] > 0);   // switch C
  msg_.swd = (data_.channels[9] > 0);   // switch D
}

void RCSubsystem::publishRC() {
  if (!pub_.impl) return;

  updateRCMessage();
  (void)rcl_publish(&pub_, &msg_, NULL);
}

}  // namespace Subsystem
