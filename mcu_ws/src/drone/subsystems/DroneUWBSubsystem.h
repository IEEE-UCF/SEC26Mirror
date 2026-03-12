#pragma once
/**
 * @file DroneUWBSubsystem.h
 * @brief UWB ranging + EKF update + ROS2 publisher, as an RTOSSubsystem.
 *        Runs at 20Hz via beginThreadedPinned() on core 0.
 *        Uses deferred publishing pattern (data_ready_ + publishAll()).
 */

#include <RTOSSubsystem.h>
#include <microros_manager_robot.h>
#include <mcu_msgs/msg/uwb_range.h>
#include <mcu_msgs/msg/uwb_ranging.h>
#include <UWBDriver.h>

#include "DroneEKFSubsystem.h"

namespace Drone {

class DroneUWBSubsystem : public Subsystem::RTOSSubsystem,
                           public Subsystem::IMicroRosParticipant {
 public:
  DroneUWBSubsystem(Drivers::UWBDriver& uwb, DroneEKFSubsystem& ekf)
      : RTOSSubsystem(setup_), uwb_(uwb), ekf_(ekf) {}

  // RTOSSubsystem lifecycle
  bool init() override { return true; }
  void begin() override {}
  void update() override;
  void pause() override {}
  void reset() override {}
  const char* getInfo() override { return "drone_uwb"; }

  // IMicroRosParticipant interface
  bool onCreate(rcl_node_t* node, rclc_executor_t* /*executor*/) override {
    // Initialize msg with dynamic ranges array
    mcu_msgs__msg__UWBRanging__init(&msg_);
    msg_.ranges.capacity = Drivers::UWBDriverData::MAX_ANCHORS;
    msg_.ranges.size = 0;
    msg_.ranges.data = (mcu_msgs__msg__UWBRange*)malloc(
        msg_.ranges.capacity * sizeof(mcu_msgs__msg__UWBRange));
    if (!msg_.ranges.data) return false;
    for (size_t i = 0; i < msg_.ranges.capacity; i++) {
      mcu_msgs__msg__UWBRange__init(&msg_.ranges.data[i]);
    }

    if (rclc_publisher_init_best_effort(
            &pub_, node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(mcu_msgs, msg, UWBRanging),
            "/mcu_drone/uwb/ranging") != RCL_RET_OK) {
      free(msg_.ranges.data);
      msg_.ranges.data = nullptr;
      return false;
    }

    data_mutex_ = xSemaphoreCreateMutex();
    return true;
  }

  void onDestroy() override {
    pub_.impl = nullptr;
    if (msg_.ranges.data) {
      free(msg_.ranges.data);
      msg_.ranges.data = nullptr;
    }
    mcu_msgs__msg__UWBRanging__fini(&msg_);
  }

  // Deferred publishing — called by MicrorosManager under g_microros_mutex
  void publishAll() override {
    if (!data_mutex_) return;
    xSemaphoreTake(data_mutex_, portMAX_DELAY);
    if (data_ready_ && pub_.impl) {
      (void)rcl_publish(&pub_, &msg_, NULL);
      data_ready_ = false;
    }
    xSemaphoreGive(data_mutex_);
  }

 private:
  void populateMsg();

  Classes::BaseSetup setup_{"drone_uwb"};
  Drivers::UWBDriver& uwb_;
  DroneEKFSubsystem& ekf_;
  rcl_publisher_t pub_{};
  mcu_msgs__msg__UWBRanging msg_{};
  SemaphoreHandle_t data_mutex_ = nullptr;
  bool data_ready_ = false;
};

}  // namespace Drone
