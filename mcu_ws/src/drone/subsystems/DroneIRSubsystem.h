#pragma once
/**
 * @file DroneIRSubsystem.h
 * @brief Thin ROS2 service wrapper around IRSubsystem.
 *        Creates /mcu_drone/transmit_ir service.
 */

#include <microros_manager_robot.h>
#include <mcu_msgs/srv/drone_transmit_ir.h>

#include "DroneStateSubsystem.h"
#include "IRSubsystem.h"

namespace Drone {

class DroneIRSubsystem : public Subsystem::IMicroRosParticipant {
 public:
  DroneIRSubsystem(IRSubsystem& ir, DroneStateSubsystem& state)
      : ir_(ir), state_(state) {}

  bool onCreate(rcl_node_t* node, rclc_executor_t* executor) override {
    s_instance_ = this;
    if (rclc_service_init_default(
            &srv_, node,
            ROSIDL_GET_SRV_TYPE_SUPPORT(mcu_msgs, srv, DroneTransmitIR),
            "/mcu_drone/transmit_ir") != RCL_RET_OK)
      return false;
    if (rclc_executor_add_service(executor, &srv_, &req_, &res_, callback) !=
        RCL_RET_OK)
      return false;
    return true;
  }

  void onDestroy() override {
    srv_ = rcl_get_zero_initialized_service();
  }

 private:
  static void callback(const void* req_raw, void* res_raw) {
    auto* req = (const mcu_msgs__srv__DroneTransmitIR_Request*)req_raw;
    auto* res = (mcu_msgs__srv__DroneTransmitIR_Response*)res_raw;
    auto* self = s_instance_;
    if (!self) {
      res->success = false;
      return;
    }

    // Only transmit in VELOCITY_CONTROL state
    if (self->state_.getState() != DroneState::VELOCITY_CONTROL) {
      res->success = false;
      return;
    }

    self->ir_.transmitAll(req->antenna_colors);
    res->success = true;
  }

  IRSubsystem& ir_;
  DroneStateSubsystem& state_;
  rcl_service_t srv_{};
  mcu_msgs__srv__DroneTransmitIR_Request req_{};
  mcu_msgs__srv__DroneTransmitIR_Response res_{};
  static DroneIRSubsystem* s_instance_;
};

inline DroneIRSubsystem* DroneIRSubsystem::s_instance_ = nullptr;

}  // namespace Drone
