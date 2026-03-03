/**
 * @file ResetSubsystem.h
 * @brief Exposes a micro-ROS service to reset all robot subsystems.
 * @date 2026-02-27
 *
 * Calls reset() on every registered subsystem when the service is invoked,
 * bringing motors, servos, encoders, LEDs, and arm back to their default
 * safe state.
 *
 * -- ROS2 interface (defaults) -----------------------------------------------
 *   /mcu_robot/reset   service   mcu_msgs/srv/Reset
 */

#pragma once

#include <BaseSubsystem.h>
#include <mcu_msgs/srv/reset.h>
#include <microros_manager_robot.h>

namespace Subsystem {

class ResetSubsystemSetup : public Classes::BaseSetup {
 public:
  static constexpr uint8_t MAX_TARGETS = 16;

  /**
   * @param _id          Subsystem identifier.
   * @param serviceName  ROS2 service name.
   */
  ResetSubsystemSetup(const char* _id,
                       const char* serviceName = "/mcu_robot/reset")
      : Classes::BaseSetup(_id), serviceName_(serviceName) {}

  const char* serviceName_ = "/mcu_robot/reset";
};

class ResetSubsystem : public IMicroRosParticipant,
                        public Classes::BaseSubsystem {
 public:
  static constexpr uint8_t MAX_TARGETS = ResetSubsystemSetup::MAX_TARGETS;

  explicit ResetSubsystem(const ResetSubsystemSetup& setup)
      : Classes::BaseSubsystem(setup), setup_(setup) {}

  /**
   * @brief Register a subsystem to be reset when the service is called.
   * @param target Pointer to a BaseSubsystem to reset.
   * @return true if successfully added, false if full.
   */
  bool addTarget(Classes::BaseSubsystem* target) {
    if (num_targets_ >= MAX_TARGETS || !target) return false;
    targets_[num_targets_++] = target;
    return true;
  }

  bool init() override { return true; }
  void begin() override {}
  void update() override {}
  void pause() override {}
  void reset() override {}

  const char* getInfo() override {
    static const char info[] = "ResetSubsystem";
    return info;
  }

  // ── IMicroRosParticipant ──────────────────────────────────────────────
  bool onCreate(rcl_node_t* node, rclc_executor_t* executor) override {
    node_ = node;

    if (rclc_service_init_default(
            &srv_, node_, ROSIDL_GET_SRV_TYPE_SUPPORT(mcu_msgs, srv, Reset),
            setup_.serviceName_) != RCL_RET_OK) {
      return false;
    }

    if (rclc_executor_add_service_with_context(
            executor, &srv_, &srv_req_, &srv_res_,
            &ResetSubsystem::srvCallback, this) != RCL_RET_OK) {
      return false;
    }

    return true;
  }

  void onDestroy() override {
    srv_ = rcl_get_zero_initialized_service();
    node_ = nullptr;
  }

 private:
  static void srvCallback(const void* req, void* res, void* ctx) {
    (void)req;
    auto* self = static_cast<ResetSubsystem*>(ctx);
    auto* rsp = static_cast<mcu_msgs__srv__Reset_Response*>(res);

    for (uint8_t i = 0; i < self->num_targets_; i++) {
      self->targets_[i]->reset();
    }

    rsp->success = true;
  }

  const ResetSubsystemSetup setup_;
  Classes::BaseSubsystem* targets_[MAX_TARGETS] = {};
  uint8_t num_targets_ = 0;

  rcl_service_t srv_{};
  mcu_msgs__srv__Reset_Request srv_req_{};
  mcu_msgs__srv__Reset_Response srv_res_{};
  rcl_node_t* node_ = nullptr;
};

}  // namespace Subsystem
