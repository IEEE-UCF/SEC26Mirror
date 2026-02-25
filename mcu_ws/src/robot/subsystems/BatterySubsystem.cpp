#include "BatterySubsystem.h"

namespace Subsystem {

bool BatterySubsystem::init() { return setup_.driver_->init(); }

void BatterySubsystem::update() {
  setup_.driver_->update();

  if (!pub_.impl) return;
  uint32_t now = millis();
  if (now - last_publish_ms_ >= 1000) {
    last_publish_ms_ = now;
    publishData();
  }
}

void BatterySubsystem::reset() { pause(); }

const char* BatterySubsystem::getInfo() {
  static const char info[] = "BatterySubsystem";
  return info;
}

bool BatterySubsystem::onCreate(rcl_node_t* node, rclc_executor_t* executor) {
  (void)executor;
  node_ = node;
  if (rclc_publisher_init_best_effort(
          &pub_, node_,
          ROSIDL_GET_MSG_TYPE_SUPPORT(mcu_msgs, msg, BatteryHealth),
          "/mcu_robot/battery_health") != RCL_RET_OK) {
    return false;
  }
  return true;
}

void BatterySubsystem::onDestroy() {
  // destroy_entities() finalises the rcl_node before calling onDestroy, so
  // rcl_publisher_fini would see node->impl == NULL and return an error while
  // leaving pub_.impl non-NULL.  On the next onCreate that stale impl causes
  // rclc_publisher_init_best_effort to fail (publisher already initialised).
  // The DDS resources are freed by rclc_support_fini — just reset local state.
  pub_ = rcl_get_zero_initialized_publisher();
  node_ = nullptr;
}

void BatterySubsystem::publishData() {
  if (!pub_.impl || !setup_.driver_) return;

  // INA219 data — convert driver units (mA, mV, mW) to SI (A, V, W).
  msg_.voltage = setup_.driver_->getVoltage();                         // V
  msg_.shunt_voltage = setup_.driver_->getShuntVoltagemV() / 1000.0f;  // V
  msg_.current = setup_.driver_->getCurrentmA() / 1000.0f;             // A
  msg_.power = setup_.driver_->getPowermW() / 1000.0f;                 // W
  // INA219 has no die-temp sensor or energy/charge accumulators.
  msg_.temperature = 0.0f;
  msg_.energy = 0.0f;
  msg_.charge_use = 0.0f;

#ifdef USE_TEENSYTHREADS
  { Threads::Scope guard(g_microros_mutex);
    (void)rcl_publish(&pub_, &msg_, NULL);
  }
#else
  (void)rcl_publish(&pub_, &msg_, NULL);
#endif
}

}  // namespace Subsystem
