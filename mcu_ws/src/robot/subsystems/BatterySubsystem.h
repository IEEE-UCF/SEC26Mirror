/**
 * @file BatterySubsystem.h
 * @date 12/18/2025
 * @author Aldem Pido
 * @brief Battery Subsystem for the SEC2026 Robot.
 */

#ifndef BATTERYSUBSYSTEM_H
#define BATTERYSUBSYSTEM_H

#include <BaseSubsystem.h>
#include <I2CPowerDriver.h>
#include <mcu_msgs/msg/battery_health.h>
#include <microros_manager_robot.h>

#ifdef USE_FREERTOS
#include "arduino_freertos.h"
#endif

namespace Subsystem {
class BatterySubsystemSetup : public Classes::BaseSetup {
 public:
  BatterySubsystemSetup(const char* _id, Drivers::I2CPowerDriver* driver)
      : Classes::BaseSetup(_id), driver_(driver) {}
  Drivers::I2CPowerDriver* driver_ = nullptr;
};

class BatterySubsystem : public IMicroRosParticipant,
                         public Classes::BaseSubsystem {
 public:
  explicit BatterySubsystem(const BatterySubsystemSetup& setup)
      : Classes::BaseSubsystem(setup), setup_(setup) {}

  bool init() override;
  void begin() override {}
  void update() override;
  void pause() override {}
  void reset() override;
  const char* getInfo() override;

  bool onCreate(rcl_node_t* node, rclc_executor_t* executor) override;
  void onDestroy() override;

  void publishData();

#ifdef USE_FREERTOS
  void beginThreaded(uint32_t stackSize, UBaseType_t priority,
                     uint32_t updateRateMs = 100) {
    task_delay_ms_ = updateRateMs;
    xTaskCreate(taskFunction, getInfo(), stackSize, this, priority, nullptr);
  }

 private:
  static void taskFunction(void* pvParams) {
    auto* self = static_cast<BatterySubsystem*>(pvParams);
    self->begin();
    while (true) {
      self->update();
      vTaskDelay(pdMS_TO_TICKS(self->task_delay_ms_));
    }
  }
  uint32_t task_delay_ms_ = 100;
#endif

 private:
  const BatterySubsystemSetup setup_;
  rcl_publisher_t pub_{};
  mcu_msgs__msg__BatteryHealth msg{};
  rcl_node_t* node_ = nullptr;
  uint32_t last_publish_ms_ = 0;
};
}  // namespace Subsystem

#endif
