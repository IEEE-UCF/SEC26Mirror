/**
 * @file BatterySubsystem.h
 * @date 12/18/2025
 * @author Aldem Pido
 * @brief Battery Subsystem for the SEC2026 Robot.
 */

#ifndef BATTERYSUBSYSTEM_H
#define BATTERYSUBSYSTEM_H

#include <BaseSubsystem.h>
#include <mcu_msgs/msg/battery_health.h>
#include <microros_manager_robot.h>
#include "TimedSubsystem.h"
#include <I2CPowerDriver.h>

namespace Subsystem {
class BatterySubsystemSetup : public Classes::BaseSetup {
 public:
  BatterySubsystemSetup(const char* _id, Drivers::I2CPowerDriver* driver) : Classes::BaseSetup(_id), driver_(driver) {}
  Drivers::I2CPowerDriver* driver_ = nullptr;
};

class BatterySubsystem : public IMicroRosParticipant,
                         public Subsystem::TimedSubsystem {
 public:
      explicit BatterySubsystem(const BatterySubsystemSetup& setup)
        : Subsystem::TimedSubsystem(setup), setup_(setup) {}

  bool init() override;
  void begin() override {}
  void update() override;
  void pause() override {}
  void reset() override {}
  const char* getInfo() override;

  bool onCreate(rcl_node_t* node, rclc_executor_t* executor) override;
  void onDestroy() override;

  void publishData();

 private:
  const BatterySubsystemSetup setup_;
  rcl_publisher_t pub_{};
  mcu_msgs__msg__BatteryHealth msg{};
  rcl_node_t* node_ = nullptr;
};
}  // namespace Subsystem

#endif