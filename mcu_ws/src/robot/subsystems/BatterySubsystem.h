/**
 * @file BatterySubsystem.h
 * @date 12/18/2025 (updated 2026-02-23)
 * @author Aldem Pido
 * @brief INA219 battery monitor subsystem.
 *
 * Reads voltage, current, and power from an INA219 (optionally via a TCA9548A
 * I2C mux) and publishes to micro-ROS at 1 Hz.
 *
 * ── ROS2 interface ───────────────────────────────────────────────────────────
 *   /mcu_robot/battery_health   topic   mcu_msgs/msg/BatteryHealth
 *     voltage      — load voltage (V)
 *     shunt_voltage — shunt drop (V)
 *     current      — current (A)
 *     power        — power (W)
 *     temperature  — 0.0 (INA219 has no die-temp sensor)
 *     energy       — 0.0 (no accumulator)
 *     charge_use   — 0.0 (no accumulator)
 */

#ifndef BATTERYSUBSYSTEM_H
#define BATTERYSUBSYSTEM_H

#include <BaseSubsystem.h>
#include <I2CPowerDriver.h>
#include <mcu_msgs/msg/battery_health.h>
#include <microros_manager_robot.h>

#ifdef USE_TEENSYTHREADS
#include <TeensyThreads.h>
#endif

namespace Subsystem {

class OLEDSubsystem;  // forward declaration

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

  // ── BaseSubsystem lifecycle ───────────────────────────────────────────────
  bool init() override;
  void begin() override {}
  void update() override;
  void pause() override {}
  void reset() override;
  const char* getInfo() override;

  // ── IMicroRosParticipant ──────────────────────────────────────────────────
  bool onCreate(rcl_node_t* node, rclc_executor_t* executor) override;
  void onDestroy() override;

  void publishData();

  /** Set an OLED subsystem to receive a persistent battery status line. */
  void setOLED(OLEDSubsystem* oled) { oled_ = oled; }

  /** Get load voltage in volts. */
  float getVoltage() const { return setup_.driver_ ? setup_.driver_->getVoltage() : 0.0f; }

  /** Get current draw in amps. */
  float getCurrentA() const { return setup_.driver_ ? setup_.driver_->getCurrentmA() / 1000.0f : 0.0f; }

#ifdef USE_TEENSYTHREADS
  void beginThreaded(uint32_t stackSize, int /*priority*/ = 1,
                     uint32_t updateRateMs = 100) {
    task_delay_ms_ = updateRateMs;
    threads.addThread(taskFunction, this, stackSize);
  }
#endif

 private:
  const BatterySubsystemSetup setup_;
  OLEDSubsystem* oled_ = nullptr;
  rcl_publisher_t pub_{};
  mcu_msgs__msg__BatteryHealth msg_{};
  rcl_node_t* node_ = nullptr;
  uint32_t last_publish_ms_ = 0;

#ifdef USE_TEENSYTHREADS
  static void taskFunction(void* pvParams) {
    auto* self = static_cast<BatterySubsystem*>(pvParams);
    self->begin();
    while (true) {
      self->update();
      threads.delay(self->task_delay_ms_);
    }
  }
  uint32_t task_delay_ms_ = 100;
#endif
};

}  // namespace Subsystem

#endif
