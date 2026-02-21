#pragma once

#include <BaseSubsystem.h>
#include <microros_manager_robot.h>

#ifdef USE_FREERTOS
#include "arduino_freertos.h"
#endif

#include "TimedSubsystem.h"
#include "mcu_msgs/msg/intake_bridge_command.h"
#include "mcu_msgs/msg/intake_bridge_state.h"

namespace Subsystem {

/**
 * @brief Intake bridge (gear-and-rack) states
 */
enum class IntakeBridgeState : uint8_t {
  STOWED = 0,      // Rack retracted, safe for travel
  EXTENDING = 1,   // Rack motors extending
  EXTENDED = 2,    // Rack fully extended, ready for duck capture
  RETRACTING = 3,  // Rack motors retracting
  ERROR = 4        // Timeout or sensor fault
};

/**
 * @brief Commands for the intake bridge
 */
enum class IntakeBridgeCommand : uint8_t {
  STOW = 0,
  EXTEND = 1,
  RETRACT = 2
};

/**
 * @brief Setup configuration for IntakeBridgeSubsystem
 */
class IntakeBridgeSubsystemSetup : public Classes::BaseSetup {
 public:
  IntakeBridgeSubsystemSetup(const char* _id, uint8_t rack_pwm_pin,
                              uint8_t rack_dir_pin, uint8_t tof_xshut_pin,
                              uint8_t tof_i2c_addr = 0x30,
                              uint32_t extend_timeout_ms = 3000,
                              uint32_t retract_timeout_ms = 3000,
                              uint16_t duck_detect_threshold_mm = 50,
                              uint8_t motor_speed = 200)
      : Classes::BaseSetup(_id),
        rack_pwm_pin_(rack_pwm_pin),
        rack_dir_pin_(rack_dir_pin),
        tof_xshut_pin_(tof_xshut_pin),
        tof_i2c_addr_(tof_i2c_addr),
        extend_timeout_ms_(extend_timeout_ms),
        retract_timeout_ms_(retract_timeout_ms),
        duck_detect_threshold_mm_(duck_detect_threshold_mm),
        motor_speed_(motor_speed) {}

  uint8_t rack_pwm_pin_;
  uint8_t rack_dir_pin_;
  uint8_t tof_xshut_pin_;              // XSHUT pin for VL53L0X address config
  uint8_t tof_i2c_addr_;               // I2C address for VL53L0X
  uint32_t extend_timeout_ms_;         // Max time to extend before error
  uint32_t retract_timeout_ms_;        // Max time to retract before error
  uint16_t duck_detect_threshold_mm_;  // TOF distance below which duck detected
  uint8_t motor_speed_;                // 0-255 motor speed
};

/**
 * @brief Intake Bridge Subsystem - gear-and-rack mechanism for pressure plate
 * duck retrieval
 *
 * Extends a rack over the pressure plate to push/capture the duck,
 * then retracts. Uses a VL53L0X TOF sensor to detect duck presence.
 */
class IntakeBridgeSubsystem : public IMicroRosParticipant,
                              public Subsystem::TimedSubsystem {
 public:
  explicit IntakeBridgeSubsystem(const IntakeBridgeSubsystemSetup& setup)
      : Subsystem::TimedSubsystem(setup), setup_(setup) {}

  // Lifecycle Hooks (BaseSubsystem interface)
  bool init() override;
  void begin() override;
  void update() override;
  void pause() override;
  void reset() override;
  const char* getInfo() override;

  // MicroROS Hooks (IMicroRosParticipant interface)
  bool onCreate(rcl_node_t* node, rclc_executor_t* executor) override;
  void onDestroy() override;

  // Public Commands
  void extend();
  void retract();
  void stow();

  // State Queries
  IntakeBridgeState getState() const { return state_; }
  bool isDuckDetected() const { return duck_detected_; }
  uint16_t getTofDistance() const { return tof_distance_mm_; }

#ifdef USE_FREERTOS
  void beginThreaded(uint32_t stackSize, UBaseType_t priority,
                     uint32_t updateRateMs = 20) {
    task_delay_ms_ = updateRateMs;
    xTaskCreate(taskFunction, getInfo(), stackSize, this, priority, nullptr);
  }

 private:
  static void taskFunction(void* pvParams) {
    auto* self = static_cast<IntakeBridgeSubsystem*>(pvParams);
    self->begin();
    while (true) {
      self->update();
      vTaskDelay(pdMS_TO_TICKS(self->task_delay_ms_));
    }
  }
  uint32_t task_delay_ms_ = 20;
#endif

 private:
  // Internal State Machine Logic
  void updateStateMachine();
  void transitionTo(IntakeBridgeState new_state);

  // Motor Control
  void setMotorExtend();
  void setMotorRetract();
  void setMotorOff();
  void applyMotorState();

  // Sensor Reading
  void readTofSensor();

  // ROS Publishing
  void publishState();

  // ROS Command callback
  static void commandCallback(const void* msgin);

  // Configuration
  const IntakeBridgeSubsystemSetup setup_;

  // State Machine
  IntakeBridgeState state_ = IntakeBridgeState::STOWED;
  IntakeBridgeCommand pending_command_ = IntakeBridgeCommand::STOW;
  bool has_pending_command_ = false;

  // Timing
  uint32_t state_entry_time_ms_ = 0;

  // Sensor State
  bool duck_detected_ = false;
  uint16_t tof_distance_mm_ = 0;

  // Motor State
  enum class MotorState : uint8_t { OFF = 0, EXTENDING = 1, RETRACTING = 2 };
  MotorState motor_state_ = MotorState::OFF;

  // ROS Entities
  rcl_publisher_t state_pub_{};
  rcl_subscription_t cmd_sub_{};
  mcu_msgs__msg__IntakeBridgeState state_msg_{};
  mcu_msgs__msg__IntakeBridgeCommand cmd_msg_{};
  rcl_node_t* node_ = nullptr;

  // Diagnostics
  uint32_t extend_count_ = 0;
  uint32_t retract_count_ = 0;
};

}  // namespace Subsystem
