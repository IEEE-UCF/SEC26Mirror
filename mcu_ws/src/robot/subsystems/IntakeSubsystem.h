#pragma once

#include <BaseSubsystem.h>
#include <microros_manager_robot.h>

#ifdef USE_FREERTOS
#include "arduino_freertos.h"
#endif

#include "TimedSubsystem.h"
#include "mcu_msgs/msg/intake_state.h"

namespace Subsystem {

/**
 * @brief Intake states: the state machine runs entirely on the Teensy, so no
 * ROS dependencies!!
 */
enum class IntakeState : uint8_t {
  IDLE = 0,      // Motor off, waiting for command
  SPINNING = 1,  // Motor forward, actively intaking
  CAPTURED = 2,  // Duck detected, motor off
  JAMMED = 3,    // Timeout expired without capture
  EJECTING = 4,  // Motor reversing to clear/eject
  FAULT = 5      // Unrecoverable error
};

/**
 * @brief Commands that can be issued to the intake
 */
enum class IntakeCommand : uint8_t { NONE = 0, START = 1, STOP = 2, EJECT = 3 };

/**
 * @brief Setup configuration for IntakeSubsystem
 */
class IntakeSubsystemSetup : public Classes::BaseSetup {
 public:
  IntakeSubsystemSetup(const char* _id, uint8_t motor_pwm_pin,
                       uint8_t motor_dir_pin, uint8_t ir_sensor_pin,
                       uint32_t jam_timeout_ms = 3000,
                       uint32_t eject_duration_ms = 500,
                       uint8_t motor_speed = 200)
      : Classes::BaseSetup(_id),
        motor_pwm_pin_(motor_pwm_pin),
        motor_dir_pin_(motor_dir_pin),
        ir_sensor_pin_(ir_sensor_pin),
        jam_timeout_ms_(jam_timeout_ms),
        eject_duration_ms_(eject_duration_ms),
        motor_speed_(motor_speed) {}

  uint8_t motor_pwm_pin_;
  uint8_t motor_dir_pin_;
  uint8_t ir_sensor_pin_;
  uint32_t jam_timeout_ms_;     // Time before declaring jam
  uint32_t eject_duration_ms_;  // How long to run motor in reverse
  uint8_t motor_speed_;         // 0-255 motor speed
};

/**
 * @brief Intake Subsystem - owns motor and IR breakbeam sensor
 *
 * This subsystem runs a fully autonomous state machine on the Teensy
 * ROS is only used for optional command input and state publishing
 * The intake will detect ducks, handle jams, and protect itself
 * whether or not ROS is connected!
 */
class IntakeSubsystem : public IMicroRosParticipant,
                        public Subsystem::TimedSubsystem {
 public:
  explicit IntakeSubsystem(const IntakeSubsystemSetup& setup)
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
  // These can be called from ROS callbacks or directly from other code
  void startIntake();
  void stopIntake();
  void eject();

  // State Queries
  IntakeState getState() const { return state_; }
  bool isDuckDetected() const { return duck_detected_; }
  bool isIntakeActive() const { return state_ == IntakeState::SPINNING; }
  bool hasDuck() const { return state_ == IntakeState::CAPTURED; }
  bool isJammed() const { return state_ == IntakeState::JAMMED; }

#ifdef USE_FREERTOS
  void beginThreaded(uint32_t stackSize, UBaseType_t priority,
                     uint32_t updateRateMs = 20) {
    task_delay_ms_ = updateRateMs;
    xTaskCreate(taskFunction, getInfo(), stackSize, this, priority, nullptr);
  }

 private:
  static void taskFunction(void* pvParams) {
    auto* self = static_cast<IntakeSubsystem*>(pvParams);
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
  void transitionTo(IntakeState new_state);

  // Motor Control
  void setMotorForward();
  void setMotorReverse();
  void setMotorOff();
  void applyMotorState();

  // Sensor Reading
  void readSensor();

  // ROS Publishing
  void publishState();

  // Configuration
  const IntakeSubsystemSetup setup_;

  // State Machine
  IntakeState state_ = IntakeState::IDLE;
  IntakeCommand pending_command_ = IntakeCommand::NONE;

  // Timing
  uint32_t state_entry_time_ms_ = 0;  // When we entered current state

  // Sensor State
  bool duck_detected_ = false;
  bool duck_detected_prev_ = false;

  // Motor State
  enum class MotorState : uint8_t { OFF = 0, FORWARD = 1, REVERSE = 2 };
  MotorState motor_state_ = MotorState::OFF;

  // ROS Entities
  rcl_publisher_t state_pub_{};
  rcl_subscription_t cmd_sub_{};
  mcu_msgs__msg__IntakeState state_msg_{};
  rcl_node_t* node_ = nullptr;

  // Debug/Diagnostics slop
  uint32_t capture_count_ = 0;
  uint32_t jam_count_ = 0;
};

}  // namespace Subsystem
