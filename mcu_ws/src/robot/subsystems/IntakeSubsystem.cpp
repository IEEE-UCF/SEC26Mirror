#include "IntakeSubsystem.h"

#include <Arduino.h>

namespace Subsystem {

// Lifecycle Hooks

bool IntakeSubsystem::init() {
  // Configure motor pins
  pinMode(setup_.motor_pwm_pin_, OUTPUT);
  pinMode(setup_.motor_dir_pin_, OUTPUT);

  // Configure IR sensor pin (INPUT_PULLUP for active-low breakbeam)
  pinMode(setup_.ir_sensor_pin_, INPUT_PULLUP);

  // Set PWM frequency for Teensy (this is optional but it improves motor
  // smoothness)
  analogWriteFrequency(setup_.motor_pwm_pin_, 18310.55);

  // Ensure motor is off at init
  setMotorOff();
  applyMotorState();

  // Initialize state
  state_ = IntakeState::IDLE;
  state_entry_time_ms_ = millis();

  return true;
}

void IntakeSubsystem::begin() {
  // Subsystem is now active and ready to accept commands!!!!
  transitionTo(IntakeState::IDLE);
}

void IntakeSubsystem::update() {
  // Doing these tasks ALWAYS regardless of ROS
  // Read sensor
  readSensor();

  // Run state machine
  updateStateMachine();

  // Apply motor output
  applyMotorState();

  // Publish state to ROS (only if connected, non-blocking)
  if (everyMs(50)) {  // 20 Hz publishing rate
    publishState();
  }
}

void IntakeSubsystem::pause() {
  // Safety: stop motor when paused
  setMotorOff();
  applyMotorState();
  transitionTo(IntakeState::IDLE);
}

void IntakeSubsystem::reset() {
  // Full reset: stop motor, clear state
  setMotorOff();
  applyMotorState();
  transitionTo(IntakeState::IDLE);
  pending_command_ = IntakeCommand::NONE;
  capture_count_ = 0;
  jam_count_ = 0;
}

const char* IntakeSubsystem::getInfo() {
  static const char info[] = "IntakeSubsystem";
  return info;
}

// Micro-ROS Hooks

bool IntakeSubsystem::onCreate(rcl_node_t* node, rclc_executor_t* executor) {
  (void)executor;
  node_ = node;

  // Initialize message
  mcu_msgs__msg__IntakeState__init(&state_msg_);

  // State publisher, best effort QoS
  if (rclc_publisher_init_best_effort(
          &state_pub_, node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(mcu_msgs, msg, IntakeState),
          "/mcu_robot/intake/state") != RCL_RET_OK) {
    return false;
  }

  return true;
}

void IntakeSubsystem::onDestroy() {
  // destroy_entities() finalises the rcl_node before calling onDestroy, so
  // rcl_*_fini would leave impl non-NULL on error; reset local state only.
  state_pub_ = rcl_get_zero_initialized_publisher();
  mcu_msgs__msg__IntakeState__fini(&state_msg_);
  node_ = nullptr;
}

// Public Commands

void IntakeSubsystem::startIntake() {
  // Only start if we're in a state that allows it
  if (state_ == IntakeState::IDLE || state_ == IntakeState::CAPTURED) {
    pending_command_ = IntakeCommand::START;
  }
}

void IntakeSubsystem::stopIntake() { pending_command_ = IntakeCommand::STOP; }

void IntakeSubsystem::eject() { pending_command_ = IntakeCommand::EJECT; }

// State Machine Logic

void IntakeSubsystem::updateStateMachine() {
  uint32_t now = millis();
  uint32_t time_in_state = now - state_entry_time_ms_;

  // Process pending commands first
  if (pending_command_ != IntakeCommand::NONE) {
    switch (pending_command_) {
      case IntakeCommand::START:
        if (state_ == IntakeState::IDLE || state_ == IntakeState::CAPTURED) {
          transitionTo(IntakeState::SPINNING);
        }
        break;

      case IntakeCommand::STOP:
        if (state_ != IntakeState::FAULT) {
          setMotorOff();
          transitionTo(IntakeState::IDLE);
        }
        break;

      case IntakeCommand::EJECT:
        if (state_ != IntakeState::FAULT) {
          transitionTo(IntakeState::EJECTING);
        }
        break;

      default:
        break;
    }
    pending_command_ = IntakeCommand::NONE;
  }

  // State-specific logic
  switch (state_) {
    case IntakeState::IDLE:
      // Motor should be off, just waiting
      setMotorOff();
      break;

    case IntakeState::SPINNING:
      // Motor forward, actively intaking
      setMotorForward();

      // Check for duck capture (rising edge detection)
      if (duck_detected_ && !duck_detected_prev_) {
        // Duck just entered the intake
        capture_count_++;
        transitionTo(IntakeState::CAPTURED);
      }
      // Check for jam (timeout without capture)
      else if (time_in_state >= setup_.jam_timeout_ms_) {
        jam_count_++;
        transitionTo(IntakeState::JAMMED);
      }
      break;

    case IntakeState::CAPTURED:
      // Duck captured, motor off
      setMotorOff();
      // Stay here until commanded to do something else
      break;

    case IntakeState::JAMMED:
      // Jam detected, motor off
      setMotorOff();
      // Could auto-transition to EJECTING, or wait for manual command
      // For now, wait for explicit eject or stop command
      break;

    case IntakeState::EJECTING:
      // Motor reverse to clear
      setMotorReverse();

      // Check if eject duration has elapsed
      if (time_in_state >= setup_.eject_duration_ms_) {
        transitionTo(IntakeState::IDLE);
      }
      break;

    case IntakeState::FAULT:
      // Fault state: motor off, require manual intervention
      setMotorOff();
      break;
  }

  // Update previous sensor state for edge detection
  duck_detected_prev_ = duck_detected_;
}

void IntakeSubsystem::transitionTo(IntakeState new_state) {
  if (state_ != new_state) {
    state_ = new_state;
    state_entry_time_ms_ = millis();
  }
}

// Motor Control

void IntakeSubsystem::setMotorForward() { motor_state_ = MotorState::FORWARD; }

void IntakeSubsystem::setMotorReverse() { motor_state_ = MotorState::REVERSE; }

void IntakeSubsystem::setMotorOff() { motor_state_ = MotorState::OFF; }

void IntakeSubsystem::applyMotorState() {
  switch (motor_state_) {
    case MotorState::OFF:
      analogWrite(setup_.motor_pwm_pin_, 0);
      digitalWrite(setup_.motor_dir_pin_, LOW);
      break;

    case MotorState::FORWARD:
      digitalWrite(setup_.motor_dir_pin_, HIGH);
      analogWrite(setup_.motor_pwm_pin_, setup_.motor_speed_);
      break;

    case MotorState::REVERSE:
      digitalWrite(setup_.motor_dir_pin_, LOW);
      analogWrite(setup_.motor_pwm_pin_, setup_.motor_speed_);
      break;
  }
}

// Sensor Reading

void IntakeSubsystem::readSensor() {
  // IR breakbeam: LOW when beam is broken (duck present)
  // Adjust logic if your sensor is active-high
  duck_detected_ = (digitalRead(setup_.ir_sensor_pin_) == LOW);
}

// ROS Publishing

void IntakeSubsystem::publishState() {
  // Only publish if ROS is connected
  if (!state_pub_.impl) return;

  // Fill message
  state_msg_.header.stamp.sec = (int32_t)(millis() / 1000);
  state_msg_.header.stamp.nanosec = (uint32_t)((millis() % 1000) * 1000000);

  state_msg_.state = static_cast<uint8_t>(state_);
  state_msg_.duck_detected = duck_detected_;
  state_msg_.motor_state = static_cast<uint8_t>(motor_state_);
  state_msg_.capture_count = capture_count_;
  state_msg_.jam_count = jam_count_;
  state_msg_.time_in_state_ms = millis() - state_entry_time_ms_;

  // Publish (non-blocking, best effort)
#ifdef USE_TEENSYTHREADS
  {
    Threads::Scope guard(g_microros_mutex);
    (void)rcl_publish(&state_pub_, &state_msg_, NULL);
  }
#else
  (void)rcl_publish(&state_pub_, &state_msg_, NULL);
#endif
}

}  // namespace Subsystem
