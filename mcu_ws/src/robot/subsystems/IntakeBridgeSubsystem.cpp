#include "IntakeBridgeSubsystem.h"

#include <Adafruit_VL53L0X.h>
#include <Arduino.h>

namespace Subsystem {

// VL53L0X TOF sensor instance (one per bridge subsystem)
static Adafruit_VL53L0X s_tof;
static bool s_tof_initialized = false;

// Static reference for ROS callback
static IntakeBridgeSubsystem* s_instance = nullptr;

// Lifecycle Hooks

bool IntakeBridgeSubsystem::init() {
  s_instance = this;

  // Configure rack motor pins
  // TODO: confirm pin numbers with electrical team
  pinMode(setup_.rack_pwm_pin_, OUTPUT);
  pinMode(setup_.rack_dir_pin_, OUTPUT);

  // Home switch -- active LOW (reads 0 when at home position)
  pinMode(setup_.home_switch_pin_, INPUT_PULLUP);

  // Set PWM frequency for Teensy motor smoothness
  analogWriteFrequency(setup_.rack_pwm_pin_, 18310.55);

  // Ensure motor is off at init
  setMotorOff();
  applyMotorState();

  // Configure TOF XSHUT pin and init sensor
  pinMode(setup_.tof_xshut_pin_, OUTPUT);
  digitalWrite(setup_.tof_xshut_pin_, HIGH);  // Enable sensor
  delay(10);

  // Initialize VL53L0X at custom I2C address
  if (s_tof.begin(setup_.tof_i2c_addr_)) {
    s_tof_initialized = true;
    // Start continuous ranging for fast reads
    s_tof.startRangeContinuous();
  } else {
    s_tof_initialized = false;
    // Non-fatal: bridge can still operate without TOF feedback
  }

  // Initialize state
  state_ = IntakeBridgeState::STOWED;
  state_entry_time_ms_ = millis();

  return true;
}

void IntakeBridgeSubsystem::begin() { transitionTo(IntakeBridgeState::STOWED); }

void IntakeBridgeSubsystem::update() {
  // Read home switch (active LOW -- 0 means at home)
  home_switch_active_ = (digitalRead(setup_.home_switch_pin_) == LOW);

  // Read TOF sensor
  readTofSensor();

  // Run state machine
  updateStateMachine();

  // Apply motor output
  applyMotorState();

  // Publish state to ROS at 20 Hz
  if (everyMs(50)) {
    publishState();
  }
}

void IntakeBridgeSubsystem::pause() {
  setMotorOff();
  applyMotorState();
  transitionTo(IntakeBridgeState::STOWED);
}

void IntakeBridgeSubsystem::reset() {
  setMotorOff();
  applyMotorState();
  transitionTo(IntakeBridgeState::STOWED);
  has_pending_command_ = false;
  extend_count_ = 0;
  retract_count_ = 0;
}

const char* IntakeBridgeSubsystem::getInfo() {
  static const char info[] = "IntakeBridgeSubsystem";
  return info;
}

// Micro-ROS Hooks

bool IntakeBridgeSubsystem::onCreate(rcl_node_t* node,
                                     rclc_executor_t* executor) {
  node_ = node;

  // Initialize messages
  mcu_msgs__msg__IntakeBridgeState__init(&state_msg_);
  mcu_msgs__msg__IntakeBridgeCommand__init(&cmd_msg_);

  // State publisher, best effort QoS
  if (rclc_publisher_init_best_effort(
          &state_pub_, node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(mcu_msgs, msg, IntakeBridgeState),
          "/mcu_robot/intake_bridge/state") != RCL_RET_OK) {
    return false;
  }

  // Command subscriber
  if (rclc_subscription_init_default(
          &cmd_sub_, node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(mcu_msgs, msg, IntakeBridgeCommand),
          "/mcu_robot/intake_bridge/command") != RCL_RET_OK) {
    return false;
  }

  // Add subscription to executor
  if (rclc_executor_add_subscription(executor, &cmd_sub_, &cmd_msg_,
                                     &IntakeBridgeSubsystem::commandCallback,
                                     ON_NEW_DATA) != RCL_RET_OK) {
    return false;
  }

  return true;
}

void IntakeBridgeSubsystem::onDestroy() {
  if (state_pub_.impl) {
    (void)rcl_publisher_fini(&state_pub_, node_);
  }
  if (cmd_sub_.impl) {
    (void)rcl_subscription_fini(&cmd_sub_, node_);
  }
  mcu_msgs__msg__IntakeBridgeState__fini(&state_msg_);
  mcu_msgs__msg__IntakeBridgeCommand__fini(&cmd_msg_);
  node_ = nullptr;
}

// Static ROS command callback
void IntakeBridgeSubsystem::commandCallback(const void* msgin) {
  if (!s_instance || !msgin) return;
  const auto* msg =
      static_cast<const mcu_msgs__msg__IntakeBridgeCommand*>(msgin);

  switch (msg->command) {
    case mcu_msgs__msg__IntakeBridgeCommand__CMD_EXTEND:
      s_instance->extend();
      break;
    case mcu_msgs__msg__IntakeBridgeCommand__CMD_RETRACT:
      s_instance->retract();
      break;
    case mcu_msgs__msg__IntakeBridgeCommand__CMD_STOW:
    default:
      s_instance->stow();
      break;
  }
}

// Public Commands

void IntakeBridgeSubsystem::extend() {
  if (state_ == IntakeBridgeState::STOWED ||
      state_ == IntakeBridgeState::EXTENDED) {
    pending_command_ = IntakeBridgeCommand::EXTEND;
    has_pending_command_ = true;
  }
}

void IntakeBridgeSubsystem::retract() {
  if (state_ == IntakeBridgeState::EXTENDED ||
      state_ == IntakeBridgeState::EXTENDING) {
    pending_command_ = IntakeBridgeCommand::RETRACT;
    has_pending_command_ = true;
  }
}

void IntakeBridgeSubsystem::stow() {
  pending_command_ = IntakeBridgeCommand::STOW;
  has_pending_command_ = true;
}

// State Machine Logic

void IntakeBridgeSubsystem::updateStateMachine() {
  uint32_t now = millis();
  uint32_t time_in_state = now - state_entry_time_ms_;

  // Process pending commands
  if (has_pending_command_) {
    switch (pending_command_) {
      case IntakeBridgeCommand::EXTEND:
        if (state_ == IntakeBridgeState::STOWED) {
          transitionTo(IntakeBridgeState::EXTENDING);
        }
        break;

      case IntakeBridgeCommand::RETRACT:
        if (state_ == IntakeBridgeState::EXTENDED ||
            state_ == IntakeBridgeState::EXTENDING) {
          transitionTo(IntakeBridgeState::RETRACTING);
        }
        break;

      case IntakeBridgeCommand::STOW:
        if (state_ != IntakeBridgeState::ERROR) {
          setMotorOff();
          transitionTo(IntakeBridgeState::STOWED);
        }
        break;
    }
    has_pending_command_ = false;
  }

  // State-specific logic
  switch (state_) {
    case IntakeBridgeState::STOWED:
      // If home switch is active, don't move in retract direction
      setMotorOff();
      break;

    case IntakeBridgeState::EXTENDING:
      setMotorExtend();
      // Timeout check -- if extend takes too long, error
      if (time_in_state >= setup_.extend_timeout_ms_) {
        extend_count_++;
        transitionTo(IntakeBridgeState::EXTENDED);
      }
      break;

    case IntakeBridgeState::EXTENDED:
      setMotorOff();
      // Stay here until commanded to retract
      // Update duck detection from TOF
      duck_detected_ = s_tof_initialized &&
                       (tof_distance_mm_ < setup_.duck_detect_threshold_mm_) &&
                       (tof_distance_mm_ > 0);
      break;

    case IntakeBridgeState::RETRACTING:
      // Home switch active (LOW) -- stop, we're home
      if (home_switch_active_) {
        setMotorOff();
        retract_count_++;
        transitionTo(IntakeBridgeState::STOWED);
      } else {
        setMotorRetract();
        // Safety timeout fallback if switch never triggers
        if (time_in_state >= setup_.retract_timeout_ms_) {
          retract_count_++;
          transitionTo(IntakeBridgeState::STOWED);
        }
      }
      break;

    case IntakeBridgeState::ERROR:
      setMotorOff();
      break;
  }
}

void IntakeBridgeSubsystem::transitionTo(IntakeBridgeState new_state) {
  if (state_ != new_state) {
    state_ = new_state;
    state_entry_time_ms_ = millis();
  }
}

// Motor Control

void IntakeBridgeSubsystem::setMotorExtend() {
  motor_state_ = MotorState::EXTENDING;
}

void IntakeBridgeSubsystem::setMotorRetract() {
  motor_state_ = MotorState::RETRACTING;
}

void IntakeBridgeSubsystem::setMotorOff() { motor_state_ = MotorState::OFF; }

void IntakeBridgeSubsystem::applyMotorState() {
  switch (motor_state_) {
    case MotorState::OFF:
      analogWrite(setup_.rack_pwm_pin_, 0);
      digitalWrite(setup_.rack_dir_pin_, LOW);
      break;

    case MotorState::EXTENDING:
      digitalWrite(setup_.rack_dir_pin_, HIGH);
      analogWrite(setup_.rack_pwm_pin_, setup_.motor_speed_);
      break;

    case MotorState::RETRACTING:
      digitalWrite(setup_.rack_dir_pin_, LOW);
      analogWrite(setup_.rack_pwm_pin_, setup_.motor_speed_);
      break;
  }
}

// Sensor Reading

void IntakeBridgeSubsystem::readTofSensor() {
  if (!s_tof_initialized) {
    tof_distance_mm_ = 0;
    duck_detected_ = false;
    return;
  }

  if (s_tof.isRangeComplete()) {
    tof_distance_mm_ = s_tof.readRange();
    // Only update duck_detected in EXTENDED state (done in updateStateMachine)
  }
}

// ROS Publishing

void IntakeBridgeSubsystem::publishState() {
  if (!state_pub_.impl) return;

  state_msg_.header.stamp.sec = (int32_t)(millis() / 1000);
  state_msg_.header.stamp.nanosec = (uint32_t)((millis() % 1000) * 1000000);

  state_msg_.state = static_cast<uint8_t>(state_);
  state_msg_.duck_detected = duck_detected_;
  state_msg_.tof_distance_mm = tof_distance_mm_;
  state_msg_.rack_motor_state = static_cast<uint8_t>(motor_state_);
  state_msg_.extend_count = extend_count_;
  state_msg_.retract_count = retract_count_;
  state_msg_.time_in_state_ms = millis() - state_entry_time_ms_;

  (void)rcl_publish(&state_pub_, &state_msg_, NULL);
}

}  // namespace Subsystem
