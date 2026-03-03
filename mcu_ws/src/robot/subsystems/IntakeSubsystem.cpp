#include "IntakeSubsystem.h"

#include <Arduino.h>

#include "DebugLog.h"

namespace Subsystem {

// ═══════════════════════════════════════════════════════════════════════════
//  Lifecycle
// ═══════════════════════════════════════════════════════════════════════════

bool IntakeSubsystem::init() {
  if (!setup_.motor_mgr_ || !setup_.encoder_sub_ || !setup_.btn_sub_) {
    DEBUG_PRINTLN("[INTAKE] init FAIL: missing dependency");
    return false;
  }

  state_ = IntakeState::UNINITIALIZED;
  calibrated_ = false;
  setpoint_ = 0.0f;
  intake_speed_ = 0.0f;
  min_ticks_ = 0;
  max_ticks_ = 0;
  state_entry_ms_ = millis();

  DEBUG_PRINTLN("[INTAKE] init OK");
  return true;
}

void IntakeSubsystem::begin() {}

void IntakeSubsystem::update() {
  // Enforce hard stops every cycle (all states except UNINITIALIZED)
  if (state_ != IntakeState::UNINITIALIZED) {
    enforceHardStops();
  }

  // State-specific logic
  switch (state_) {
    case IntakeState::UNINITIALIZED:
      // Waiting for initialize() command
      break;

    case IntakeState::CALIBRATING:
      updateCalibration();
      break;

    case IntakeState::IDLE:
      // Nothing to do — motor already off
      break;

    case IntakeState::MOVING:
      updateMoving();
      break;

    case IntakeState::FAULT:
      // Motors stopped, waiting for reset
      break;
  }

  // Publish state at ~20 Hz
  if (everyMs(50)) {
    publishState();
  }
}

void IntakeSubsystem::pause() {
  setup_.motor_mgr_->setSpeed(setup_.rail_motor_idx_, 0.0f);
  setup_.motor_mgr_->setSpeed(setup_.intake_motor_idx_, 0.0f);
  intake_speed_ = 0.0f;
}

void IntakeSubsystem::reset() {
  pause();
  state_ = IntakeState::UNINITIALIZED;
  calibrated_ = false;
  setpoint_ = 0.0f;
  min_ticks_ = 0;
  max_ticks_ = 0;
  state_entry_ms_ = millis();
}

const char* IntakeSubsystem::getInfo() {
  static const char info[] = "IntakeSubsystem";
  return info;
}

// ═══════════════════════════════════════════════════════════════════════════
//  micro-ROS
// ═══════════════════════════════════════════════════════════════════════════

bool IntakeSubsystem::onCreate(rcl_node_t* node, rclc_executor_t* executor) {
  node_ = node;

  mcu_msgs__msg__IntakeState__init(&state_msg_);
  mcu_msgs__msg__IntakeCommand__init(&cmd_msg_);

  // State publisher (best effort)
  if (rclc_publisher_init_best_effort(
          &state_pub_, node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(mcu_msgs, msg, IntakeState),
          setup_.state_topic_) != RCL_RET_OK) {
    DEBUG_PRINTLN("[INTAKE] onCreate FAIL: state publisher");
    return false;
  }

  // Command subscription
  if (rclc_subscription_init_default(
          &cmd_sub_, node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(mcu_msgs, msg, IntakeCommand),
          setup_.cmd_topic_) != RCL_RET_OK) {
    DEBUG_PRINTLN("[INTAKE] onCreate FAIL: command subscription");
    return false;
  }

  if (rclc_executor_add_subscription_with_context(
          executor, &cmd_sub_, &cmd_msg_, &IntakeSubsystem::cmdCallback, this,
          ON_NEW_DATA) != RCL_RET_OK) {
    DEBUG_PRINTLN("[INTAKE] onCreate FAIL: executor add sub");
    return false;
  }

  DEBUG_PRINTLN("[INTAKE] onCreate OK");
  return true;
}

void IntakeSubsystem::onDestroy() {
  state_pub_ = rcl_get_zero_initialized_publisher();
  cmd_sub_ = rcl_get_zero_initialized_subscription();
  mcu_msgs__msg__IntakeState__fini(&state_msg_);
  mcu_msgs__msg__IntakeCommand__fini(&cmd_msg_);
  node_ = nullptr;
}

// ═══════════════════════════════════════════════════════════════════════════
//  Public API — Linear Rail
// ═══════════════════════════════════════════════════════════════════════════

void IntakeSubsystem::initialize() {
  if (state_ == IntakeState::CALIBRATING) return;  // Already running

  DEBUG_PRINTLN("[INTAKE] Starting calibration");
  calib_phase_ = CalibPhase::SEEK_RETRACT;
  calib_tried_reverse_ = false;
  calib_last_ticks_ = setup_.encoder_sub_->getAccumulatedTicks(
      setup_.rail_encoder_idx_);
  calib_last_change_ms_ = millis();
  transitionTo(IntakeState::CALIBRATING);
}

void IntakeSubsystem::setPosition(float normalized) {
  if (!calibrated_) {
    DEBUG_PRINTLN("[INTAKE] setPosition ignored: not calibrated");
    return;
  }
  setpoint_ = constrain(normalized, 0.0f, 1.0f);
  if (state_ == IntakeState::IDLE || state_ == IntakeState::MOVING) {
    transitionTo(IntakeState::MOVING);
  }
}

void IntakeSubsystem::stopRail() {
  setup_.motor_mgr_->setSpeed(setup_.rail_motor_idx_, 0.0f);
  if (state_ == IntakeState::MOVING) {
    transitionTo(IntakeState::IDLE);
  }
}

float IntakeSubsystem::getPosition() const {
  if (!calibrated_ || max_ticks_ == min_ticks_) return 0.0f;
  int32_t current = setup_.encoder_sub_->getAccumulatedTicks(
      setup_.rail_encoder_idx_);
  float pos = (float)(current - min_ticks_) / (float)(max_ticks_ - min_ticks_);
  return constrain(pos, 0.0f, 1.0f);
}

// ═══════════════════════════════════════════════════════════════════════════
//  Public API — Intake Motor
// ═══════════════════════════════════════════════════════════════════════════

void IntakeSubsystem::setIntakeSpeed(float speed) {
  intake_speed_ = constrain(speed, -1.0f, 1.0f);
  setup_.motor_mgr_->setSpeed(setup_.intake_motor_idx_, intake_speed_);
}

// ═══════════════════════════════════════════════════════════════════════════
//  State Machine — Calibration
// ═══════════════════════════════════════════════════════════════════════════

void IntakeSubsystem::updateCalibration() {
  int32_t current_ticks = setup_.encoder_sub_->getAccumulatedTicks(
      setup_.rail_encoder_idx_);
  uint32_t now = millis();

  // Check for encoder movement — update stall timer
  if (current_ticks != calib_last_ticks_) {
    calib_last_ticks_ = current_ticks;
    calib_last_change_ms_ = now;
  }

  bool stalled = (now - calib_last_change_ms_) >= setup_.stall_timeout_ms_;

  switch (calib_phase_) {
    case CalibPhase::SEEK_RETRACT: {
      // Check if retract limit switch is hit
      if (setup_.btn_sub_->isPressed(setup_.retract_limit_btn_)) {
        setup_.motor_mgr_->setSpeed(setup_.rail_motor_idx_, 0.0f);
        min_ticks_ = current_ticks;
        DEBUG_PRINTF("[INTAKE] Retract limit hit, min_ticks=%d\n",
                     (int)min_ticks_);

        // Move to next phase: seek extend
        calib_phase_ = CalibPhase::SEEK_EXTEND;
        calib_tried_reverse_ = false;
        calib_last_change_ms_ = now;
        setup_.motor_mgr_->setSpeed(setup_.rail_motor_idx_,
                                    setup_.calibration_speed_);
        break;
      }

      // Drive toward retract (negative speed)
      setup_.motor_mgr_->setSpeed(setup_.rail_motor_idx_,
                                  -setup_.calibration_speed_);

      if (stalled) {
        if (!calib_tried_reverse_) {
          // Try extending instead (maybe we're already at retract limit)
          DEBUG_PRINTLN("[INTAKE] Stalled seeking retract, trying extend");
          calib_tried_reverse_ = true;
          // Assume we're at retract limit
          min_ticks_ = current_ticks;
          calib_phase_ = CalibPhase::SEEK_EXTEND;
          calib_last_change_ms_ = now;
          setup_.motor_mgr_->setSpeed(setup_.rail_motor_idx_,
                                      setup_.calibration_speed_);
        } else {
          // Stalled in both directions — fault
          DEBUG_PRINTLN("[INTAKE] FAULT: stalled in both directions");
          setup_.motor_mgr_->setSpeed(setup_.rail_motor_idx_, 0.0f);
          transitionTo(IntakeState::FAULT);
        }
      }
      break;
    }

    case CalibPhase::SEEK_EXTEND: {
      // Check if extend limit switch is hit
      if (setup_.btn_sub_->isPressed(setup_.extend_limit_btn_)) {
        setup_.motor_mgr_->setSpeed(setup_.rail_motor_idx_, 0.0f);
        max_ticks_ = current_ticks;
        DEBUG_PRINTF("[INTAKE] Extend limit hit, max_ticks=%d\n",
                     (int)max_ticks_);

        // Validate calibration (need meaningful range)
        if (max_ticks_ <= min_ticks_) {
          DEBUG_PRINTLN("[INTAKE] FAULT: invalid tick range");
          transitionTo(IntakeState::FAULT);
          break;
        }

        // Return home (retract to 0.0)
        calib_phase_ = CalibPhase::RETURN_HOME;
        setpoint_ = 0.0f;
        setup_.motor_mgr_->setSpeed(setup_.rail_motor_idx_,
                                    -setup_.calibration_speed_);
        break;
      }

      // Drive toward extend (positive speed)
      setup_.motor_mgr_->setSpeed(setup_.rail_motor_idx_,
                                  setup_.calibration_speed_);

      if (stalled) {
        // Stalled extending without hitting limit — fault
        DEBUG_PRINTLN("[INTAKE] FAULT: stalled seeking extend");
        setup_.motor_mgr_->setSpeed(setup_.rail_motor_idx_, 0.0f);
        transitionTo(IntakeState::FAULT);
      }
      break;
    }

    case CalibPhase::RETURN_HOME: {
      // Check if retract limit reached
      if (setup_.btn_sub_->isPressed(setup_.retract_limit_btn_)) {
        setup_.motor_mgr_->setSpeed(setup_.rail_motor_idx_, 0.0f);
        calibrated_ = true;
        DEBUG_PRINTF("[INTAKE] Calibration complete: min=%d max=%d range=%d\n",
                     (int)min_ticks_, (int)max_ticks_,
                     (int)(max_ticks_ - min_ticks_));
        transitionTo(IntakeState::IDLE);
        break;
      }

      // Use P control to return home
      int32_t target = min_ticks_;
      float error = (float)(target - current_ticks);
      float range = (float)(max_ticks_ - min_ticks_);
      float output = setup_.kp_ * (error / range);
      output = constrain(output, -setup_.calibration_speed_,
                         setup_.calibration_speed_);
      setup_.motor_mgr_->setSpeed(setup_.rail_motor_idx_, output);

      // If close enough, call it done
      if (fabsf(error / range) < setup_.position_tolerance_) {
        setup_.motor_mgr_->setSpeed(setup_.rail_motor_idx_, 0.0f);
        calibrated_ = true;
        DEBUG_PRINTF("[INTAKE] Calibration complete: min=%d max=%d range=%d\n",
                     (int)min_ticks_, (int)max_ticks_,
                     (int)(max_ticks_ - min_ticks_));
        transitionTo(IntakeState::IDLE);
      }
      break;
    }
  }
}

// ═══════════════════════════════════════════════════════════════════════════
//  State Machine — Moving (P controller)
// ═══════════════════════════════════════════════════════════════════════════

void IntakeSubsystem::updateMoving() {
  int32_t target_ticks =
      min_ticks_ + (int32_t)(setpoint_ * (float)(max_ticks_ - min_ticks_));
  int32_t current_ticks = setup_.encoder_sub_->getAccumulatedTicks(
      setup_.rail_encoder_idx_);

  float error = (float)(target_ticks - current_ticks);
  float range = (float)(max_ticks_ - min_ticks_);
  if (range == 0.0f) {
    transitionTo(IntakeState::FAULT);
    return;
  }

  float normalized_error = error / range;

  if (fabsf(normalized_error) < setup_.position_tolerance_) {
    // Arrived at setpoint
    setup_.motor_mgr_->setSpeed(setup_.rail_motor_idx_, 0.0f);
    transitionTo(IntakeState::IDLE);
    return;
  }

  float output = setup_.kp_ * normalized_error;
  output = constrain(output, -setup_.max_speed_, setup_.max_speed_);
  setup_.motor_mgr_->setSpeed(setup_.rail_motor_idx_, output);
}

// ═══════════════════════════════════════════════════════════════════════════
//  Hard Stop Logic
// ═══════════════════════════════════════════════════════════════════════════

void IntakeSubsystem::enforceHardStops() {
  bool at_retract = setup_.btn_sub_->isPressed(setup_.retract_limit_btn_);
  bool at_extend = setup_.btn_sub_->isPressed(setup_.extend_limit_btn_);

  float rail_speed = setup_.motor_mgr_->getSpeed(setup_.rail_motor_idx_);

  // If at retract limit and motor driving toward retract → stop
  if (at_retract && rail_speed < 0.0f) {
    setup_.motor_mgr_->setSpeed(setup_.rail_motor_idx_, 0.0f);
  }

  // If at extend limit and motor driving toward extend → stop
  if (at_extend && rail_speed > 0.0f) {
    setup_.motor_mgr_->setSpeed(setup_.rail_motor_idx_, 0.0f);
  }
}

// ═══════════════════════════════════════════════════════════════════════════
//  State Transition
// ═══════════════════════════════════════════════════════════════════════════

void IntakeSubsystem::transitionTo(IntakeState new_state) {
  if (state_ == new_state) return;

  static const char* const names[] = {"UNINITIALIZED", "CALIBRATING", "IDLE",
                                      "MOVING", "FAULT"};
  DEBUG_PRINTF("[INTAKE] %s -> %s\n", names[(uint8_t)state_],
               names[(uint8_t)new_state]);

  state_ = new_state;
  state_entry_ms_ = millis();

  // Stop rail motor on entering IDLE or FAULT
  if (new_state == IntakeState::IDLE || new_state == IntakeState::FAULT) {
    setup_.motor_mgr_->setSpeed(setup_.rail_motor_idx_, 0.0f);
  }
}

// ═══════════════════════════════════════════════════════════════════════════
//  ROS Publishing
// ═══════════════════════════════════════════════════════════════════════════

void IntakeSubsystem::publishState() {
  if (!state_pub_.impl) return;

  state_msg_.header.stamp.sec = (int32_t)(millis() / 1000);
  state_msg_.header.stamp.nanosec = (uint32_t)((millis() % 1000) * 1000000);

  state_msg_.state = static_cast<uint8_t>(state_);
  state_msg_.position = getPosition();
  state_msg_.setpoint = setpoint_;
  state_msg_.encoder_ticks = setup_.encoder_sub_->getAccumulatedTicks(
      setup_.rail_encoder_idx_);
  state_msg_.min_ticks = min_ticks_;
  state_msg_.max_ticks = max_ticks_;
  state_msg_.limit_retract_active =
      setup_.btn_sub_->isPressed(setup_.retract_limit_btn_);
  state_msg_.limit_extend_active =
      setup_.btn_sub_->isPressed(setup_.extend_limit_btn_);
  state_msg_.calibrated = calibrated_;
  state_msg_.intake_motor_speed = intake_speed_;
  state_msg_.time_in_state_ms = millis() - state_entry_ms_;

#ifdef USE_TEENSYTHREADS
  {
    Threads::Scope guard(g_microros_mutex);
    (void)rcl_publish(&state_pub_, &state_msg_, NULL);
  }
#else
  (void)rcl_publish(&state_pub_, &state_msg_, NULL);
#endif
}

// ═══════════════════════════════════════════════════════════════════════════
//  ROS Command Callback
// ═══════════════════════════════════════════════════════════════════════════

void IntakeSubsystem::cmdCallback(const void* msg, void* ctx) {
  auto* self = static_cast<IntakeSubsystem*>(ctx);
  auto* cmd = static_cast<const mcu_msgs__msg__IntakeCommand*>(msg);

  switch (cmd->command) {
    case mcu_msgs__msg__IntakeCommand__CMD_INITIALIZE:
      self->initialize();
      break;
    case mcu_msgs__msg__IntakeCommand__CMD_EXTEND:
      self->extend();
      break;
    case mcu_msgs__msg__IntakeCommand__CMD_RETRACT:
      self->retract();
      break;
    case mcu_msgs__msg__IntakeCommand__CMD_SET_POSITION:
      self->setPosition(cmd->value);
      break;
    case mcu_msgs__msg__IntakeCommand__CMD_SET_INTAKE_SPEED:
      self->setIntakeSpeed(cmd->value);
      break;
    case mcu_msgs__msg__IntakeCommand__CMD_STOP_RAIL:
      self->stopRail();
      break;
    case mcu_msgs__msg__IntakeCommand__CMD_STOP_INTAKE:
      self->stopIntake();
      break;
    default:
      DEBUG_PRINTF("[INTAKE] Unknown command: %d\n", cmd->command);
      break;
  }
}

}  // namespace Subsystem
