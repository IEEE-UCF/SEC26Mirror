/**
 * @file crank_turn.cpp
 * @author Rafeed Khan
 * @brief Implementation of crank turn task for Antenna #2
 */

#include "secbot_autonomy/crank_turn.hpp"

#include <cmath>

namespace secbot {

// DegUnwrap implementation

float CrankTurnTask::wrapDeltaDeg(float d) {
  while (d <= -180.0f) d += 360.0f;
  while (d > 180.0f) d -= 360.0f;
  return d;
}

void CrankTurnTask::DegUnwrap::reset(float deg) {
  last = deg;
  acc = deg;
  inited = true;
}

float CrankTurnTask::DegUnwrap::update(float deg) {
  if (!inited) {
    reset(deg);
    return acc;
  }
  const float d = CrankTurnTask::wrapDeltaDeg(deg - last);
  acc += d;
  last = deg;
  return acc;
}

// Helpers

float CrankTurnTask::clamp01(float x) {
  if (x < 0.0f) return 0.0f;
  if (x > 1.0f) return 1.0f;
  return x;
}

void CrankTurnTask::enterState(State s) {
  state_ = s;
  state_entry_time_ = node_->now();
  t_state_ = 0.0f;
}

void CrankTurnTask::commandMotor(int16_t speed) {
  secbot_msgs::msg::ArmCommand msg;
  msg.joint_id = cfg_.crank_joint_id;
  msg.position = speed;  // Using position field for speed command
  msg.speed = cfg_.spin_speed;
  arm_pub_->publish(msg);
}

void CrankTurnTask::stopMotor() {
  commandMotor(0);
}

// Constructor

CrankTurnTask::CrankTurnTask(rclcpp::Node::SharedPtr node,
                             const CrankTurnConfig& cfg)
    : TaskBase(node), cfg_(cfg) {
  arm_pub_ = node_->create_publisher<secbot_msgs::msg::ArmCommand>(
      cfg_.arm_command_topic, 10);

  encoder_sub_ = node_->create_subscription<std_msgs::msg::Float32>(
      cfg_.crank_encoder_topic, 10,
      std::bind(&CrankTurnTask::onEncoderAngle, this, std::placeholders::_1));

  complete_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
      cfg_.task_complete_topic, 10,
      std::bind(&CrankTurnTask::onTaskComplete, this, std::placeholders::_1));
}

// Callbacks

void CrankTurnTask::onEncoderAngle(const std_msgs::msg::Float32::SharedPtr msg) {
  current_encoder_deg_ = msg->data;
  encoder_valid_ = true;
}

void CrankTurnTask::onTaskComplete(const std_msgs::msg::Bool::SharedPtr msg) {
  task_complete_signal_ = msg->data;
}

// TaskBase interface

void CrankTurnTask::start() {
  // Compute open-loop spin time
  if (cfg_.open_loop_spin_s > 0.0f) {
    spin_goal_s_ = cfg_.open_loop_spin_s;
  } else {
    const float rate = (cfg_.est_deg_per_s > 1e-3f) ? cfg_.est_deg_per_s : 1e-3f;
    spin_goal_s_ = cfg_.target_deg / rate;
  }

  // Reset state
  progress_ = 0.0f;
  turned_deg_ = 0.0f;
  task_complete_signal_ = false;
  start_time_ = node_->now();

  // Prime encoder tracking if available
  if (encoder_valid_) {
    unwrap_.reset(current_encoder_deg_);
    start_unwrapped_deg_ = unwrap_.acc;
  } else {
    unwrap_.inited = false;
    start_unwrapped_deg_ = 0.0f;
  }

  stopMotor();
  status_ = TaskStatus::kRunning;
  enterState(State::kSettle);

  RCLCPP_INFO(node_->get_logger(), "CrankTurn: Starting (target %.0f deg)",
              cfg_.target_deg);
}

void CrankTurnTask::step() {
  if (status_ != TaskStatus::kRunning) {
    return;
  }

  // Update timing
  t_state_ = (node_->now() - state_entry_time_).seconds();
  float t_total = (node_->now() - start_time_).seconds();

  // Timeout check
  if (cfg_.timeout_s > 0.0f && t_total >= cfg_.timeout_s) {
    RCLCPP_WARN(node_->get_logger(), "CrankTurn: Timeout");
    stopMotor();
    status_ = TaskStatus::kFailed;
    state_ = State::kDone;
    return;
  }

  // External completion signal wins immediately
  if (task_complete_signal_) {
    stopMotor();
    status_ = TaskStatus::kSucceeded;
    progress_ = 1.0f;
    enterState(State::kDone);
    RCLCPP_INFO(node_->get_logger(), "CrankTurn: Completed via external signal");
    return;
  }

  // Update turned degrees
  if (encoder_valid_) {
    const float uw = unwrap_.update(current_encoder_deg_);
    turned_deg_ = std::fabs(uw - start_unwrapped_deg_);
  } else if (state_ == State::kSpin && spin_goal_s_ > 1e-3f) {
    // Open-loop estimate
    turned_deg_ = cfg_.target_deg * clamp01(t_state_ / spin_goal_s_);
  }

  // Progress
  if (cfg_.target_deg > 1e-3f) {
    progress_ = clamp01(turned_deg_ / cfg_.target_deg);
  } else {
    progress_ = 1.0f;
  }

  // State machine
  switch (state_) {
    case State::kSettle:
      stopMotor();
      if (t_state_ >= cfg_.settle_s) {
        enterState(State::kSpin);
      }
      break;

    case State::kSpin: {
      // Command motor in configured direction
      int16_t cmd = static_cast<int16_t>(cfg_.spin_speed);
      if (cfg_.direction < 0) cmd = -cmd;
      commandMotor(cmd);

      bool done = false;
      if (encoder_valid_) {
        // Closed-loop: stop when turned enough
        if (turned_deg_ >= (cfg_.target_deg - cfg_.deg_tolerance)) {
          done = true;
        }
      } else {
        // Open-loop: stop when time elapsed
        if (t_state_ >= spin_goal_s_) {
          done = true;
        }
      }

      if (done) {
        stopMotor();
        enterState(State::kHold);
      }
    } break;

    case State::kHold:
      stopMotor();
      if (t_state_ >= cfg_.hold_s) {
        status_ = TaskStatus::kSucceeded;
        progress_ = 1.0f;
        enterState(State::kDone);
        RCLCPP_INFO(node_->get_logger(), "CrankTurn: Completed (%.0f deg)",
                    turned_deg_);
      }
      break;

    case State::kDone:
    case State::kIdle:
    default:
      break;
  }
}

void CrankTurnTask::cancel() {
  if (status_ == TaskStatus::kRunning) {
    stopMotor();
    status_ = TaskStatus::kFailed;
    state_ = State::kDone;
    RCLCPP_INFO(node_->get_logger(), "CrankTurn: Cancelled");
  }
}

void CrankTurnTask::reset() {
  TaskBase::reset();
  stopMotor();
  state_ = State::kIdle;
  turned_deg_ = 0.0f;
  spin_goal_s_ = 0.0f;
  unwrap_.inited = false;
  start_unwrapped_deg_ = 0.0f;
  task_complete_signal_ = false;
}

}  // namespace secbot
