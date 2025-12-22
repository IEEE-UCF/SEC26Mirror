/**
 * @file button_press.cpp
 * @author Rafeed Khan
 * @brief Implementation of button press task for Antenna #1
 */

#include "secbot_autonomy/button_press.hpp"

namespace secbot {

namespace {
float clamp01(float x) {
  if (x < 0.0f) return 0.0f;
  if (x > 1.0f) return 1.0f;
  return x;
}
}  // namespace

ButtonPressTask::ButtonPressTask(rclcpp::Node::SharedPtr node,
                                 const ButtonPressConfig& cfg)
    : TaskBase(node), cfg_(cfg) {
  arm_pub_ = node_->create_publisher<secbot_msgs::msg::ArmCommand>(
      cfg_.arm_command_topic, 10);
}

void ButtonPressTask::enterState(State s) {
  state_ = s;
  state_entry_time_ = node_->now();
  t_state_ = 0.0f;
}

void ButtonPressTask::commandPusher(int16_t position) {
  secbot_msgs::msg::ArmCommand msg;
  msg.joint_id = cfg_.pusher_joint_id;
  msg.position = position;
  msg.speed = cfg_.actuator_speed;
  arm_pub_->publish(msg);
}

void ButtonPressTask::start() {
  status_ = TaskStatus::kRunning;
  presses_done_ = 0;
  progress_ = 0.0f;
  start_time_ = node_->now();

  // Start with pusher retracted
  commandPusher(cfg_.release_position);
  enterState(State::kSettle);

  RCLCPP_INFO(node_->get_logger(), "ButtonPress: Starting (%d presses)",
              cfg_.presses_total);
}

void ButtonPressTask::step() {
  if (status_ != TaskStatus::kRunning) {
    return;
  }

  // Update time in current state
  t_state_ = (node_->now() - state_entry_time_).seconds();
  float t_total = (node_->now() - start_time_).seconds();

  // Timeout check
  if (cfg_.timeout_s > 0.0f && t_total >= cfg_.timeout_s) {
    RCLCPP_WARN(node_->get_logger(), "ButtonPress: Timeout");
    commandPusher(cfg_.release_position);
    status_ = TaskStatus::kFailed;
    state_ = State::kDone;
    return;
  }

  // Progress calculation
  const float total = (cfg_.presses_total == 0)
                          ? 1.0f
                          : static_cast<float>(cfg_.presses_total);
  float phase_frac = 0.0f;

  if (state_ == State::kSettle && cfg_.settle_s > 0.0f) {
    phase_frac = clamp01(t_state_ / cfg_.settle_s) * (1.0f / total) * 0.10f;
  } else if (state_ == State::kPressHold && cfg_.press_hold_s > 0.0f) {
    phase_frac = clamp01(t_state_ / cfg_.press_hold_s) * (1.0f / total) * 0.50f;
  } else if (state_ == State::kReleaseHold && cfg_.release_hold_s > 0.0f) {
    phase_frac = clamp01(t_state_ / cfg_.release_hold_s) * (1.0f / total) * 0.40f;
  }
  progress_ = clamp01((static_cast<float>(presses_done_) / total) + phase_frac);

  // State machine
  switch (state_) {
    case State::kSettle:
      // Keep released while settling
      if (t_state_ >= cfg_.settle_s) {
        commandPusher(cfg_.press_position);
        enterState(State::kPressHold);
      }
      break;

    case State::kPressHold:
      // Holding in pressed position
      if (t_state_ >= cfg_.press_hold_s) {
        commandPusher(cfg_.release_position);
        enterState(State::kReleaseHold);
      }
      break;

    case State::kReleaseHold:
      // Holding in released position
      if (t_state_ >= cfg_.release_hold_s) {
        presses_done_++;

        if (presses_done_ >= cfg_.presses_total) {
          // Done!!!!!!!!
          RCLCPP_INFO(node_->get_logger(), "ButtonPress: Completed %d presses",
                      presses_done_);
          status_ = TaskStatus::kSucceeded;
          progress_ = 1.0f;
          enterState(State::kDone);
        } else {
          // Next press
          commandPusher(cfg_.press_position);
          enterState(State::kPressHold);
        }
      }
      break;

    case State::kDone:
      // Keep retracted
      break;

    case State::kIdle:
    default:
      //This SHOULDNT happen while running
      status_ = TaskStatus::kFailed;
      break;
  }
}

void ButtonPressTask::cancel() {
  if (status_ == TaskStatus::kRunning) {
    commandPusher(cfg_.release_position);
    status_ = TaskStatus::kFailed;
    state_ = State::kDone;
    RCLCPP_INFO(node_->get_logger(), "ButtonPress: Cancelled");
  }
}

void ButtonPressTask::reset() {
  TaskBase::reset();
  commandPusher(cfg_.release_position);
  state_ = State::kIdle;
  presses_done_ = 0;
  t_state_ = 0.0f;
}

}  // namespace secbot
