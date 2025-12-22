/**
 * @file keypad_enter.cpp
 * @author Rafeed Khan
 * @brief Implementation of keypad entry task for Antenna #4
 */

#include "secbot_autonomy/keypad_enter.hpp"

namespace secbot {

float KeypadEnterTask::clamp01(float x) {
  if (x < 0.0f) return 0.0f;
  if (x > 1.0f) return 1.0f;
  return x;
}

KeypadEnterTask::KeypadEnterTask(rclcpp::Node::SharedPtr node,
                                 const KeypadEnterConfig& cfg)
    : TaskBase(node), cfg_(cfg) {
  arm_pub_ = node_->create_publisher<mcu_msgs::msg::ArmCommand>(
      cfg_.arm_command_topic, 10);

  key_target_pub_ =
      node_->create_publisher<std_msgs::msg::Char>(cfg_.key_target_topic, 10);

  at_key_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
      cfg_.at_key_topic, 10,
      std::bind(&KeypadEnterTask::onAtKey, this, std::placeholders::_1));
}

void KeypadEnterTask::onAtKey(const std_msgs::msg::Bool::SharedPtr msg) {
  at_key_ = msg->data;
  at_key_valid_ = true;
}

void KeypadEnterTask::enterState(State s) {
  state_ = s;
  state_entry_time_ = node_->now();
}

void KeypadEnterTask::commandPusher(int16_t position) {
  mcu_msgs::msg::ArmCommand msg;
  msg.joint_id = cfg_.pusher_joint_id;
  msg.position = position;
  msg.speed = cfg_.actuator_speed;
  arm_pub_->publish(msg);
}

void KeypadEnterTask::commandKeyTarget(char key) {
  std_msgs::msg::Char msg;
  msg.data = key;
  key_target_pub_->publish(msg);
}

void KeypadEnterTask::start() {
  if (cfg_.code.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "KeypadEnter: No code configured");
    status_ = TaskStatus::kFailed;
    return;
  }

  status_ = TaskStatus::kRunning;
  progress_ = 0.0f;
  start_time_ = node_->now();

  // Reset sequence
  idx_ = 0;
  cur_key_ = cfg_.code[0];
  at_key_ = false;

  // Start with pusher released
  commandPusher(cfg_.release_position);
  enterState(State::kSettle);

  RCLCPP_INFO(node_->get_logger(), "KeypadEnter: Starting (code: %s)",
              cfg_.code.c_str());
}

void KeypadEnterTask::step() {
  if (status_ != TaskStatus::kRunning) {
    return;
  }

  float t_total = (node_->now() - start_time_).seconds();
  float t_state = (node_->now() - state_entry_time_).seconds();

  // Timeout check
  if (cfg_.timeout_s > 0.0f && t_total >= cfg_.timeout_s) {
    RCLCPP_WARN(node_->get_logger(), "KeypadEnter: Timeout");
    commandPusher(cfg_.release_position);
    status_ = TaskStatus::kFailed;
    state_ = State::kDone;
    return;
  }

  // Progress: based on current index
  float code_len = static_cast<float>(cfg_.code.length());
  if (code_len > 0) {
    progress_ = clamp01(static_cast<float>(idx_) / code_len);
  }

  // State machine
  switch (state_) {
    case State::kSettle:
      // Keep released while settling
      commandPusher(cfg_.release_position);

      if (t_state >= cfg_.settle_s) {
        cur_key_ = cfg_.code[idx_];
        commandKeyTarget(cur_key_);
        at_key_ = false;  // Reset for new target
        enterState(State::kMoveToKey);
        RCLCPP_DEBUG(node_->get_logger(), "KeypadEnter: Moving to key '%c'",
                     cur_key_);
      }
      break;

    case State::kMoveToKey:
      // Command target continuously
      commandKeyTarget(cur_key_);
      commandPusher(cfg_.release_position);

      // Transition when at key or timeout
      if ((at_key_valid_ && at_key_) || t_state >= cfg_.move_timeout_s) {
        commandPusher(cfg_.press_position);
        enterState(State::kPressHold);
        RCLCPP_DEBUG(node_->get_logger(), "KeypadEnter: Pressing key '%c'",
                     cur_key_);
      }
      break;

    case State::kPressHold:
      // Hold key pressed
      commandPusher(cfg_.press_position);

      if (t_state >= cfg_.press_hold_s) {
        commandPusher(cfg_.release_position);
        enterState(State::kReleaseHold);
      }
      break;

    case State::kReleaseHold:
      // Hold released
      commandPusher(cfg_.release_position);

      if (t_state >= cfg_.release_hold_s) {
        idx_++;

        if (idx_ >= cfg_.code.length()) {
          // All keys entered!
          status_ = TaskStatus::kSucceeded;
          progress_ = 1.0f;
          state_ = State::kDone;
          RCLCPP_INFO(node_->get_logger(), "KeypadEnter: Complete!");
        } else {
          // Move to next key
          cur_key_ = cfg_.code[idx_];
          commandKeyTarget(cur_key_);
          at_key_ = false;
          enterState(State::kMoveToKey);
          RCLCPP_DEBUG(node_->get_logger(), "KeypadEnter: Moving to key '%c'",
                       cur_key_);
        }
      }
      break;

    case State::kDone:
      commandPusher(cfg_.release_position);
      break;

    case State::kIdle:
    default:
      break;
  }
}

void KeypadEnterTask::cancel() {
  if (status_ == TaskStatus::kRunning) {
    commandPusher(cfg_.release_position);
    status_ = TaskStatus::kFailed;
    state_ = State::kDone;
    RCLCPP_INFO(node_->get_logger(), "KeypadEnter: Cancelled at key %d ('%c')",
                idx_, cur_key_);
  }
}

void KeypadEnterTask::reset() {
  TaskBase::reset();
  commandPusher(cfg_.release_position);
  state_ = State::kIdle;
  idx_ = 0;
  cur_key_ = '\0';
  at_key_ = false;
}

}  // namespace secbot
