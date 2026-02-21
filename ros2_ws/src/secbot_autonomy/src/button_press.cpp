/**
 * @file button_press.cpp
 * @author Rafeed Khan
 * @brief Implementation of drive-based button press task for Antenna #1
 *
 * Robot drives forward into button, backs up, repeats.
 * All distances/speeds/offsets configurable via ButtonPressConfig.
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
  cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(
      cfg_.cmd_vel_topic, 10);
}

void ButtonPressTask::enterState(State s) {
  state_ = s;
  state_entry_time_ = node_->now();
}

void ButtonPressTask::publishVel(float vx, float vy, float wz) {
  geometry_msgs::msg::Twist msg;
  msg.linear.x = vx;
  msg.linear.y = vy;
  msg.angular.z = wz;
  cmd_vel_pub_->publish(msg);
}

void ButtonPressTask::stopRobot() { publishVel(0.0f, 0.0f, 0.0f); }

void ButtonPressTask::start() {
  status_ = TaskStatus::kRunning;
  presses_done_ = 0;
  progress_ = 0.0f;
  start_time_ = node_->now();

  stopRobot();
  enterState(State::kSettle);

  RCLCPP_INFO(node_->get_logger(),
              "ButtonPress: Starting (%d presses, drive-based)", cfg_.presses_total);
}

void ButtonPressTask::step() {
  if (status_ != TaskStatus::kRunning) {
    return;
  }

  float t_state = (node_->now() - state_entry_time_).seconds();
  float t_total = (node_->now() - start_time_).seconds();

  // Timeout check
  if (cfg_.timeout_s > 0.0f && t_total >= cfg_.timeout_s) {
    RCLCPP_WARN(node_->get_logger(), "ButtonPress: Timeout (%.1fs)", t_total);
    stopRobot();
    status_ = TaskStatus::kFailed;
    state_ = State::kDone;
    return;
  }

  // Progress: based on presses done + phase within current cycle
  const float total =
      (cfg_.presses_total == 0) ? 1.0f : static_cast<float>(cfg_.presses_total);
  const float cycle_time =
      cfg_.approach_s + cfg_.hold_s + cfg_.backup_s + cfg_.adjust_s +
      cfg_.between_presses_s;
  float phase_frac = 0.0f;
  if (cycle_time > 0.001f) {
    float elapsed_in_cycle = 0.0f;
    switch (state_) {
      case State::kApproach:
        elapsed_in_cycle = t_state;
        break;
      case State::kHold:
        elapsed_in_cycle = cfg_.approach_s + t_state;
        break;
      case State::kBackup:
        elapsed_in_cycle = cfg_.approach_s + cfg_.hold_s + t_state;
        break;
      case State::kAdjust:
        elapsed_in_cycle =
            cfg_.approach_s + cfg_.hold_s + cfg_.backup_s + t_state;
        break;
      default:
        break;
    }
    phase_frac = clamp01(elapsed_in_cycle / cycle_time) / total;
  }
  progress_ = clamp01((static_cast<float>(presses_done_) / total) + phase_frac);

  // State machine
  switch (state_) {
    case State::kSettle:
      stopRobot();
      if (t_state >= cfg_.settle_s) {
        RCLCPP_INFO(node_->get_logger(), "ButtonPress: Approach #%d",
                    presses_done_ + 1);
        enterState(State::kApproach);
      }
      break;

    case State::kApproach:
      // Drive forward into button
      publishVel(cfg_.approach_speed, 0.0f, 0.0f);
      if (t_state >= cfg_.approach_s) {
        enterState(State::kHold);
      }
      break;

    case State::kHold:
      // Keep gentle pressure on button
      publishVel(cfg_.hold_speed, 0.0f, 0.0f);
      if (t_state >= cfg_.hold_s) {
        RCLCPP_INFO(node_->get_logger(), "ButtonPress: Backing up from press #%d",
                    presses_done_ + 1);
        enterState(State::kBackup);
      }
      break;

    case State::kBackup:
      // Reverse away from button
      publishVel(-cfg_.backup_speed, 0.0f, 0.0f);
      if (t_state >= cfg_.backup_s) {
        presses_done_++;

        if (presses_done_ >= cfg_.presses_total) {
          stopRobot();
          RCLCPP_INFO(node_->get_logger(),
                      "ButtonPress: Completed %d presses!", presses_done_);
          status_ = TaskStatus::kSucceeded;
          progress_ = 1.0f;
          enterState(State::kDone);
        } else {
          enterState(State::kAdjust);
        }
      }
      break;

    case State::kAdjust: {
      // Optional lateral adjustment between presses
      // First part: lateral shift (if configured)
      // Second part: pause before next approach
      float adjust_total = cfg_.adjust_s + cfg_.between_presses_s;

      if (t_state < cfg_.adjust_s && cfg_.adjust_s > 0.001f) {
        // Lateral movement phase
        publishVel(0.0f, cfg_.adjust_lateral_speed, 0.0f);
      } else {
        // Pause phase (stopped, waiting before next approach)
        stopRobot();
      }

      if (t_state >= adjust_total) {
        RCLCPP_INFO(node_->get_logger(), "ButtonPress: Approach #%d",
                    presses_done_ + 1);
        enterState(State::kApproach);
      }
      break;
    }

    case State::kDone:
      // Stay stopped
      break;

    case State::kIdle:
    default:
      status_ = TaskStatus::kFailed;
      break;
  }
}

void ButtonPressTask::cancel() {
  if (status_ == TaskStatus::kRunning) {
    stopRobot();
    status_ = TaskStatus::kFailed;
    state_ = State::kDone;
    RCLCPP_INFO(node_->get_logger(), "ButtonPress: Cancelled");
  }
}

void ButtonPressTask::reset() {
  TaskBase::reset();
  stopRobot();
  state_ = State::kIdle;
  presses_done_ = 0;
}

}  // namespace secbot
