/**
 * @file pressure_clear.cpp
 * @author Rafeed Khan
 * @brief Implementation of pressure clear task for Antenna #3
 */

#include "secbot_autonomy/pressure_clear.hpp"

namespace secbot {

namespace {
float clamp01(float x) {
  if (x < 0.0f) return 0.0f;
  if (x > 1.0f) return 1.0f;
  return x;
}
}  // namespace

PressureClearTask::PressureClearTask(rclcpp::Node::SharedPtr node,
                                     const PressureClearConfig& cfg)
    : TaskBase(node), cfg_(cfg) {
  arm_pub_ = node_->create_publisher<mcu_msgs::msg::ArmCommand>(
      cfg_.arm_command_topic, 10);

  if (cfg_.intake_speed != 0) {
    intake_pub_ =
        node_->create_publisher<std_msgs::msg::Int16>(cfg_.intake_topic, 10);
  }

  captured_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
      cfg_.duck_captured_topic, 10,
      std::bind(&PressureClearTask::onDuckCaptured, this,
                std::placeholders::_1));
}

void PressureClearTask::onDuckCaptured(
    const std_msgs::msg::Bool::SharedPtr msg) {
  duck_captured_ = msg->data;
  sensor_valid_ = true;
}

void PressureClearTask::enterState(State s) {
  state_ = s;
  state_entry_time_ = node_->now();
}

void PressureClearTask::commandSweeper(int16_t position) {
  mcu_msgs::msg::ArmCommand msg;
  msg.joint_id = cfg_.sweeper_joint_id;
  msg.position = position;
  msg.speed = cfg_.actuator_speed;
  arm_pub_->publish(msg);
}

void PressureClearTask::commandIntake(int16_t speed) {
  if (intake_pub_) {
    std_msgs::msg::Int16 msg;
    msg.data = speed;
    intake_pub_->publish(msg);
  }
}

void PressureClearTask::applyOutputs() {
  int16_t sweeper_pos = cfg_.sweeper_safe_pos;
  int16_t intake_spd = 0;

  switch (state_) {
    case State::kIdle:
    case State::kSettle:
    case State::kBetween:
    case State::kRetract:
    case State::kDone:
      sweeper_pos = cfg_.sweeper_safe_pos;
      intake_spd = 0;
      break;

    case State::kPushOut:
    case State::kHold:
      sweeper_pos = cfg_.sweeper_push_pos;
      intake_spd = cfg_.intake_speed;
      break;
  }

  commandSweeper(sweeper_pos);
  commandIntake(intake_spd);
}

void PressureClearTask::start() {
  status_ = TaskStatus::kRunning;
  progress_ = 0.0f;
  sweeps_done_ = 0;
  duck_captured_ = false;
  start_time_ = node_->now();

  enterState(State::kSettle);

  RCLCPP_INFO(node_->get_logger(), "PressureClear: Starting (%d sweeps)",
              cfg_.sweeps);
}

void PressureClearTask::step() {
  if (status_ != TaskStatus::kRunning) {
    return;
  }

  float t_total = (node_->now() - start_time_).seconds();
  float t_state = (node_->now() - state_entry_time_).seconds();

  // Timeout check
  if (cfg_.timeout_s > 0.0f && t_total >= cfg_.timeout_s) {
    RCLCPP_WARN(node_->get_logger(), "PressureClear: Timeout");
    commandSweeper(cfg_.sweeper_safe_pos);
    commandIntake(0);
    status_ = TaskStatus::kFailed;
    state_ = State::kDone;
    return;
  }

  // Progress estimate
  const uint8_t sweeps = (cfg_.sweeps == 0) ? 1 : cfg_.sweeps;
  const float per_sweep = cfg_.push_out_s + cfg_.push_hold_s + cfg_.retract_s;
  const float est_total =
      cfg_.settle_s + (sweeps * per_sweep) +
      ((sweeps > 1) ? ((sweeps - 1) * cfg_.between_sweeps_s) : 0.0f);
  if (est_total > 0.001f) {
    progress_ = clamp01(t_total / est_total);
  }

  // State machine
  switch (state_) {
    case State::kSettle:
      if (t_state >= cfg_.settle_s) {
        enterState(State::kPushOut);
      }
      break;

    case State::kPushOut:
      if (t_state >= cfg_.push_out_s) {
        enterState(State::kHold);
      }
      break;

    case State::kHold:
      // Check for duck captured sensor
      if (sensor_valid_ && duck_captured_) {
        if (t_state >= cfg_.capture_wait_s) {
          enterState(State::kRetract);
        }
      } else {
        if (t_state >= cfg_.push_hold_s) {
          enterState(State::kRetract);
        }
      }
      break;

    case State::kRetract:
      if (t_state >= cfg_.retract_s) {
        sweeps_done_++;

        if (sweeps_done_ >= cfg_.sweeps) {
          status_ = TaskStatus::kSucceeded;
          progress_ = 1.0f;
          state_ = State::kDone;
          RCLCPP_INFO(node_->get_logger(), "PressureClear: Complete!");
        } else {
          enterState(State::kBetween);
        }
      }
      break;

    case State::kBetween:
      if (t_state >= cfg_.between_sweeps_s) {
        enterState(State::kPushOut);
      }
      break;

    case State::kDone:
    case State::kIdle:
    default:
      break;
  }

  applyOutputs();
}

void PressureClearTask::cancel() {
  if (status_ == TaskStatus::kRunning) {
    commandSweeper(cfg_.sweeper_safe_pos);
    commandIntake(0);
    status_ = TaskStatus::kFailed;
    state_ = State::kDone;
    RCLCPP_INFO(node_->get_logger(), "PressureClear: Cancelled");
  }
}

void PressureClearTask::reset() {
  TaskBase::reset();
  commandSweeper(cfg_.sweeper_safe_pos);
  commandIntake(0);
  state_ = State::kIdle;
  sweeps_done_ = 0;
  duck_captured_ = false;
}

}  // namespace secbot
