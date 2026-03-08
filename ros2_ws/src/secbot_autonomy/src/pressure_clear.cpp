/**
 * @file pressure_clear.cpp
 * @author Rafeed Khan
 * @brief Implementation of pressure clear task using intake rail
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
  intake_cmd_pub_ = node_->create_publisher<mcu_msgs::msg::IntakeCommand>(
      cfg_.intake_command_topic, 10);

  intake_state_sub_ =
      node_->create_subscription<mcu_msgs::msg::IntakeState>(
          cfg_.intake_state_topic, 10,
          std::bind(&PressureClearTask::onIntakeState, this,
                    std::placeholders::_1));
}

void PressureClearTask::onIntakeState(
    const mcu_msgs::msg::IntakeState::SharedPtr msg) {
  intake_position_ = msg->position;
  intake_limit_extend_ = msg->limit_extend_active;
  intake_limit_retract_ = msg->limit_retract_active;
}

void PressureClearTask::enterState(State s) {
  state_ = s;
  state_entry_time_ = node_->now();
}

void PressureClearTask::sendIntakeCommand(uint8_t cmd, float value) {
  mcu_msgs::msg::IntakeCommand msg;
  msg.command = cmd;
  msg.value = value;
  intake_cmd_pub_->publish(msg);
}

void PressureClearTask::start() {
  status_ = TaskStatus::kRunning;
  progress_ = 0.0f;
  start_time_ = node_->now();

  enterState(State::kSettle);

  RCLCPP_INFO(node_->get_logger(),
              "PressureClear: Starting (intake rail mode)");
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
    sendIntakeCommand(mcu_msgs::msg::IntakeCommand::CMD_STOP_RAIL);
    sendIntakeCommand(mcu_msgs::msg::IntakeCommand::CMD_STOP_INTAKE);
    status_ = TaskStatus::kFailed;
    state_ = State::kDone;
    return;
  }

  // Progress estimate
  const float est_total = cfg_.settle_s + cfg_.extend_timeout_s +
                          cfg_.capture_timeout_s + cfg_.retract_timeout_s;
  if (est_total > 0.001f) {
    progress_ = clamp01(t_total / est_total);
  }

  // State machine
  switch (state_) {
    case State::kSettle:
      if (t_state >= cfg_.settle_s) {
        enterState(State::kExtendRail);
        sendIntakeCommand(mcu_msgs::msg::IntakeCommand::CMD_EXTEND);
        sendIntakeCommand(mcu_msgs::msg::IntakeCommand::CMD_SET_INTAKE_SPEED,
                          cfg_.intake_speed);
        RCLCPP_INFO(node_->get_logger(),
                    "PressureClear: Extending rail, intake on");
      }
      break;

    case State::kExtendRail:
      if (intake_position_ >= 0.95f || intake_limit_extend_) {
        enterState(State::kWaitCapture);
        RCLCPP_INFO(node_->get_logger(),
                    "PressureClear: Rail extended, waiting for duck capture");
      } else if (t_state >= cfg_.extend_timeout_s) {
        RCLCPP_WARN(node_->get_logger(),
                    "PressureClear: Rail extend timeout, proceeding");
        enterState(State::kWaitCapture);
      }
      break;

    case State::kWaitCapture:
      if (t_state >= cfg_.capture_timeout_s) {
        enterState(State::kRetractRail);
        sendIntakeCommand(mcu_msgs::msg::IntakeCommand::CMD_RETRACT);
        RCLCPP_INFO(node_->get_logger(),
                    "PressureClear: Capture window elapsed, retracting rail");
      }
      break;

    case State::kRetractRail:
      if (intake_position_ <= 0.05f || intake_limit_retract_) {
        sendIntakeCommand(mcu_msgs::msg::IntakeCommand::CMD_STOP_INTAKE);
        sendIntakeCommand(mcu_msgs::msg::IntakeCommand::CMD_STOP_RAIL);
        status_ = TaskStatus::kSucceeded;
        progress_ = 1.0f;
        state_ = State::kDone;
        RCLCPP_INFO(node_->get_logger(), "PressureClear: Complete!");
      } else if (t_state >= cfg_.retract_timeout_s) {
        sendIntakeCommand(mcu_msgs::msg::IntakeCommand::CMD_STOP_INTAKE);
        sendIntakeCommand(mcu_msgs::msg::IntakeCommand::CMD_STOP_RAIL);
        status_ = TaskStatus::kSucceeded;
        progress_ = 1.0f;
        state_ = State::kDone;
        RCLCPP_WARN(node_->get_logger(),
                    "PressureClear: Retract timeout, marking complete");
      }
      break;

    case State::kDone:
    case State::kIdle:
    default:
      break;
  }
}

void PressureClearTask::cancel() {
  if (status_ == TaskStatus::kRunning) {
    sendIntakeCommand(mcu_msgs::msg::IntakeCommand::CMD_STOP_RAIL);
    sendIntakeCommand(mcu_msgs::msg::IntakeCommand::CMD_STOP_INTAKE);
    status_ = TaskStatus::kFailed;
    state_ = State::kDone;
    RCLCPP_INFO(node_->get_logger(), "PressureClear: Cancelled");
  }
}

void PressureClearTask::reset() {
  TaskBase::reset();
  sendIntakeCommand(mcu_msgs::msg::IntakeCommand::CMD_STOP_RAIL);
  sendIntakeCommand(mcu_msgs::msg::IntakeCommand::CMD_STOP_INTAKE);
  state_ = State::kIdle;
}

}  // namespace secbot
