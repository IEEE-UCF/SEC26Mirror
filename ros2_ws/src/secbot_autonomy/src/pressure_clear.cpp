/**
 * @file pressure_clear.cpp
 * @author Rafeed Khan
 * @brief Implementation of pressure clear task using intake bridge
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
  bridge_cmd_pub_ = node_->create_publisher<mcu_msgs::msg::IntakeBridgeCommand>(
      cfg_.bridge_command_topic, 10);

  if (cfg_.intake_speed != 0) {
    intake_pub_ =
        node_->create_publisher<std_msgs::msg::Int16>(cfg_.intake_topic, 10);
  }

  bridge_state_sub_ =
      node_->create_subscription<mcu_msgs::msg::IntakeBridgeState>(
          cfg_.bridge_state_topic, 10,
          std::bind(&PressureClearTask::onBridgeState, this,
                    std::placeholders::_1));
}

void PressureClearTask::onBridgeState(
    const mcu_msgs::msg::IntakeBridgeState::SharedPtr msg) {
  bridge_state_ = msg->state;
  bridge_duck_detected_ = msg->duck_detected;
}

void PressureClearTask::enterState(State s) {
  state_ = s;
  state_entry_time_ = node_->now();
}

void PressureClearTask::sendBridgeCommand(uint8_t cmd) {
  mcu_msgs::msg::IntakeBridgeCommand msg;
  msg.command = cmd;
  bridge_cmd_pub_->publish(msg);
}

void PressureClearTask::commandIntake(int16_t speed) {
  if (intake_pub_) {
    std_msgs::msg::Int16 msg;
    msg.data = speed;
    intake_pub_->publish(msg);
  }
}

void PressureClearTask::start() {
  status_ = TaskStatus::kRunning;
  progress_ = 0.0f;
  bridge_duck_detected_ = false;
  start_time_ = node_->now();

  enterState(State::kSettle);

  RCLCPP_INFO(node_->get_logger(),
              "PressureClear: Starting (intake bridge mode)");
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
    sendBridgeCommand(mcu_msgs::msg::IntakeBridgeCommand::CMD_STOW);
    commandIntake(0);
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
        enterState(State::kExtendBridge);
        sendBridgeCommand(mcu_msgs::msg::IntakeBridgeCommand::CMD_EXTEND);
        commandIntake(cfg_.intake_speed);
        RCLCPP_INFO(node_->get_logger(),
                    "PressureClear: Extending bridge, intake on");
      }
      break;

    case State::kExtendBridge:
      // Wait for bridge to report EXTENDED or timeout
      if (bridge_state_ == mcu_msgs::msg::IntakeBridgeState::STATE_EXTENDED) {
        enterState(State::kWaitCapture);
        RCLCPP_INFO(node_->get_logger(),
                    "PressureClear: Bridge extended, waiting for duck");
      } else if (t_state >= cfg_.extend_timeout_s) {
        // Timeout: bridge didn't fully extend, proceed anyway
        RCLCPP_WARN(node_->get_logger(),
                    "PressureClear: Bridge extend timeout, proceeding");
        enterState(State::kWaitCapture);
      }
      break;

    case State::kWaitCapture:
      // Wait for duck detection from TOF or timeout
      if (bridge_duck_detected_) {
        // Duck detected, brief extra wait then retract
        if (t_state >= cfg_.capture_wait_s) {
          enterState(State::kRetractBridge);
          sendBridgeCommand(mcu_msgs::msg::IntakeBridgeCommand::CMD_RETRACT);
          RCLCPP_INFO(node_->get_logger(),
                      "PressureClear: Duck captured! Retracting bridge");
        }
      } else if (t_state >= cfg_.capture_timeout_s) {
        // Timeout: no duck detected, retract anyway
        enterState(State::kRetractBridge);
        sendBridgeCommand(mcu_msgs::msg::IntakeBridgeCommand::CMD_RETRACT);
        RCLCPP_WARN(node_->get_logger(),
                    "PressureClear: Capture timeout, retracting");
      }
      break;

    case State::kRetractBridge:
      // Wait for bridge to report STOWED or timeout
      if (bridge_state_ == mcu_msgs::msg::IntakeBridgeState::STATE_STOWED) {
        commandIntake(0);
        status_ = TaskStatus::kSucceeded;
        progress_ = 1.0f;
        state_ = State::kDone;
        RCLCPP_INFO(node_->get_logger(), "PressureClear: Complete!");
      } else if (t_state >= cfg_.retract_timeout_s) {
        commandIntake(0);
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
    sendBridgeCommand(mcu_msgs::msg::IntakeBridgeCommand::CMD_STOW);
    commandIntake(0);
    status_ = TaskStatus::kFailed;
    state_ = State::kDone;
    RCLCPP_INFO(node_->get_logger(), "PressureClear: Cancelled");
  }
}

void PressureClearTask::reset() {
  TaskBase::reset();
  sendBridgeCommand(mcu_msgs::msg::IntakeBridgeCommand::CMD_STOW);
  commandIntake(0);
  state_ = State::kIdle;
  bridge_duck_detected_ = false;
}

}  // namespace secbot
