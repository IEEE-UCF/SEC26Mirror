/**
 * @file flag_plant.cpp
 * @author Rafeed Khan
 * @brief Implementation of flag plant task
 */

#include "secbot_autonomy/flag_plant.hpp"

namespace secbot {

float FlagPlantTask::clamp01(float x) {
  if (x < 0.0f) return 0.0f;
  if (x > 1.0f) return 1.0f;
  return x;
}

FlagPlantTask::FlagPlantTask(rclcpp::Node::SharedPtr node,
                             const FlagPlantConfig& cfg)
    : TaskBase(node), cfg_(cfg) {
  arm_pub_ = node_->create_publisher<secbot_msgs::msg::ArmCommand>(
      cfg_.arm_command_topic, 10);

  if (cfg_.use_flag_sensor) {
    flag_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
        cfg_.flag_present_topic, 10,
        std::bind(&FlagPlantTask::onFlagPresent, this, std::placeholders::_1));
  }
}

void FlagPlantTask::onFlagPresent(const std_msgs::msg::Bool::SharedPtr msg) {
  flag_present_ = msg->data;
  sensor_valid_ = true;
  last_sensor_time_ = node_->now();
}

void FlagPlantTask::enterState(State s) {
  state_ = s;
  state_entry_time_ = node_->now();
  present_stable_t_ = 0.0f;
  last_present_ = true;
}

void FlagPlantTask::commandLatch(int16_t position) {
  secbot_msgs::msg::ArmCommand msg;
  msg.joint_id = cfg_.latch_joint_id;
  msg.position = position;
  msg.speed = cfg_.actuator_speed;
  arm_pub_->publish(msg);
}

void FlagPlantTask::start() {
  status_ = TaskStatus::kRunning;
  progress_ = 0.0f;
  release_confirmed_ = false;
  start_time_ = node_->now();

  // Start with latch closed (much safer!!!)
  commandLatch(cfg_.latch_closed_pos);
  enterState(State::kSettle);

  RCLCPP_INFO(node_->get_logger(), "FlagPlant: Starting");
}

void FlagPlantTask::step() {
  if (status_ != TaskStatus::kRunning) {
    return;
  }

  float t_total = (node_->now() - start_time_).seconds();
  float t_state = (node_->now() - state_entry_time_).seconds();

  // Timeout check
  if (cfg_.timeout_s > 0.0f && t_total >= cfg_.timeout_s) {
    RCLCPP_WARN(node_->get_logger(), "FlagPlant: Timeout");
    commandLatch(cfg_.latch_closed_pos);
    status_ = TaskStatus::kFailed;
    state_ = State::kDone;
    return;
  }

  // Sensor debounce (if enabled of course)
  bool sensor_ok = cfg_.use_flag_sensor && sensor_valid_;
  if (sensor_ok) {
    if (flag_present_ == last_present_) {
      present_stable_t_ += (node_->now() - state_entry_time_).seconds() - t_state;
      // Approximate delta - will be refined each step
      present_stable_t_ = t_state;  // Use state time as proxy
    } else {
      present_stable_t_ = 0.0f;
      last_present_ = flag_present_;
    }
  }

  // Progress calculation
  if (state_ == State::kSettle && cfg_.settle_s > 0.0f) {
    progress_ = 0.10f * clamp01(t_state / cfg_.settle_s);
  } else if (state_ == State::kUnlatchHold && cfg_.unlatch_s > 0.0f) {
    progress_ = 0.10f + 0.70f * clamp01(t_state / cfg_.unlatch_s);
  } else if (state_ == State::kPostDrop && cfg_.post_drop_s > 0.0f) {
    progress_ = 0.80f + 0.20f * clamp01(t_state / cfg_.post_drop_s);
  }

  // State machine
  switch (state_) {
    case State::kSettle:
      // Keep latched while settling
      commandLatch(cfg_.latch_closed_pos);

      if (t_state >= cfg_.settle_s) {
        commandLatch(cfg_.latch_open_pos);
        enterState(State::kUnlatchHold);
        RCLCPP_DEBUG(node_->get_logger(), "FlagPlant: Unlatching");
      }
      break;

    case State::kUnlatchHold:
      // Hold open for flag to fall
      commandLatch(cfg_.latch_open_pos);

      // Early transition if sensor confirms flag is gone
      if (sensor_ok && !flag_present_ && present_stable_t_ >= cfg_.sensor_debounce_s) {
        enterState(State::kPostDrop);
        RCLCPP_DEBUG(node_->get_logger(), "FlagPlant: Flag released (sensor)");
        break;
      }

      if (t_state >= cfg_.unlatch_s) {
        enterState(State::kPostDrop);
      }
      break;

    case State::kPostDrop:
      // Keep open to avoid snagging
      commandLatch(cfg_.latch_open_pos);

      if (t_state >= cfg_.post_drop_s) {
        // Determine success
        if (sensor_ok) {
          release_confirmed_ = !flag_present_ && present_stable_t_ >= cfg_.sensor_debounce_s;
        } else {
          // No sensor - assume success after timing
          release_confirmed_ = true;
        }

        status_ = release_confirmed_ ? TaskStatus::kSucceeded : TaskStatus::kFailed;
        progress_ = 1.0f;
        state_ = State::kDone;

        RCLCPP_INFO(node_->get_logger(), "FlagPlant: %s",
                    release_confirmed_ ? "Complete!" : "Failed (flag still present)");
      }
      break;

    case State::kDone:
      // Stay open (relativelyt harmless lul)
      commandLatch(cfg_.latch_open_pos);
      break;

    case State::kIdle:
    default:
      break;
  }
}

void FlagPlantTask::cancel() {
  if (status_ == TaskStatus::kRunning) {
    commandLatch(cfg_.latch_closed_pos);
    status_ = TaskStatus::kFailed;
    state_ = State::kDone;
    RCLCPP_INFO(node_->get_logger(), "FlagPlant: Cancelled");
  }
}

void FlagPlantTask::reset() {
  TaskBase::reset();
  commandLatch(cfg_.latch_closed_pos);
  state_ = State::kIdle;
  release_confirmed_ = false;
  present_stable_t_ = 0.0f;
  last_present_ = true;
}

}  // namespace secbot
