/**
 * @file crater_entry.cpp
 * @author Rafeed Khan
 * @brief Implementation of crater entry task
 */

#include "secbot_autonomy/crater_entry.hpp"

#include <cmath>

namespace secbot {

namespace {
float clamp01(float x) {
  if (x < 0.0f) return 0.0f;
  if (x > 1.0f) return 1.0f;
  return x;
}
}  // namespace

CraterEntryTask::CraterEntryTask(rclcpp::Node::SharedPtr node,
                                 const CraterEntryConfig& cfg)
    : TaskBase(node), cfg_(cfg) {
  cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(
      cfg_.cmd_vel_topic, 10);

  pose_sub_ = node_->create_subscription<geometry_msgs::msg::Pose2D>(
      cfg_.pose_topic, 10,
      std::bind(&CraterEntryTask::onPose, this, std::placeholders::_1));
}

void CraterEntryTask::onPose(const geometry_msgs::msg::Pose2D::SharedPtr msg) {
  current_x_ = msg->x;
  current_y_ = msg->y;
  current_theta_ = msg->theta;
  pose_valid_ = true;
}

void CraterEntryTask::enterState(State s) {
  state_ = s;
  state_entry_time_ = node_->now();

  // Capture phase start position for distance tracking
  phase_start_x_ = current_x_;
  phase_start_y_ = current_y_;
}

void CraterEntryTask::publishVelocity(float linear, float angular) {
  geometry_msgs::msg::Twist msg;
  msg.linear.x = linear;
  msg.angular.z = angular;
  cmd_vel_pub_->publish(msg);
}

void CraterEntryTask::stopMotion() { publishVelocity(0.0f, 0.0f); }

float CraterEntryTask::distanceFromPhaseStart() const {
  float dx = current_x_ - phase_start_x_;
  float dy = current_y_ - phase_start_y_;
  return std::sqrt(dx * dx + dy * dy);
}

void CraterEntryTask::start() {
  if (!pose_valid_) {
    RCLCPP_WARN(node_->get_logger(),
                "CraterEntry: No pose data yet, starting anyway");
  }

  status_ = TaskStatus::kRunning;
  progress_ = 0.0f;
  start_time_ = node_->now();

  enterState(State::kApproachRim);

  RCLCPP_INFO(node_->get_logger(), "CraterEntry: Starting");
}

void CraterEntryTask::step() {
  if (status_ != TaskStatus::kRunning) {
    return;
  }

  if (!pose_valid_) {
    // Wait for pose data
    return;
  }

  float t_total = (node_->now() - start_time_).seconds();
  float t_state = (node_->now() - state_entry_time_).seconds();

  // Timeout check
  if (cfg_.timeout_s > 0.0f && t_total > cfg_.timeout_s) {
    RCLCPP_WARN(node_->get_logger(), "CraterEntry: Timeout");
    stopMotion();
    status_ = TaskStatus::kFailed;
    state_ = State::kDone;
    return;
  }

  float dist = distanceFromPhaseStart();

  switch (state_) {
    case State::kApproachRim: {
      // Drive forward to rim
      progress_ = 0.33f * clamp01(dist / cfg_.approach_distance_m);

      if (dist >= cfg_.approach_distance_m - cfg_.distance_tolerance_m) {
        stopMotion();
        enterState(State::kDescendToLine);
        RCLCPP_DEBUG(node_->get_logger(),
                     "CraterEntry: Reached rim, descending");
      } else {
        publishVelocity(cfg_.approach_speed, 0.0f);
      }
    } break;

    case State::kDescendToLine: {
      // Drive forward into crater
      progress_ = 0.33f + 0.33f * clamp01(dist / cfg_.descend_distance_m);

      if (dist >= cfg_.descend_distance_m - cfg_.distance_tolerance_m) {
        stopMotion();
        enterState(State::kDwellOnLine);
        RCLCPP_DEBUG(node_->get_logger(),
                     "CraterEntry: Touched line, dwelling");
      } else {
        publishVelocity(cfg_.descend_speed, 0.0f);
      }
    } break;

    case State::kDwellOnLine: {
      // Stop and dwell
      progress_ = 0.66f;
      stopMotion();

      if (t_state >= cfg_.dwell_time_s) {
        enterState(State::kExitCrater);
        RCLCPP_DEBUG(node_->get_logger(),
                     "CraterEntry: Dwell complete, exiting");
      }
    } break;

    case State::kExitCrater: {
      // Reverse out
      progress_ = 0.66f + 0.34f * clamp01(dist / cfg_.exit_distance_m);

      if (dist >= cfg_.exit_distance_m - cfg_.distance_tolerance_m) {
        stopMotion();
        status_ = TaskStatus::kSucceeded;
        progress_ = 1.0f;
        state_ = State::kDone;
        RCLCPP_INFO(node_->get_logger(), "CraterEntry: Complete!");
      } else {
        publishVelocity(-cfg_.exit_speed, 0.0f);  // Negative = reverse
      }
    } break;

    case State::kDone:
    case State::kIdle:
    default:
      stopMotion();
      break;
  }
}

void CraterEntryTask::cancel() {
  if (status_ == TaskStatus::kRunning) {
    stopMotion();
    status_ = TaskStatus::kFailed;
    state_ = State::kDone;
    RCLCPP_INFO(node_->get_logger(), "CraterEntry: Cancelled");
  }
}

void CraterEntryTask::reset() {
  TaskBase::reset();
  stopMotion();
  state_ = State::kIdle;
  phase_start_x_ = 0.0f;
  phase_start_y_ = 0.0f;
}

}  // namespace secbot
