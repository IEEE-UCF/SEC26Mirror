/**
 * @file antenna_align.cpp
 * @author Rafeed Khan
 * @brief Implementation of antenna alignment task
 */

#include "secbot_autonomy/antenna_align.hpp"

#include <cmath>

namespace secbot {

namespace {
constexpr float kPi = 3.14159265358979323846f;

// Alignment control gains (for fine heading adjustment)
constexpr float kAlignKp = 1.5f;           // P gain for angular velocity
constexpr float kMaxAngularVel = 0.5f;     // rad/s
constexpr float kMinAngularVel = 0.08f;    // rad/s, minimum to overcome friction
}  // namespace

// Static helpers

float AntennaAlignTask::wrapAngle(float angle) {
  while (angle <= -kPi) angle += 2.0f * kPi;
  while (angle > kPi) angle -= 2.0f * kPi;
  return angle;
}

float AntennaAlignTask::clamp(float val, float lo, float hi) {
  return (val < lo) ? lo : (val > hi) ? hi : val;
}

AntennaFace AntennaAlignTask::faceForId(uint8_t antenna_id) {
  switch (antenna_id) {
    case 1: return AntennaFace::kSouth;
    case 2: return AntennaFace::kSouth;
    case 3: return AntennaFace::kWest;
    case 4: return AntennaFace::kNorth;
    default: return AntennaFace::kUnknown;
  }
}

float AntennaAlignTask::headingForFace(AntennaFace face) {
  // Robot heading to face the antenna (opposite of face direction)
  // Face points outward from antenna, robot should face inward
  switch (face) {
    case AntennaFace::kNorth: return -0.5f * kPi;  // Robot faces south
    case AntennaFace::kSouth: return 0.5f * kPi;   // Robot faces north
    case AntennaFace::kEast:  return kPi;          // Robot faces west
    case AntennaFace::kWest:  return 0.0f;         // Robot faces east
    default: return 0.0f;
  }
}

// Constructor

AntennaAlignTask::AntennaAlignTask(rclcpp::Node::SharedPtr node, const Config& cfg)
    : TaskBase(node), cfg_(cfg) {
  // Create action client for approach
  approach_client_ = rclcpp_action::create_client<ApproachTarget>(
      node_, cfg_.approach_action);

  // Publisher for fine alignment commands
  cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(
      cfg_.cmd_vel_topic, 10);

  // Subscribe to pose for alignment feedback
  pose_sub_ = node_->create_subscription<geometry_msgs::msg::Pose2D>(
      cfg_.pose_topic, 10,
      std::bind(&AntennaAlignTask::onPose, this, std::placeholders::_1));
}

// Configuration

void AntennaAlignTask::setTarget(uint8_t antenna_id) {
  antenna_id_ = antenna_id;
  AntennaFace face = faceForId(antenna_id);
  target_heading_ = headingForFace(face);
}

// TaskBase interface

void AntennaAlignTask::start() {
  if (antenna_id_ == 0) {
    RCLCPP_ERROR(node_->get_logger(), "AntennaAlign: No target set");
    status_ = TaskStatus::kFailed;
    return;
  }

  // Check action server availability
  if (!approach_client_->wait_for_action_server(std::chrono::seconds(1))) {
    RCLCPP_ERROR(node_->get_logger(),
        "AntennaAlign: ApproachTarget action server not available");
    status_ = TaskStatus::kFailed;
    return;
  }

  status_ = TaskStatus::kRunning;
  state_ = State::kApproaching;
  start_time_ = node_->now();
  timed_out_ = false;
  progress_ = 0.0f;
  distance_remaining_ = 0.0f;
  heading_error_ = 0.0f;
  approach_done_ = false;
  approach_success_ = false;

  RCLCPP_INFO(node_->get_logger(),
      "AntennaAlign: Starting approach to antenna %d", antenna_id_);

  startApproach();
}

void AntennaAlignTask::step() {
  if (status_ != TaskStatus::kRunning) {
    return;
  }

  // Check timeout
  auto elapsed = (node_->now() - start_time_).seconds();
  if (elapsed > cfg_.timeout_s) {
    RCLCPP_WARN(node_->get_logger(), "AntennaAlign: Timeout");
    timed_out_ = true;
    stopMotion();
    status_ = TaskStatus::kFailed;
    state_ = State::kDone;
    return;
  }

  switch (state_) {
    case State::kApproaching:
      // Waiting for approach action to complete
      if (approach_done_) {
        if (approach_success_) {
          RCLCPP_INFO(node_->get_logger(), "AntennaAlign: Approach complete, aligning");
          state_ = State::kAligning;
          progress_ = 0.7f;
        } else {
          RCLCPP_ERROR(node_->get_logger(), "AntennaAlign: Approach failed");
          status_ = TaskStatus::kFailed;
          state_ = State::kDone;
        }
      }
      break;

    case State::kAligning:
      if (!pose_valid_) {
        // Wait for pose data
        return;
      }

      heading_error_ = wrapAngle(target_heading_ - current_theta_);

      if (std::fabs(heading_error_) <= cfg_.yaw_tolerance_rad) {
        // Aligned!
        stopMotion();
        RCLCPP_INFO(node_->get_logger(), "AntennaAlign: Alignment complete");
        status_ = TaskStatus::kSucceeded;
        state_ = State::kDone;
        progress_ = 1.0f;
      } else {
        // P controller for angular velocity
        float angular_cmd = kAlignKp * heading_error_;
        angular_cmd = clamp(angular_cmd, -kMaxAngularVel, kMaxAngularVel);

        // Apply minimum velocity to overcome friction
        if (angular_cmd != 0.0f && std::fabs(angular_cmd) < kMinAngularVel) {
          angular_cmd = (angular_cmd > 0) ? kMinAngularVel : -kMinAngularVel;
        }

        publishTwist(0.0f, angular_cmd);
        progress_ = 0.7f + 0.3f * (1.0f - std::fabs(heading_error_) / kPi);
      }
      break;

    case State::kIdle:
    case State::kDone:
      // Nothing to do
      break;
  }
}

void AntennaAlignTask::cancel() {
  if (status_ == TaskStatus::kRunning) {
    // Cancel approach action if active
    if (goal_handle_ && state_ == State::kApproaching) {
      approach_client_->async_cancel_goal(goal_handle_);
    }
    stopMotion();
    status_ = TaskStatus::kFailed;
    state_ = State::kDone;
    RCLCPP_INFO(node_->get_logger(), "AntennaAlign: Cancelled");
  }
}

void AntennaAlignTask::reset() {
  TaskBase::reset();
  state_ = State::kIdle;
  antenna_id_ = 0;
  target_heading_ = 0.0f;
  distance_remaining_ = 0.0f;
  heading_error_ = 0.0f;
  approach_done_ = false;
  approach_success_ = false;
  goal_handle_ = nullptr;
}

// State machine helpers

void AntennaAlignTask::startApproach() {
  auto goal_msg = ApproachTarget::Goal();
  goal_msg.antenna_id = antenna_id_;

  auto send_goal_options = rclcpp_action::Client<ApproachTarget>::SendGoalOptions();

  send_goal_options.result_callback =
      std::bind(&AntennaAlignTask::onApproachResult, this, std::placeholders::_1);

  send_goal_options.feedback_callback =
      std::bind(&AntennaAlignTask::onApproachFeedback, this,
                std::placeholders::_1, std::placeholders::_2);

  auto future = approach_client_->async_send_goal(goal_msg, send_goal_options);
}

void AntennaAlignTask::startAlign() {
  // Alignment is handled in step() via P controller
}

void AntennaAlignTask::publishTwist(float linear, float angular) {
  geometry_msgs::msg::Twist msg;
  msg.linear.x = linear;
  msg.angular.z = angular;
  cmd_vel_pub_->publish(msg);
}

void AntennaAlignTask::stopMotion() {
  publishTwist(0.0f, 0.0f);
}

// Callbacks

void AntennaAlignTask::onApproachResult(const GoalHandle::WrappedResult& result) {
  approach_done_ = true;
  approach_success_ = (result.code == rclcpp_action::ResultCode::SUCCEEDED)
                      && result.result->success;
}

void AntennaAlignTask::onApproachFeedback(
    GoalHandle::SharedPtr,
    const std::shared_ptr<const ApproachTarget::Feedback> feedback) {
  distance_remaining_ = feedback->distance_remaining;
  // Update progress based on approach (0-70% of total)
  // Assuming initial distance is around 1m max
  float approach_progress = 1.0f - clamp(distance_remaining_, 0.0f, 1.0f);
  progress_ = 0.7f * approach_progress;
}

void AntennaAlignTask::onPose(const geometry_msgs::msg::Pose2D::SharedPtr msg) {
  current_x_ = msg->x;
  current_y_ = msg->y;
  current_theta_ = msg->theta;
  pose_valid_ = true;
}

}  // namespace secbot
