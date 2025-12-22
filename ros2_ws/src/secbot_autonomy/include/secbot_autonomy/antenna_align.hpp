#pragma once
/**
 * @file antenna_align.hpp
 * @author Rafeed Khan
 * @brief Antenna alignment task: approach and dock to an antenna face
 *
 * This task uses the ApproachTarget action to navigate to an antenna 
 * and align with the correct face for interaction
 *
 * State machine:
 *   IDLE -> APPROACHING -> ALIGNING -> SUCCEEDED/FAILED
 *
 * Motion is delegated to the navigation/approach action server
 * This task only sequences the high-level behavior
 */

#include "secbot_autonomy/task_base.hpp"

#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <secbot_msgs/action/approach_target.hpp>

#include <cstdint>
#include <functional>
#include <memory>

namespace secbot {

/** @brief Cardinal direction of the antenna task face */
enum class AntennaFace : uint8_t {
  kNorth = 0,
  kEast,
  kSouth,
  kWest,
  kUnknown
};

/**
 * @brief Antenna alignment task
 *
 * Approaches an antenna and aligns to the correct face for task interaction
 * Uses ApproachTarget action for navigation
 */
class AntennaAlignTask : public TaskBase {
 public:
  using ApproachTarget = secbot_msgs::action::ApproachTarget;
  using GoalHandle = rclcpp_action::ClientGoalHandle<ApproachTarget>;

  struct Config {
    // Tolerances for final alignment
    float yaw_tolerance_rad = 0.10f;   ///< around 5.7 degrees
    float position_tolerance_m = 0.03f;

    // Timeout for entire task
    float timeout_s = 15.0f;

    // Topic names
    std::string cmd_vel_topic = "cmd_vel";
    std::string pose_topic = "odom_pose";
    std::string approach_action = "approach_target";
  };

  AntennaAlignTask(rclcpp::Node::SharedPtr node, const Config& cfg = Config{});

  /**
   * @brief Configure which antenna to approach
   * @param antenna_id 1..4 per competition rules
   */
  void setTarget(uint8_t antenna_id);

  // TaskBase interface
  void start() override;
  void step() override;
  void cancel() override;
  void reset() override;
  std::string name() const override { return "antenna_align"; }

  // Feedback
  float distanceRemaining() const { return distance_remaining_; }
  float headingError() const { return heading_error_; }
  uint8_t targetAntennaId() const { return antenna_id_; }

 private:
  enum class State : uint8_t {
    kIdle = 0,
    kApproaching,   ///< Navigating to antenna standoff point
    kAligning,      ///< Fine-tuning heading to face antenna
    kDone
  };

  // Rules-based mapping
  static AntennaFace faceForId(uint8_t antenna_id);
  static float headingForFace(AntennaFace face);

  // State machine helpers
  void startApproach();
  void startAlign();
  void publishTwist(float linear, float angular);
  void stopMotion();

  // Action callbacks
  void onApproachResult(const GoalHandle::WrappedResult& result);
  void onApproachFeedback(
      GoalHandle::SharedPtr,
      const std::shared_ptr<const ApproachTarget::Feedback> feedback);

  // Pose callback
  void onPose(const geometry_msgs::msg::Pose2D::SharedPtr msg);

  // Utility
  static float wrapAngle(float angle);
  static float clamp(float val, float lo, float hi);

  Config cfg_;
  State state_ = State::kIdle;

  // Target
  uint8_t antenna_id_ = 0;
  float target_heading_ = 0.0f;

  // Current pose (from subscription)
  float current_x_ = 0.0f;
  float current_y_ = 0.0f;
  float current_theta_ = 0.0f;
  bool pose_valid_ = false;

  // Timing
  rclcpp::Time start_time_;
  bool timed_out_ = false;

  // Feedback
  float distance_remaining_ = 0.0f;
  float heading_error_ = 0.0f;

  // Action state
  bool approach_done_ = false;
  bool approach_success_ = false;

  // ROS interfaces
  rclcpp_action::Client<ApproachTarget>::SharedPtr approach_client_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr pose_sub_;
  GoalHandle::SharedPtr goal_handle_;
};

}  // namespace secbot
