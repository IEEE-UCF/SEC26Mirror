#pragma once
/**
 * @file crater_entry.hpp
 * @author Rafeed Khan
 * @brief Crater entry task: drive into crater, touch the line, reverse out
 *
 * This task:
 *   - Approaches the crater rim
 *   - Descends until drive mechanism touches the crater line (around 3" down)
 *   - Dwells briefly for contact/settling
 *   - Reverses out of the crater
 *
 * State machine:
 *   IDLE -> APPROACH_RIM -> DESCEND_TO_LINE -> DWELL -> EXIT_CRATER -> DONE
 *
 * Motion is commanded via cmd_vel, distance tracked via odometry
 */

#include "secbot_autonomy/task_base.hpp"

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>

#include <cstdint>

namespace secbot {

/**
 * @brief Configuration for crater entry task
 */
struct CraterEntryConfig {
  /// Forward distance to reach rim staging point (meters)
  float approach_distance_m = 0.25f;

  /// Forward distance into crater to touch line (meters)
  float descend_distance_m = 0.35f;

  /// Reverse distance to exit crater (meters)
  float exit_distance_m = 0.60f;

  /// Dwell time on the line (seconds)
  float dwell_time_s = 0.25f;

  /// Safety timeout (seconds)
  float timeout_s = 8.0f;

  /// Drive speeds (m/s)
  float approach_speed = 0.15f;
  float descend_speed = 0.10f;
  float exit_speed = 0.15f;

  /// Distance tolerance for phase completion (meters)
  float distance_tolerance_m = 0.02f;

  /// If true, fail immediately on motion errors
  bool fail_fast = true;

  /// Topic names
  std::string cmd_vel_topic = "cmd_vel";
  std::string pose_topic = "odom_pose";
};

/**
 * @brief Crater entry task
 *
 * Drives into the crater to touch the scoring line, then reverses out
 */
class CraterEntryTask : public TaskBase {
 public:
  CraterEntryTask(rclcpp::Node::SharedPtr node,
                  const CraterEntryConfig& cfg = CraterEntryConfig{});

  /// Update configuration
  void setConfig(const CraterEntryConfig& cfg) { cfg_ = cfg; }

  // TaskBase interface
  void start() override;
  void step() override;
  void cancel() override;
  void reset() override;
  std::string name() const override { return "crater_entry"; }

 private:
  enum class State : uint8_t {
    kIdle = 0,
    kApproachRim,     ///< Drive forward to rim
    kDescendToLine,   ///< Drive forward into crater
    kDwellOnLine,     ///< Pause for contact
    kExitCrater,      ///< Reverse out
    kDone
  };

  void enterState(State s);
  void publishVelocity(float linear, float angular);
  void stopMotion();
  float distanceFromPhaseStart() const;

  // Pose callback
  void onPose(const geometry_msgs::msg::Pose2D::SharedPtr msg);

  CraterEntryConfig cfg_;
  State state_ = State::kIdle;

  rclcpp::Time start_time_;
  rclcpp::Time state_entry_time_;

  // Pose tracking
  float current_x_ = 0.0f;
  float current_y_ = 0.0f;
  float current_theta_ = 0.0f;
  bool pose_valid_ = false;

  // Phase start pose (for distance tracking)
  float phase_start_x_ = 0.0f;
  float phase_start_y_ = 0.0f;

  // ROS interfaces
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr pose_sub_;
};

}  // namespace secbot
