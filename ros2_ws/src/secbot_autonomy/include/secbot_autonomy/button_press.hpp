#pragma once
/**
 * @file button_press.hpp
 * @author Rafeed Khan
 * @brief Button press task for Antenna #1: drive into button repeatedly
 *
 * This task drives the robot forward into the button, backs up, and repeats.
 * All distances, speeds, and offsets are configurable so the team can dial
 * in the exact approach on the real field.
 *
 * State machine:
 *   IDLE -> SETTLE -> APPROACH -> HOLD -> BACKUP -> ADJUST -> (repeat or DONE)
 *
 * Each press cycle:
 *   1. APPROACH: drive forward at approach_speed for approach_s seconds
 *   2. HOLD: hold against button for hold_s seconds (keeps pushing)
 *   3. BACKUP: reverse at backup_speed for backup_s seconds
 *   4. ADJUST: optional lateral shift (left/right) to hit button from slightly
 *              different angle, then pause before next press
 */

#include <cstdint>
#include <geometry_msgs/msg/twist.hpp>

#include "secbot_autonomy/task_base.hpp"

namespace secbot {

/**
 * @brief Configuration for drive-based button press sequence
 *
 * All speeds are in m/s, all times are in seconds.
 * Tune these on the real field -- they control exactly how the robot
 * approaches, presses, and backs up from the button.
 */
struct ButtonPressConfig {
  /// Total presses required (3 for Antenna #1 per ruleset)
  uint8_t presses_total = 3;

  /// Settle delay before first approach (let robot stabilize after nav)
  float settle_s = 0.30f;

  // -- Approach phase --
  /// Forward speed when driving into button (m/s)
  float approach_speed = 0.15f;
  /// How long to drive forward into button (seconds)
  float approach_s = 1.0f;

  // -- Hold phase --
  /// Speed while holding against button (low push, keeps contact)
  float hold_speed = 0.05f;
  /// How long to hold against button (seconds)
  float hold_s = 0.3f;

  // -- Backup phase --
  /// Reverse speed when backing up (m/s, positive value -- will be negated)
  float backup_speed = 0.20f;
  /// How long to back up (seconds)
  float backup_s = 0.8f;

  // -- Adjust phase (lateral correction between presses) --
  /// Lateral speed for adjustment (m/s, positive = left, negative = right)
  /// Set to 0 to disable lateral adjustment
  float adjust_lateral_speed = 0.0f;
  /// How long to apply lateral adjustment (seconds)
  float adjust_s = 0.0f;
  /// Pause after adjustment before next approach (seconds)
  float between_presses_s = 0.3f;

  /// Safety timeout for entire task (seconds)
  float timeout_s = 15.0f;

  /// Topic for velocity commands
  std::string cmd_vel_topic = "/cmd_vel";
};

/**
 * @brief Button press task for Antenna #1
 *
 * Drives the robot into the button repeatedly.
 * Robot must already be roughly facing the button.
 */
class ButtonPressTask : public TaskBase {
 public:
  ButtonPressTask(rclcpp::Node::SharedPtr node,
                  const ButtonPressConfig& cfg = ButtonPressConfig{});

  /// Update configuration (call before start)
  void setConfig(const ButtonPressConfig& cfg) { cfg_ = cfg; }

  // TaskBase interface
  void start() override;
  void step() override;
  void cancel() override;
  void reset() override;
  std::string name() const override { return "button_press"; }

  // Feedback
  uint8_t pressesCompleted() const { return presses_done_; }
  uint8_t pressesTotal() const { return cfg_.presses_total; }

 private:
  enum class State : uint8_t {
    kIdle = 0,
    kSettle,    ///< Brief pause before starting
    kApproach,  ///< Driving forward into button
    kHold,      ///< Holding against button
    kBackup,    ///< Reversing away from button
    kAdjust,    ///< Optional lateral shift + pause between presses
    kDone
  };

  void enterState(State s);
  void publishVel(float vx, float vy, float wz);
  void stopRobot();

  ButtonPressConfig cfg_;
  State state_ = State::kIdle;

  uint8_t presses_done_ = 0;
  rclcpp::Time start_time_;
  rclcpp::Time state_entry_time_;

  // ROS interfaces
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
};

}  // namespace secbot
