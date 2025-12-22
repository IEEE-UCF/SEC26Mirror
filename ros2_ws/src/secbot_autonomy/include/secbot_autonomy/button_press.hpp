#pragma once
/**
 * @file button_press.hpp
 * @author Rafeed Khan
 * @brief Button press task for Antenna #1: press the button 3 times
 *
 * This task:
 *   - Assumes robot is already aligned to the antenna
 *   - Commands a pusher actuator via ROS topic
 *   - Outputs no drive commands (stationary task)
 *
 * State machine:
 *   IDLE -> SETTLE -> PRESS_HOLD <-> RELEASE_HOLD -> DONE
 */

#include "secbot_autonomy/task_base.hpp"

#include <secbot_msgs/msg/arm_command.hpp>

#include <cstdint>

namespace secbot {

/**
 * @brief Configuration for button press sequence
 */
struct ButtonPressConfig {
  /// Total presses required (3 for Antenna #1 according to the ruleset!!!)
  uint8_t presses_total = 3;

  /// Settle delay before starting (let robot stabilize after docking)
  float settle_s = 0.20f;

  /// How long to hold pusher in pressed position
  float press_hold_s = 0.25f;

  /// How long to hold pusher in released position between presses
  float release_hold_s = 0.25f;

  /// Actuator position for pressing (degrees or ticks, depends on hardware)
  int16_t press_position = 90;

  /// Actuator position for releasing
  int16_t release_position = 0;

  /// Actuator speed (0-255)
  uint8_t actuator_speed = 200;

  /// Joint ID for the pusher actuator
  uint8_t pusher_joint_id = 0;

  /// Safety timeout for entire task
  float timeout_s = 6.0f;

  /// Topic for arm commands
  std::string arm_command_topic = "arm_command";
};

/**
 * @brief Button press task for Antenna #1
 *
 * Presses the button 3 times using a pusher actuator.
 * Robot must already be aligned - this task does not drive.
 */
class ButtonPressTask : public TaskBase {
 public:
  ButtonPressTask(rclcpp::Node::SharedPtr node,
                  const ButtonPressConfig& cfg = ButtonPressConfig{});

  /// Update configuration
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
    kSettle,      ///< Brief pause before starting
    kPressHold,   ///< Pusher extended, holding
    kReleaseHold, ///< Pusher retracted, waiting before next press
    kDone
  };

  void enterState(State s);
  void commandPusher(int16_t position);

  ButtonPressConfig cfg_;
  State state_ = State::kIdle;

  uint8_t presses_done_ = 0;
  float t_state_ = 0.0f;    ///< Time in current state
  rclcpp::Time start_time_;
  rclcpp::Time state_entry_time_;

  // ROS interfaces
  rclcpp::Publisher<secbot_msgs::msg::ArmCommand>::SharedPtr arm_pub_;
};

}  // namespace secbot
