#pragma once
/**
 * @file keypad_enter.hpp
 * @author Rafeed Khan
 * @brief Keypad entry task for Antenna #4: Wtype the reset code "73738#"
 *
 * This task:
 *   - Commands arm to move over each key
 *   - Presses and releases each key in sequence
 *   - Does NOT drive the robot (must already be positioned)
 *
 * State machine (per key):
 *   SETTLE -> MOVE_TO_KEY -> PRESS_HOLD -> RELEASE_HOLD -> (next key or DONE)
 */

#include <cstdint>
#include <mcu_msgs/msg/arm_command.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/char.hpp>
#include <string>

#include "secbot_autonomy/task_base.hpp"

namespace secbot {

/**
 * @brief Configuration for keypad entry task
 */
struct KeypadEnterConfig {
  /// The code to type (ruleset says "73738#")
  std::string code = "73738#";

  /// Timing (seconds)
  float settle_s = 0.25f;        ///< Initial settle before first key
  float move_timeout_s = 0.80f;  ///< Max time to wait for arm positioning
  float press_hold_s = 0.18f;    ///< How long to hold key down
  float release_hold_s = 0.18f;  ///< How long to wait between keys

  /// Pusher positions (degrees or ticks)
  int16_t press_position = 90;
  int16_t release_position = 0;

  /// Actuator settings
  uint8_t pusher_joint_id = 3;
  uint8_t actuator_speed = 200;

  /// Safety timeout
  float timeout_s = 10.0f;

  /// Topic names
  std::string arm_command_topic = "arm_command";
  std::string key_target_topic = "keypad_target";  ///< Char msg for which key
  std::string at_key_topic = "keypad_at_target";   ///< Bool feedback (optional)
};

/**
 * @brief Keypad entry task for Antenna #4
 *
 * Types the reset code by commanding arm movements and key presses
 */
class KeypadEnterTask : public TaskBase {
 public:
  KeypadEnterTask(rclcpp::Node::SharedPtr node,
                  const KeypadEnterConfig& cfg = KeypadEnterConfig{});

  /// Update configuration
  void setConfig(const KeypadEnterConfig& cfg) { cfg_ = cfg; }

  /// Set a different code to type
  void setCode(const std::string& code) { cfg_.code = code; }

  // TaskBase interface
  void start() override;
  void step() override;
  void cancel() override;
  void reset() override;
  std::string name() const override { return "keypad_enter"; }

  // Feedback
  uint8_t currentIndex() const { return idx_; }
  char currentKey() const { return cur_key_; }
  uint8_t codeLength() const {
    return static_cast<uint8_t>(cfg_.code.length());
  }

 private:
  enum class State : uint8_t {
    kIdle = 0,
    kSettle,       ///< Initial pause
    kMoveToKey,    ///< Arm moving to key position
    kPressHold,    ///< Key pressed, holding
    kReleaseHold,  ///< Key released, waiting
    kDone
  };

  void enterState(State s);
  void commandPusher(int16_t position);
  void commandKeyTarget(char key);

  // Callback
  void onAtKey(const std_msgs::msg::Bool::SharedPtr msg);

  static float clamp01(float x);

  KeypadEnterConfig cfg_;
  State state_ = State::kIdle;

  rclcpp::Time start_time_;
  rclcpp::Time state_entry_time_;

  // Sequence tracking
  uint8_t idx_ = 0;
  char cur_key_ = '\0';

  // Arm feedback
  bool at_key_ = false;
  bool at_key_valid_ = false;

  // ROS interfaces
  rclcpp::Publisher<mcu_msgs::msg::ArmCommand>::SharedPtr arm_pub_;
  rclcpp::Publisher<std_msgs::msg::Char>::SharedPtr key_target_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr at_key_sub_;
};

}  // namespace secbot
