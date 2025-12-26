#pragma once
/**
 * @file crank_turn.hpp
 * @author Rafeed Khan
 * @brief Antenna #2 crank task: rotate the crank at least 540 degrees
 *
 * This task:
 *   - Assumes robot is already aligned to the antenna
 *   - Commands a crank motor via ROS topic
 *   - Optionally reads encoder feedback for closed-loop control
 *   - Falls back to open-loop timed rotation if no encoder
 *
 * State machine:
 *   IDLE -> SETTLE -> SPIN -> HOLD -> DONE
 */

#include <cstdint>
#include <mcu_msgs/msg/arm_command.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>

#include "secbot_autonomy/task_base.hpp"

namespace secbot {

/**
 * @brief Configuration for crank turn task
 */
struct CrankTurnConfig {
  /// Target rotation in degrees (rules say >= 540)
  float target_deg = 540.0f;

  /// Direction: +1 = CW, -1 = CCW
  int8_t direction = +1;

  /// Motor speed command (0-255)
  uint8_t spin_speed = 200;

  /// Estimated degrees per second (for open-loop fallback)
  float est_deg_per_s = 180.0f;

  /// Override open-loop spin time (if > 0, use this instead of computed)
  float open_loop_spin_s = -1.0f;

  /// Tolerance for encoder-based stopping
  float deg_tolerance = 8.0f;

  /// Settle time before spinning
  float settle_s = 0.25f;

  /// Hold time after spinning (let motor stop)
  float hold_s = 0.20f;

  /// Safety timeout
  float timeout_s = 8.0f;

  /// Joint ID for the crank motor
  uint8_t crank_joint_id = 1;

  /// Topic names
  std::string arm_command_topic = "arm_command";
  std::string crank_encoder_topic =
      "crank_angle";  ///< Optional encoder feedback
  std::string task_complete_topic =
      "task_complete";  ///< Optional completion signal
};

/**
 * @brief Crank turn task for Antenna #2
 *
 * Rotates the crank >= 540 degrees. Uses encoder if available,
 * otherwise falls back to timed open-loop rotation.
 */
class CrankTurnTask : public TaskBase {
 public:
  CrankTurnTask(rclcpp::Node::SharedPtr node,
                const CrankTurnConfig& cfg = CrankTurnConfig{});

  /// Update configuration
  void setConfig(const CrankTurnConfig& cfg) { cfg_ = cfg; }

  // TaskBase interface
  void start() override;
  void step() override;
  void cancel() override;
  void reset() override;
  std::string name() const override { return "crank_turn"; }

  // Feedback
  float degreesTurned() const { return turned_deg_; }
  float targetDegrees() const { return cfg_.target_deg; }

 private:
  enum class State : uint8_t { kIdle = 0, kSettle, kSpin, kHold, kDone };

  /// Degree unwrapping for continuous rotation tracking
  struct DegUnwrap {
    void reset(float deg);
    float update(float deg);
    float last = 0.0f;
    float acc = 0.0f;
    bool inited = false;
  };

  void enterState(State s);
  void commandMotor(int16_t speed);
  void stopMotor();

  static float wrapDeltaDeg(float d);
  static float clamp01(float x);

  // Callbacks
  void onEncoderAngle(const std_msgs::msg::Float32::SharedPtr msg);
  void onTaskComplete(const std_msgs::msg::Bool::SharedPtr msg);

  CrankTurnConfig cfg_;
  State state_ = State::kIdle;

  rclcpp::Time start_time_;
  rclcpp::Time state_entry_time_;
  float t_state_ = 0.0f;

  // Encoder tracking
  DegUnwrap unwrap_;
  float start_unwrapped_deg_ = 0.0f;
  float turned_deg_ = 0.0f;
  bool encoder_valid_ = false;
  float current_encoder_deg_ = 0.0f;

  // Open-loop timing
  float spin_goal_s_ = 0.0f;

  // External completion signal
  bool task_complete_signal_ = false;

  // ROS interfaces
  rclcpp::Publisher<mcu_msgs::msg::ArmCommand>::SharedPtr arm_pub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr encoder_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr complete_sub_;
};

}  // namespace secbot
