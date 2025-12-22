#pragma once
/**
 * @file pressure_clear.hpp
 * @author Rafeed Khan
 * @brief Pressure plate clearing task for Antenna #3 - remove the Astro-Duck
 *
 * This task:
 *   - Commands a sweeper arm to push the duck off the pressure plate
 *   - Optionally runs an intake motor to capture the duck
 *   - Does NOT drive the robot (must already be positioned)
 *
 * State machine:
 *   IDLE -> SETTLE -> PUSH_OUT -> HOLD -> RETRACT -> (repeat or DONE)
 */

#include "secbot_autonomy/task_base.hpp"

#include <secbot_msgs/msg/arm_command.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int16.hpp>

#include <cstdint>

namespace secbot {

/**
 * @brief Configuration for pressure clear task
 */
struct PressureClearConfig {
  /// Sweeper positions (degrees or ticks)
  int16_t sweeper_safe_pos = 0;    ///< Retracted / travel-safe
  int16_t sweeper_push_pos = 90;   ///< Extended / pushes duck off

  /// Actuator settings
  uint8_t sweeper_joint_id = 4;
  uint8_t actuator_speed = 200;

  /// Optional intake motor speed (0 = disabled)
  int16_t intake_speed = 0;

  /// Number of sweep cycles
  uint8_t sweeps = 2;

  /// Timing (seconds)
  float settle_s = 0.20f;          ///< Initial pause
  float push_out_s = 0.35f;        ///< Extend time
  float push_hold_s = 0.25f;       ///< Hold at extended position
  float retract_s = 0.35f;         ///< Retract time
  float between_sweeps_s = 0.20f;  ///< Pause between sweeps
  float capture_wait_s = 0.20f;    ///< Time for duck capture sensor

  /// Safety timeout
  float timeout_s = 4.0f;

  /// Topic names
  std::string arm_command_topic = "arm_command";
  std::string intake_topic = "intake_speed";          ///< Optional
  std::string duck_captured_topic = "duck_captured";  ///< Optional sensor
};

/**
 * @brief Pressure clear task for Antenna #3
 *
 * Sweeps the Astro-Duck off the pressure plate using a sweeper arm.
 */
class PressureClearTask : public TaskBase {
 public:
  PressureClearTask(rclcpp::Node::SharedPtr node,
                    const PressureClearConfig& cfg = PressureClearConfig{});

  /// Update configuration
  void setConfig(const PressureClearConfig& cfg) { cfg_ = cfg; }

  // TaskBase interface
  void start() override;
  void step() override;
  void cancel() override;
  void reset() override;
  std::string name() const override { return "pressure_clear"; }

  // Feedback
  uint8_t sweepsCompleted() const { return sweeps_done_; }
  uint8_t sweepsTotal() const { return cfg_.sweeps; }

 private:
  enum class State : uint8_t {
    kIdle = 0,
    kSettle,    ///< Initial pause
    kPushOut,   ///< Extending sweeper
    kHold,      ///< Holding at push position
    kRetract,   ///< Retracting sweeper
    kBetween,   ///< Pause between sweeps
    kDone
  };

  void enterState(State s);
  void applyOutputs();
  void commandSweeper(int16_t position);
  void commandIntake(int16_t speed);

  // Callback
  void onDuckCaptured(const std_msgs::msg::Bool::SharedPtr msg);

  PressureClearConfig cfg_;
  State state_ = State::kIdle;

  rclcpp::Time start_time_;
  rclcpp::Time state_entry_time_;

  uint8_t sweeps_done_ = 0;

  // Sensor state
  bool duck_captured_ = false;
  bool sensor_valid_ = false;

  // ROS interfaces
  rclcpp::Publisher<secbot_msgs::msg::ArmCommand>::SharedPtr arm_pub_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr intake_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr captured_sub_;
};

}  // namespace secbot
