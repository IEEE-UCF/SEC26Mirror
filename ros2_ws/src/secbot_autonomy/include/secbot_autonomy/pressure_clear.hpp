#pragma once
/**
 * @file pressure_clear.hpp
 * @author Rafeed Khan
 * @brief Pressure plate clearing task for Antenna #3 - retrieve the Astro-Duck
 *
 * This task uses the intake rail (linear rail + spinning motor) to:
 *   - Extend the rail over the pressure plate
 *   - Run the intake motor to capture the duck
 *   - Wait for capture (timeout-based)
 *   - Retract the rail with the duck
 *   - Does NOT drive the robot (must already be positioned)
 *
 * State machine:
 *   IDLE -> SETTLE -> EXTEND_RAIL -> WAIT_CAPTURE -> RETRACT_RAIL -> DONE
 */

#include <cstdint>
#include <mcu_msgs/msg/intake_command.hpp>
#include <mcu_msgs/msg/intake_state.hpp>

#include "secbot_autonomy/task_base.hpp"

namespace secbot {

/**
 * @brief Configuration for pressure clear task
 */
struct PressureClearConfig {
  /// Intake motor speed during capture (0.0 = disabled, 1.0 = full)
  float intake_speed = 1.0f;

  /// Timing (seconds)
  float settle_s = 0.20f;          ///< Initial pause before extending
  float extend_timeout_s = 3.0f;   ///< Max time to wait for rail to extend
  float capture_timeout_s = 2.0f;  ///< Max time to wait for duck capture
  float retract_timeout_s = 3.0f;  ///< Max time to wait for rail to retract

  /// Safety timeout for entire task
  float timeout_s = 12.0f;

  /// Topic names
  std::string intake_command_topic = "/mcu_robot/intake/command";
  std::string intake_state_topic = "/mcu_robot/intake/state";
};

/**
 * @brief Pressure clear task for Antenna #3
 *
 * Uses the intake rail to retrieve the Astro-Duck from the pressure plate.
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

 private:
  enum class State : uint8_t {
    kIdle = 0,
    kSettle,        ///< Initial pause
    kExtendRail,    ///< Extending rail over pressure plate
    kWaitCapture,   ///< Waiting for duck capture (timeout)
    kRetractRail,   ///< Retracting rail with duck
    kDone
  };

  void enterState(State s);
  void sendIntakeCommand(uint8_t cmd, float value = 0.0f);

  // Callback
  void onIntakeState(const mcu_msgs::msg::IntakeState::SharedPtr msg);

  PressureClearConfig cfg_;
  State state_ = State::kIdle;

  rclcpp::Time start_time_;
  rclcpp::Time state_entry_time_;

  // Intake state from subscription
  float intake_position_ = 0.0f;
  bool intake_limit_extend_ = false;
  bool intake_limit_retract_ = false;

  // ROS interfaces
  rclcpp::Publisher<mcu_msgs::msg::IntakeCommand>::SharedPtr intake_cmd_pub_;
  rclcpp::Subscription<mcu_msgs::msg::IntakeState>::SharedPtr
      intake_state_sub_;
};

}  // namespace secbot
