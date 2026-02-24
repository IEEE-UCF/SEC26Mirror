#pragma once
/**
 * @file pressure_clear.hpp
 * @author Rafeed Khan
 * @brief Pressure plate clearing task for Antenna #3 - retrieve the Astro-Duck
 *
 * This task uses the intake bridge (gear-and-rack mechanism) to:
 *   - Extend the bridge over the pressure plate
 *   - Wait for the duck to be captured (TOF detection)
 *   - Run the intake motor to secure the duck
 *   - Retract the bridge with the duck
 *   - Does NOT drive the robot (must already be positioned)
 *
 * State machine:
 *   IDLE -> SETTLE -> EXTEND_BRIDGE -> WAIT_CAPTURE -> RETRACT_BRIDGE -> DONE
 */

#include <cstdint>
#include <mcu_msgs/msg/intake_bridge_command.hpp>
#include <mcu_msgs/msg/intake_bridge_state.hpp>
#include <std_msgs/msg/int16.hpp>

#include "secbot_autonomy/task_base.hpp"

namespace secbot {

/**
 * @brief Configuration for pressure clear task
 */
struct PressureClearConfig {
  /// Intake motor speed during capture (0 = disabled)
  int16_t intake_speed = 200;

  /// Timing (seconds)
  float settle_s = 0.20f;          ///< Initial pause before extending
  float extend_timeout_s = 3.0f;   ///< Max time to wait for bridge to extend
  float capture_timeout_s = 2.0f;  ///< Max time to wait for duck detection
  float retract_timeout_s = 3.0f;  ///< Max time to wait for bridge to retract
  float capture_wait_s = 0.20f;    ///< Extra time after duck detected

  /// Safety timeout for entire task
  float timeout_s = 12.0f;

  /// Topic names
  std::string bridge_command_topic = "/mcu_robot/intake_bridge/command";
  std::string bridge_state_topic = "/mcu_robot/intake_bridge/state";
  std::string intake_topic = "intake_speed";
};

/**
 * @brief Pressure clear task for Antenna #3
 *
 * Uses the intake bridge (gear-and-rack) to retrieve the Astro-Duck
 * from the pressure plate.
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
    kSettle,         ///< Initial pause
    kExtendBridge,   ///< Extending bridge over pressure plate
    kWaitCapture,    ///< Waiting for duck detection from TOF
    kRetractBridge,  ///< Retracting bridge with duck
    kDone
  };

  void enterState(State s);
  void sendBridgeCommand(uint8_t cmd);
  void commandIntake(int16_t speed);

  // Callback
  void onBridgeState(const mcu_msgs::msg::IntakeBridgeState::SharedPtr msg);

  PressureClearConfig cfg_;
  State state_ = State::kIdle;

  rclcpp::Time start_time_;
  rclcpp::Time state_entry_time_;

  // Bridge state from subscription
  uint8_t bridge_state_ = 0;
  bool bridge_duck_detected_ = false;

  // ROS interfaces
  rclcpp::Publisher<mcu_msgs::msg::IntakeBridgeCommand>::SharedPtr
      bridge_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr intake_pub_;
  rclcpp::Subscription<mcu_msgs::msg::IntakeBridgeState>::SharedPtr
      bridge_state_sub_;
};

}  // namespace secbot
