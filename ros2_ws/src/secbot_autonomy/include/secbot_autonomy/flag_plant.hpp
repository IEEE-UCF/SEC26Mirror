#pragma once
/**
 * @file flag_plant.hpp
 * @author Rafeed Khan
 * @brief Plant the final flag (release/drop) task
 *
 * This task:
 *   - Does NOT drive the robot (must already be positioned)
 *   - Commands a latch servo/actuator to release the flag
 *   - Optionally uses a "flag present" sensor to confirm release
 *
 * State machine:
 *   IDLE -> SETTLE -> UNLATCH_HOLD -> POST_DROP -> DONE
 */

#include "secbot_autonomy/task_base.hpp"

#include <secbot_msgs/msg/arm_command.hpp>
#include <std_msgs/msg/bool.hpp>

#include <cstdint>

namespace secbot {

/**
 * @brief Configuration for flag plant task
 */
struct FlagPlantConfig {
  /// Actuator positions (degrees or ticks)
  int16_t latch_closed_pos = 0;    ///< Holding the flag (latched)
  int16_t latch_open_pos = 90;     ///< Releasing the flag (unlatched)

  /// Actuator speed (0-255)
  uint8_t actuator_speed = 200;

  /// Joint ID for latch servo
  uint8_t latch_joint_id = 2;

  /// Timing (seconds)
  float settle_s = 0.20f;      ///< Brief settle before unlatching
  float unlatch_s = 0.80f;     ///< Hold open for flag to fall
  float post_drop_s = 0.40f;   ///< Keep open to avoid re-catching

  /// Optional sensor settings
  bool use_flag_sensor = false;
  float sensor_debounce_s = 0.06f;

  /// Safety timeout
  float timeout_s = 3.0f;

  /// Topic names
  std::string arm_command_topic = "arm_command";
  std::string flag_present_topic = "flag_present";  ///< Optional sensor
};

/**
 * @brief Flag plant task
 *
 * Releases the flag by commanding a latch actuator.
 * Robot must already be positioned outside the starting area.
 */
class FlagPlantTask : public TaskBase {
 public:
  FlagPlantTask(rclcpp::Node::SharedPtr node,
                const FlagPlantConfig& cfg = FlagPlantConfig{});

  /// Update configuration
  void setConfig(const FlagPlantConfig& cfg) { cfg_ = cfg; }

  // TaskBase interface
  void start() override;
  void step() override;
  void cancel() override;
  void reset() override;
  std::string name() const override { return "flag_plant"; }

  /// Check if flag was confirmed released (only valid after completion)
  bool flagReleaseConfirmed() const { return release_confirmed_; }

 private:
  enum class State : uint8_t {
    kIdle = 0,
    kSettle,       ///< Brief pause before releasing
    kUnlatchHold,  ///< Latch open, waiting for flag to fall
    kPostDrop,     ///< Keep open to avoid snagging
    kDone
  };

  void enterState(State s);
  void commandLatch(int16_t position);

  // Sensor callback
  void onFlagPresent(const std_msgs::msg::Bool::SharedPtr msg);

  static float clamp01(float x);

  FlagPlantConfig cfg_;
  State state_ = State::kIdle;

  rclcpp::Time start_time_;
  rclcpp::Time state_entry_time_;

  // Sensor state
  bool flag_present_ = true;
  bool sensor_valid_ = false;
  float present_stable_t_ = 0.0f;
  bool last_present_ = true;
  rclcpp::Time last_sensor_time_;

  // Result
  bool release_confirmed_ = false;

  // ROS interfaces
  rclcpp::Publisher<secbot_msgs::msg::ArmCommand>::SharedPtr arm_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr flag_sub_;
};

}  // namespace secbot
