#pragma once
/**
 * @file mission_node.hpp
 * @author Rafeed Khan
 * @brief Mission sequencer that coordinates the full competition run
 *
 * Sequences navigation, task execution, minibot/drone deployment, and duck
 * handling.
 *
 * State ordering:
 *   0-WAIT -> 1-START -> 2-MINIBOT -> 3-NAV_UWB -> 4-PLACE_UWB ->
 *   5-NAV_BUTTON -> 6-SOLVE_BUTTON -> 7-NAV_CRANK_FLAG -> 8-PLACE_FLAG ->
 *   9-NAV_CRANK -> 10-SOLVE_CRANK -> 11-COLLECT_DUCKS ->
 *   12-NAV_KEYPAD -> 13-SOLVE_KEYPAD -> 14-NAV_PRESSURE -> 15-SOLVE_PRESSURE ->
 *   16-DEPOSIT_DUCK -> 17-VIEW_LEDS -> 18-NAV_LAUNCH -> 19-LAUNCH_DRONE ->
 *   20-NAV_FINISH -> 21-MISSION_COMPLETE
 */

#include <array>
#include <chrono>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <mcu_msgs/msg/antenna_marker.hpp>
#include <mcu_msgs/msg/arm_command.hpp>
#include <mcu_msgs/msg/drone_control.hpp>
#include <mcu_msgs/msg/drone_state.hpp>
#include <mcu_msgs/msg/intake_bridge_command.hpp>
#include <mcu_msgs/msg/intake_bridge_state.hpp>
#include <mcu_msgs/msg/intake_state.hpp>
#include <mcu_msgs/msg/mini_robot_state.hpp>
#include <mcu_msgs/msg/robot_inputs.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <secbot_msgs/msg/duck_detections.hpp>
#include <secbot_msgs/msg/task_status.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <vector>

namespace secbot {

/** @brief All mission phases */
enum class MissionPhase : uint8_t {
  WAIT_FOR_START = 0,
  ROBOT_START = 1,
  MINIBOT_LAUNCH = 2,
  NAV_TO_UWB_CORNER = 3,
  PLACE_UWB_BEACON = 4,
  NAV_TO_BUTTON = 5,
  SOLVE_BUTTON = 6,
  NAV_TO_CRANK_FLAG = 7,
  PLACE_UWB_FLAG = 8,
  NAV_TO_CRANK = 9,
  SOLVE_CRANK = 10,
  COLLECT_KNOWN_DUCKS = 11,
  NAV_TO_KEYPAD = 12,
  SOLVE_KEYPAD = 13,
  NAV_TO_PRESSURE = 14,
  SOLVE_PRESSURE = 15,
  DEPOSIT_DUCK = 16,
  VIEW_LED_COLORS = 17,
  NAV_TO_LAUNCH = 18,
  LAUNCH_DRONE = 19,
  NAV_TO_FINISH = 20,
  MISSION_COMPLETE = 21,
  // Duck interrupt states (entered from any NAV phase after COLLECT_KNOWN_DUCKS)
  DUCK_INTERRUPT_NAV,
  DUCK_INTERRUPT_CAPTURE,
  DUCK_INTERRUPT_DEPOSIT_NAV,
  DUCK_INTERRUPT_DEPOSIT,
  DUCK_INTERRUPT_RESUME
};

/** @brief Sub-steps for antenna-align-then-solve pattern */
enum class SolveSubStep : uint8_t { ALIGN_ANTENNA, EXECUTE_TASK, DONE };

/** @brief Sub-steps for intake bridge operation in SOLVE_PRESSURE */
enum class IntakeBridgeStep : uint8_t { EXTEND, WAIT_CAPTURE, RETRACT, DONE };

/** @brief Sub-steps for COLLECT_KNOWN_DUCKS multi-duck loop */
enum class DuckCollectStep : uint8_t {
  NAV_TO_DUCK,
  INTAKE_ON,
  WAIT_CAPTURE,
  NAV_TO_DEPOSIT,
  EJECT,
  WAIT_EJECT,
  NEXT_DUCK
};

/** @brief Sub-steps for DEPOSIT_DUCK after pressure clear */
enum class DepositStep : uint8_t { NAV_TO_ZONE, EJECT, WAIT_EJECT, DONE };

/** @brief Sub-steps for VIEW_LED_COLORS camera + reading */
enum class LedReadStep : uint8_t {
  EXTEND_CAMERA,
  READING,
  RETRACT_CAMERA,
  DONE
};

/** @brief Simple 2D position for arena waypoints */
struct ArenaPosition {
  double x;
  double y;
};

/**
 * @brief Mission sequencer node
 *
 * Orchestrates the entire competition run as a linear state machine.
 * Talks to pathing_node via /goal_pose and /nav/goal_reached, and to
 * autonomy_node via /autonomy/task_command and /autonomy/task_status.
 */
class MissionNode : public rclcpp::Node {
 public:
  MissionNode();

 private:
  // State machine
  void stepMission();
  void transitionTo(MissionPhase phase);
  const char* phaseName(MissionPhase phase) const;

  // Navigation helpers
  void sendNavGoal(const ArenaPosition& pos);

  // Task helpers
  void sendTaskCommand(uint8_t task_id);
  void sendAntennaTarget(uint8_t antenna_id);
  void sendBackup(double speed, double duration_sec);
  void sendArmCommand(uint8_t joint_id, int16_t position, uint8_t speed);
  void setIntakeSpeed(int16_t speed);
  void sendBridgeCommand(uint8_t cmd);

  // Timeout helper
  bool checkStateTimeout(double timeout_s);

  // Callbacks
  void onGoalReached(const std_msgs::msg::Bool::SharedPtr msg);
  void onTaskStatus(const secbot_msgs::msg::TaskStatus::SharedPtr msg);
  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg);
  void onMinibotState(const mcu_msgs::msg::MiniRobotState::SharedPtr msg);
  void onDroneState(const mcu_msgs::msg::DroneState::SharedPtr msg);
  void onRobotInputs(const mcu_msgs::msg::RobotInputs::SharedPtr msg);
  void onIntakeState(const mcu_msgs::msg::IntakeState::SharedPtr msg);
  void onBridgeState(const mcu_msgs::msg::IntakeBridgeState::SharedPtr msg);
  void onDuckDetections(const secbot_msgs::msg::DuckDetections::SharedPtr msg);
  void onAntennaMarker(const mcu_msgs::msg::AntennaMarker::SharedPtr msg);

  // Current state
  MissionPhase phase_ = MissionPhase::WAIT_FOR_START;
  bool phase_entry_ = true;

  // Navigation state
  bool goal_reached_ = false;

  // Task state
  uint8_t current_task_id_ = 0;
  bool task_idle_ = true;

  // Solve sub-step (antenna align then task execute)
  SolveSubStep sub_step_ = SolveSubStep::ALIGN_ANTENNA;

  // Intake bridge sub-step (for SOLVE_PRESSURE)
  IntakeBridgeStep bridge_step_ = IntakeBridgeStep::EXTEND;

  // Robot pose from odom
  double robot_x_ = 0.0;
  double robot_y_ = 0.0;

  // Start signal
  bool start_button_pressed_ = false;

  // Minibot state
  uint8_t minibot_state_ = 0;
  uint8_t minibot_task_ = 0;

  // Drone state
  uint8_t drone_state_ = 0;
  uint8_t drone_task_ = 0;

  // Intake state
  uint8_t intake_state_ = 0;
  bool duck_in_intake_ = false;

  // Bridge state
  uint8_t bridge_state_ = 0;
  bool bridge_duck_detected_ = false;

  // Backup / phase timing
  std::chrono::steady_clock::time_point phase_timer_;
  double backup_duration_sec_ = 0.0;

  // Match timer (5 minute limit)
  std::chrono::steady_clock::time_point match_start_;

  // Duck collection
  std::vector<ArenaPosition> known_duck_positions_;
  size_t duck_collect_index_ = 0;
  DuckCollectStep duck_collect_step_ = DuckCollectStep::NAV_TO_DUCK;
  bool duck_interrupt_enabled_ = false;
  bool pending_duck_interrupt_ = false;
  MissionPhase saved_phase_ = MissionPhase::WAIT_FOR_START;
  bool saved_phase_entry_ = true;
  ArenaPosition interrupt_duck_pos_ = {0.0, 0.0};

  // Deposit duck sub-state
  DepositStep deposit_step_ = DepositStep::NAV_TO_ZONE;

  // LED reading sub-state
  LedReadStep led_step_ = LedReadStep::EXTEND_CAMERA;
  std::array<uint8_t, 5> antenna_colors_ = {};  // indexed by antenna_id (1-4)
  uint8_t antennas_read_ = 0;

  // Arena positions (UPDATE WITH REAL FIELD COORDINATES FOR GODS SAKE)
  static constexpr ArenaPosition POS_UWB_CORNER = {1.5, 0.5};
  static constexpr ArenaPosition POS_BUTTON_BEACON = {2.0, 1.0};
  static constexpr ArenaPosition POS_CRANK_FLAG = {3.5, 1.0};
  static constexpr ArenaPosition POS_CRANK_APPROACH = {4.0, 1.0};
  static constexpr ArenaPosition POS_KEYPAD_BEACON = {6.0, 1.0};
  static constexpr ArenaPosition POS_PRESSURE_PLATE = {6.0, -1.0};
  static constexpr ArenaPosition POS_DUCK_DEPOSIT = {3.0, -2.0};
  static constexpr ArenaPosition POS_MINIBOT_LAUNCH = {1.0, 0.0};
  static constexpr ArenaPosition POS_LAUNCH_POINT = {1.0, -1.0};
  static constexpr ArenaPosition POS_FINISH = {1.0, 0.5};

  // Per-state timeout defaults (seconds)
  static constexpr double DEFAULT_NAV_TIMEOUT_S = 30.0;
  static constexpr double DEFAULT_TASK_TIMEOUT_S = 15.0;

  // Task IDs (must match autonomy_node TaskId enum)
  static constexpr uint8_t TASK_NONE = 0;
  static constexpr uint8_t TASK_ANTENNA_ALIGN = 1;
  static constexpr uint8_t TASK_BUTTON_PRESS = 2;
  static constexpr uint8_t TASK_CRANK_TURN = 3;
  static constexpr uint8_t TASK_CRATER_ENTRY = 4;
  static constexpr uint8_t TASK_FLAG_PLANT = 5;
  static constexpr uint8_t TASK_KEYPAD_ENTER = 6;
  static constexpr uint8_t TASK_PRESSURE_CLEAR = 7;

  // Servo Joint IDs (match MCU firmware joint mapping)
  static constexpr uint8_t JOINT_CAMERA_MAST = 5;
  static constexpr uint8_t JOINT_MINIBOT_LATCH = 6;
  static constexpr uint8_t JOINT_DRONE_LATCH = 7;

  // Servo positions & speed
  static constexpr int16_t SERVO_RETRACTED = 0;
  static constexpr int16_t SERVO_EXTENDED = 90;
  static constexpr uint8_t SERVO_SPEED = 200;

  // Intake speeds
  static constexpr int16_t INTAKE_CAPTURE_SPEED = 200;
  static constexpr int16_t INTAKE_EJECT_SPEED = -200;
  static constexpr int16_t INTAKE_OFF_SPEED = 0;

  // ROS interfaces
  rclcpp::TimerBase::SharedPtr step_timer_;

  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr task_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr antenna_target_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr minibot_cmd_pub_;
  rclcpp::Publisher<mcu_msgs::msg::ArmCommand>::SharedPtr arm_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr intake_speed_pub_;
  rclcpp::Publisher<mcu_msgs::msg::DroneControl>::SharedPtr drone_cmd_pub_;
  rclcpp::Publisher<mcu_msgs::msg::IntakeBridgeCommand>::SharedPtr
      bridge_cmd_pub_;

  // Subscribers
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr goal_reached_sub_;
  rclcpp::Subscription<secbot_msgs::msg::TaskStatus>::SharedPtr
      task_status_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<mcu_msgs::msg::MiniRobotState>::SharedPtr
      minibot_state_sub_;
  rclcpp::Subscription<mcu_msgs::msg::DroneState>::SharedPtr drone_state_sub_;
  rclcpp::Subscription<mcu_msgs::msg::RobotInputs>::SharedPtr robot_inputs_sub_;
  rclcpp::Subscription<mcu_msgs::msg::IntakeState>::SharedPtr intake_state_sub_;
  rclcpp::Subscription<mcu_msgs::msg::IntakeBridgeState>::SharedPtr
      bridge_state_sub_;
  rclcpp::Subscription<secbot_msgs::msg::DuckDetections>::SharedPtr
      duck_detect_sub_;
  rclcpp::Subscription<mcu_msgs::msg::AntennaMarker>::SharedPtr
      antenna_marker_sub_;
};

}  // namespace secbot
