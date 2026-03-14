#pragma once
/**
 * @file mission_node.hpp
 * @brief Mission sequencer for SEC26 competition robot
 *
 * Coordinate system (from statemachine.md):
 *   Origin (0,0) = bottom-right corner of field
 *   +X = LEFT (0 -> 120 cm), +Y = UP (0 -> 240 cm)
 *   Yaw 0 = facing +X (left), positive = CCW
 *   All coordinates in cm, all yaw in degrees internally.
 *
 * Phase 1: Button task (14 steps)
 * Phase 2: Keypad + Crater + Crank (22 steps)
 */

#include <chrono>
#include <cmath>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <mcu_msgs/msg/arm_command.hpp>
#include <mcu_msgs/msg/drive_base.hpp>
#include <mcu_msgs/msg/intake_command.hpp>
#include <mcu_msgs/msg/intake_state.hpp>
#include <mcu_msgs/msg/mini_robot_state.hpp>
#include <mcu_msgs/msg/robot_inputs.hpp>
#include <rclcpp/rclcpp.hpp>
#include <secbot_msgs/msg/task_status.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace secbot {

enum class MissionPhase : uint8_t {
  WAIT_FOR_START = 0,
  ROBOT_START = 1,
  PHASE_BUTTON = 2,
  PHASE_KEYPAD_CRATER = 3,
  MISSION_COMPLETE = 4,
};

class MissionNode : public rclcpp::Node {
 public:
  MissionNode();

 private:
  // State machine
  void stepMission();
  void transitionTo(MissionPhase phase);
  const char* phaseName(MissionPhase phase) const;

  // Step management
  void setStep(int step);
  double stepElapsed() const;

  // Navigation helpers (all coordinates in cm, yaw in degrees)
  void sendGoal(double x_cm, double y_cm, double yaw_deg,
                bool reverse = false);
  void sendVelocity(double vx_mps, double omega_radps);
  void sendNudge(double speed_mps);
  void stopRobot();
  void refreshDriveCommand();
  void resetPose(double x_cm, double y_cm, double yaw_deg);

  // Task helpers
  void sendTaskCommand(uint8_t task_id);
  void sendArmCommand(uint8_t joint_id, int16_t position, uint8_t speed);
  void sendIntakeCommand(uint8_t cmd, float value = 0.0f);

  // Dummy antenna reader (placeholder)
  void readAntennaDummy(const char* label);

  // Timeout helper
  bool checkStepTimeout(double timeout_s);

  // Service callbacks
  void onStartService(const std_srvs::srv::Trigger::Request::SharedPtr req,
                      std_srvs::srv::Trigger::Response::SharedPtr res);

  // Subscriber callbacks
  void onDriveStatus(const mcu_msgs::msg::DriveBase::SharedPtr msg);
  void onTaskStatus(const secbot_msgs::msg::TaskStatus::SharedPtr msg);
  void onRobotInputs(const mcu_msgs::msg::RobotInputs::SharedPtr msg);
  void onIntakeState(const mcu_msgs::msg::IntakeState::SharedPtr msg);

  // Current state
  MissionPhase phase_ = MissionPhase::WAIT_FOR_START;
  bool phase_entry_ = true;
  int step_ = 0;
  bool step_entry_ = true;

  // Timers
  std::chrono::steady_clock::time_point step_timer_;
  std::chrono::steady_clock::time_point phase_timer_;
  std::chrono::steady_clock::time_point match_start_;

  // Navigation state
  bool goal_reached_ = false;
  mcu_msgs::msg::DriveBase last_drive_cmd_;
  uint8_t last_drive_mode_ = 0;

  // Goal tracking (cm / degrees)
  double goal_x_cm_ = 0.0;
  double goal_y_cm_ = 0.0;
  double goal_yaw_deg_ = 0.0;

  // Robot pose from drive_base/status (stored in cm / degrees)
  double robot_x_cm_ = 0.0;
  double robot_y_cm_ = 0.0;
  double robot_yaw_deg_ = 0.0;

  // Status tracking
  bool has_drive_status_ = false;
  uint8_t debug_status_tick_ = 0;
  uint8_t debug_tick_ = 0;

  // Start signal
  bool start_button_pressed_ = false;

  // Task state
  bool task_idle_ = true;

  // Intake state
  uint8_t intake_state_ = 0;

  // Unit conversion helpers
  static double cm2m(double cm) { return cm * 0.01; }
  static double m2cm(double m) { return m * 100.0; }
  static double deg2rad(double deg) { return deg * M_PI / 180.0; }
  static double rad2deg(double rad) { return rad * 180.0 / M_PI; }

  // Tolerances
  static constexpr double GOAL_DIST_TOL_CM = 5.0;      // 5 cm
  static constexpr double GOAL_HEADING_TOL_DEG = 6.0;   // 6 degrees
  static constexpr double DEFAULT_STEP_TIMEOUT_S = 15.0;
  static constexpr double NUDGE_SPEED_MPS = 0.25;       // 25 cm/s for button taps

  // Task IDs
  static constexpr uint8_t TASK_NONE = 0;
  static constexpr uint8_t TASK_KEYPAD_ENTER = 6;

  // Servo Joint IDs
  static constexpr uint8_t JOINT_MINIBOT_LATCH = 6;

  // Servo positions & speed
  static constexpr int16_t SERVO_RETRACTED = 0;
  static constexpr int16_t SERVO_EXTENDED = 90;
  static constexpr uint8_t SERVO_SPEED = 200;

  // ROS interfaces
  rclcpp::TimerBase::SharedPtr tick_timer_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_service_;

  // Publishers
  rclcpp::Publisher<mcu_msgs::msg::DriveBase>::SharedPtr drive_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr task_cmd_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr minibot_cmd_pub_;
  rclcpp::Publisher<mcu_msgs::msg::ArmCommand>::SharedPtr arm_cmd_pub_;
  rclcpp::Publisher<mcu_msgs::msg::IntakeCommand>::SharedPtr intake_cmd_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pose_reset_pub_;

  // Subscribers
  rclcpp::Subscription<mcu_msgs::msg::DriveBase>::SharedPtr drive_status_sub_;
  rclcpp::Subscription<secbot_msgs::msg::TaskStatus>::SharedPtr
      task_status_sub_;
  rclcpp::Subscription<mcu_msgs::msg::RobotInputs>::SharedPtr robot_inputs_sub_;
  rclcpp::Subscription<mcu_msgs::msg::IntakeState>::SharedPtr intake_state_sub_;
};

}  // namespace secbot
