#pragma once
/**
 * @file mission_sequencer.hpp
 * @brief Corrected mission sequencer with turn-before-drive and coordinate
 *        frame conversion.
 *
 * Fixes three issues in the original mission_node:
 *   1. Coordinate frame mismatch: Rafeed uses +Y=east, +angle=CW.
 *      MCU uses +Y=west, +angle=CCW. Every Y and angle is negated.
 *   2. No turn-before-drive: simultaneous translation+rotation causes arcing.
 *   3. No IMU tare or inter-move pause.
 *
 * All waypoints are preserved from Rafeed's original mission_node in his
 * coordinate frame (cm, degrees). Conversion happens at the boundary via
 * rafeedToMcu() before any DriveBase message is built.
 */

#include <chrono>
#include <cmath>
#include <future>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <mcu_msgs/msg/arm_command.hpp>
#include <mcu_msgs/msg/drive_base.hpp>
#include <mcu_msgs/msg/intake_command.hpp>
#include <mcu_msgs/msg/intake_state.hpp>
#include <mcu_msgs/msg/mini_robot_state.hpp>
#include <mcu_msgs/msg/robot_inputs.hpp>
#include <mcu_msgs/srv/reset.hpp>
#include <rclcpp/rclcpp.hpp>
#include <secbot_msgs/msg/task_status.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace secbot {

// ── Mission phases (same as Rafeed's) ──
enum class MissionPhase : uint8_t {
  WAIT_FOR_START = 0,
  ROBOT_START = 1,
  PHASE_BUTTON = 2,
  PHASE_KEYPAD_CRATER = 3,
  MISSION_COMPLETE = 4,
};

// ── Setup state machine (IMU tare → pose reset → settle) ──
enum class SetupState : uint8_t {
  IDLE = 0,
  TARE_WAIT,
  POSE_RESET,
  SETTLE,
  DONE,
};

// ── NavMove sub-steps ──
enum class NavSubStep : uint8_t {
  IDLE = 0,
  TURN_TO_FACE,
  DRIVE,
  TURN_TO_HEADING,
  PAUSE,
  DONE,
};

class MissionSequencer : public rclcpp::Node {
 public:
  MissionSequencer();

 private:
  // ── Coordinate conversion ──
  // Rafeed: +X=north, +Y=east, +angle=CW, cm, degrees
  // MCU:    +X=north, +Y=west, +angle=CCW, meters, radians
  struct McuPose {
    double x_m;
    double y_m;
    double theta_rad;
  };
  struct RafeedPose {
    double x_cm;
    double y_cm;
    double yaw_deg;
  };

  static McuPose rafeedToMcu(double x_cm, double y_cm, double yaw_deg) {
    return {x_cm * 0.01, -y_cm * 0.01, -yaw_deg * M_PI / 180.0};
  }
  static RafeedPose mcuToRafeed(double x_m, double y_m, double theta_rad) {
    return {x_m * 100.0, -y_m * 100.0, -theta_rad * 180.0 / M_PI};
  }

  // ── Unit helpers ──
  static double cm2m(double cm) { return cm * 0.01; }
  static double m2cm(double m) { return m * 100.0; }
  static double deg2rad(double deg) { return deg * M_PI / 180.0; }
  static double rad2deg(double rad) { return rad * 180.0 / M_PI; }

  static double normalizeAngle(double a) {
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
  }

  // ── Tolerances & timing ──
  static constexpr double DIST_TOL_M = 0.03;          // 3 cm
  static constexpr double HEADING_TOL_RAD = 0.12;      // ~7 deg
  static constexpr double TURN_TIMEOUT_S = 8.0;
  static constexpr double DRIVE_TIMEOUT_S = 12.0;
  static constexpr double INTER_MOVE_PAUSE_S = 0.4;
  static constexpr double MIN_DRIVE_DIST_M = 0.02;     // below = pure turn
  static constexpr double HEADING_MATCH_RAD = 0.1;     // ~6 deg, skip final turn
  static constexpr double DEFAULT_STEP_TIMEOUT_S = 15.0;
  static constexpr double NUDGE_SPEED_MPS = 0.25;
  static constexpr double SETUP_SETTLE_S = 0.5;

  // ── Task IDs ──
  static constexpr uint8_t TASK_NONE = 0;
  static constexpr uint8_t TASK_KEYPAD_ENTER = 6;

  // ── Servo constants ──
  static constexpr uint8_t JOINT_MINIBOT_LATCH = 6;
  static constexpr int16_t SERVO_RETRACTED = 0;
  static constexpr int16_t SERVO_EXTENDED = 90;
  static constexpr uint8_t SERVO_SPEED = 200;

  // ── Setup state machine ──
  void startSetup(double start_x_cm, double start_y_cm, double start_yaw_deg);
  bool tickSetup();  // returns true when done

  SetupState setup_state_ = SetupState::IDLE;
  bool tare_sent_ = false;
  std::shared_future<mcu_msgs::srv::Reset::Response::SharedPtr> tare_future_;
  McuPose setup_pose_{};
  std::chrono::steady_clock::time_point settle_start_;

  // ── NavMove executor ──
  void startNavMove(double x_cm, double y_cm, double heading_deg,
                    bool reverse = false);
  void tickNavMove();
  bool navMoveComplete() const { return nav_sub_step_ == NavSubStep::DONE; }

  NavSubStep nav_sub_step_ = NavSubStep::IDLE;
  McuPose nav_target_{};       // target in MCU coords
  double nav_final_heading_;   // final heading in MCU radians
  double nav_travel_heading_;  // heading to face target in MCU radians
  bool nav_reverse_ = false;
  bool nav_is_pure_turn_ = false;
  NavSubStep nav_pause_next_ = NavSubStep::DONE;  // what to do after PAUSE
  std::chrono::steady_clock::time_point nav_substep_timer_;

  void advanceNavSubStep(NavSubStep next);

  // ── Low-level MCU command helpers ──
  void sendGoalMcu(double x_m, double y_m, double theta_rad,
                   bool reverse = false);
  void sendVelocity(double vx_mps, double omega_radps);
  void sendNudge(double speed_mps);
  void stopRobot();
  void refreshDriveCommand();
  void resetPoseMcu(double x_m, double y_m, double theta_rad);

  // ── Task / arm / intake helpers ──
  void sendTaskCommand(uint8_t task_id);
  void sendArmCommand(uint8_t joint_id, int16_t position, uint8_t speed);
  void sendIntakeCommand(uint8_t cmd, float value = 0.0f);
  void readAntennaDummy(const char* label);

  // ── State machine ──
  void stepMission();
  void transitionTo(MissionPhase phase);
  const char* phaseName(MissionPhase phase) const;
  void setStep(int step);
  double stepElapsed() const;
  bool checkStepTimeout(double timeout_s);

  // ── Service callbacks ──
  void onStartService(const std_srvs::srv::Trigger::Request::SharedPtr req,
                      std_srvs::srv::Trigger::Response::SharedPtr res);

  // ── Subscriber callbacks ──
  void onDriveStatus(const mcu_msgs::msg::DriveBase::SharedPtr msg);
  void onTaskStatus(const secbot_msgs::msg::TaskStatus::SharedPtr msg);
  void onRobotInputs(const mcu_msgs::msg::RobotInputs::SharedPtr msg);
  void onIntakeState(const mcu_msgs::msg::IntakeState::SharedPtr msg);

  // ── State ──
  MissionPhase phase_ = MissionPhase::WAIT_FOR_START;
  bool phase_entry_ = true;
  int step_ = 0;
  bool step_entry_ = true;

  std::chrono::steady_clock::time_point step_timer_;
  std::chrono::steady_clock::time_point phase_timer_;
  std::chrono::steady_clock::time_point match_start_;

  // Navigation state
  bool goal_reached_ = false;
  mcu_msgs::msg::DriveBase last_drive_cmd_;
  uint8_t last_drive_mode_ = 0;

  // Current goal in MCU coords (for goal-reached check)
  double goal_x_m_ = 0.0;
  double goal_y_m_ = 0.0;
  double goal_theta_rad_ = 0.0;

  // Robot pose in MCU coords (meters, radians)
  double robot_x_m_ = 0.0;
  double robot_y_m_ = 0.0;
  double robot_theta_rad_ = 0.0;

  // Status tracking
  bool has_drive_status_ = false;
  uint8_t debug_status_tick_ = 0;

  // Start signal
  bool start_button_pressed_ = false;

  // Task state
  bool task_idle_ = true;

  // Intake state
  uint8_t intake_state_ = 0;

  // ── ROS interfaces ──
  rclcpp::TimerBase::SharedPtr tick_timer_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_service_;

  // Service clients
  rclcpp::Client<mcu_msgs::srv::Reset>::SharedPtr imu_tare_client_;

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
  rclcpp::Subscription<mcu_msgs::msg::RobotInputs>::SharedPtr
      robot_inputs_sub_;
  rclcpp::Subscription<mcu_msgs::msg::IntakeState>::SharedPtr
      intake_state_sub_;
};

}  // namespace secbot
