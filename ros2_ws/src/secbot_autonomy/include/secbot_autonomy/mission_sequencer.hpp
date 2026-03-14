#pragma once
/**
 * @file mission_sequencer.hpp
 * @brief Corrected mission sequencer with turn-before-drive and coordinate
 *        frame conversion.
 *
 * Fixes three issues in the original mission_node:
 *   1. Coordinate frame mismatch: Rafeed uses +Y=east, +angle=CW.
 *      MCU uses +Y=west, +angle=CCW. Every Y and angle is negated.
 *   2. Navigation: DRIVE → TURN_TO_HEADING (assumes already facing goal).
 *   3. No IMU tare or inter-move pause.
 *
 * All waypoints are preserved from Rafeed's original mission_node in his
 * coordinate frame (cm, degrees). Conversion happens at the boundary via
 * rafeedToMcu() before any DriveBase message is built.
 */

#include <array>
#include <chrono>
#include <cmath>
#include <future>
#include <string>
#include <vector>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <mcu_msgs/msg/arm_command.hpp>
#include <mcu_msgs/msg/drive_base.hpp>
#include <mcu_msgs/msg/intake_command.hpp>
#include <mcu_msgs/msg/intake_state.hpp>
#include <mcu_msgs/msg/robot_inputs.hpp>
#include <mcu_msgs/srv/reset.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <secbot_msgs/action/read_beacon_color.hpp>
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
  static double rad2deg(double rad) { return rad * 180.0 / M_PI; }

  static double normalizeAngle(double a) {
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
  }

  // ── Tolerances & timing ──
  static constexpr double DIST_TOL_M = 0.03;          // 3 cm
  static constexpr double HEADING_TOL_RAD = 0.12;      // ~7 deg
  double turn_timeout_s_ = 3.0;
  double drive_timeout_s_ = 3.0;
  double default_step_timeout_s_ = 3.0;
  static constexpr double INTER_MOVE_PAUSE_S = 0.4;
  static constexpr double MIN_DRIVE_DIST_M = 0.02;     // below = pure turn
  static constexpr double HEADING_MATCH_RAD = 0.1;     // ~6 deg, skip final turn
  static constexpr double SETUP_SETTLE_S = 0.5;

  // ── Turn-in-place P-controller ──
  // Uses DRIVE_VECTOR mode (vx=0, omega=K*err) so MCU never tries to
  // correct a phantom position error from atan2 when dist < poseDistTol.
  static constexpr double TURN_KP = 2.0;              // rad/s per rad (matches MCU)
  static constexpr double TURN_MAX_OMEGA = 3.0;       // rad/s cap (conservative)
  static constexpr double TURN_MIN_OMEGA = 0.3;       // rad/s floor to overcome friction

  // ── Task IDs ──
  static constexpr uint8_t TASK_NONE = 0;

  // ── Parsed mission command ──
  enum class CmdType : uint8_t {
    NAV, NAV_REV, NUDGE, STOP, WAIT, TASK,
    READ_ANTENNA, ARM, INTAKE, VELOCITY_RAW, MINIBOT,
  };

  struct MissionCmd {
    CmdType type;
    double x = 0, y = 0, heading = 0;  // NAV/NAV_REV
    double timeout = 0;                 // 0 = use default
    double duration = 1.0;              // WAIT/NUDGE/VELOCITY_RAW
    double vx = 0, omega = 0;          // VELOCITY_RAW
    uint8_t task_id = 0;               // TASK
    std::string label;                  // READ_ANTENNA
    float camera_angle = 90.0f;        // READ_ANTENNA
    int antenna_slot = -1;             // READ_ANTENNA storage index
    uint8_t joint = 0;                 // ARM
    int16_t position = 0;              // ARM
    uint8_t speed = 0;                 // ARM
    uint8_t intake_cmd = 0;            // INTAKE
    std::string minibot_action;        // MINIBOT ("enter" or "exit")
  };

  static MissionCmd parseCommand(const std::string& line);
  void loadMission();
  void executePhaseCommands(std::vector<MissionCmd>& cmds, int& idx,
                            MissionPhase next_phase, const char* phase_name);

  std::vector<MissionCmd> button_cmds_;
  std::vector<MissionCmd> keypad_cmds_;
  int button_cmd_idx_ = 0;
  int keypad_cmd_idx_ = 0;

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
  void sendTurnInPlace(double target_rad);  // P-controller via DRIVE_VECTOR
  bool turnReachedHeading(double target_rad) const;
  void sendVelocity(double vx_mps, double omega_radps);
  void sendVelocityRaw(double vx_mps, double omega_radps);
  void sendNudge(double speed_mps);
  void stopRobot();
  void refreshDriveCommand();
  void resetPoseMcu(double x_m, double y_m, double theta_rad);

  // ── Task / arm / intake helpers ──
  void sendTaskCommand(uint8_t task_id);
  void sendArmCommand(uint8_t joint_id, int16_t position, uint8_t speed);
  void sendIntakeCommand(uint8_t cmd, float value = 0.0f);

  // ── Antenna reading (ReadBeaconColor action) ──
  using ReadBeaconColor = secbot_msgs::action::ReadBeaconColor;
  using BeaconGoalHandle = rclcpp_action::ClientGoalHandle<ReadBeaconColor>;

  void startAntennaRead(const char* label, float camera_angle);
  bool antennaReadComplete() const { return !antenna_read_active_; }
  std::string lastAntennaColor() const { return last_antenna_color_; }

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

  // Minibot service call state
  bool minibot_call_active_ = false;
  bool minibot_call_done_ = false;
  bool minibot_call_success_ = false;

  // Antenna reading state
  bool antenna_read_active_ = false;
  std::string last_antenna_color_;
  std::array<std::string, 5> antenna_colors_;  // indexed by antenna (0-3: beacon, keypad, crater, crank)

  // ── Tunable parameters from YAML ──
  double nudge_speed_mps_ = 0.25;

  // ── ROS interfaces ──
  rclcpp::TimerBase::SharedPtr tick_timer_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_service_;

  // Service clients
  rclcpp::Client<mcu_msgs::srv::Reset>::SharedPtr imu_tare_client_;
  rclcpp::Client<mcu_msgs::srv::Reset>::SharedPtr minibot_enter_client_;
  rclcpp::Client<mcu_msgs::srv::Reset>::SharedPtr minibot_exit_client_;

  // Action clients
  rclcpp_action::Client<ReadBeaconColor>::SharedPtr beacon_color_client_;

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
