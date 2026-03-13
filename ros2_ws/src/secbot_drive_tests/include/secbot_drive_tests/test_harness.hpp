#pragma once
/**
 * @file test_harness.hpp
 * @brief Common test harness for drive base integration tests
 *
 * Provides shared infrastructure: drive_base publisher/subscriber,
 * pose tracking, goal helpers, result reporting, a phase-based
 * state machine framework, and pre-test setup (IMU tare + drive config).
 */

#include <chrono>
#include <cmath>
#include <functional>
#include <string>
#include <vector>

#include <mcu_msgs/msg/drive_base.hpp>
#include <mcu_msgs/srv/reset.hpp>
#include <mcu_msgs/srv/set_drive_config.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>

namespace secbot_drive_tests {

// ── Result of a single test step ──
struct StepResult {
  std::string name;
  bool passed;
  double position_error;   // meters
  double heading_error;    // radians
  double duration;         // seconds
  std::string note;
};

// ── Common base for all drive test nodes ──
class DriveTestHarness : public rclcpp::Node {
 public:
  explicit DriveTestHarness(const std::string& node_name);

 protected:
  // ── Robot state (updated from drive_base/status) ──
  bool has_status_ = false;
  double robot_x_ = 0.0;
  double robot_y_ = 0.0;
  double robot_theta_ = 0.0;
  double robot_vx_ = 0.0;
  double robot_omega_ = 0.0;

  // ── Origin (captured at test start) ──
  double origin_x_ = 0.0;
  double origin_y_ = 0.0;
  double origin_theta_ = 0.0;

  // ── Test parameters ──
  double goal_tolerance_;
  double heading_tolerance_;
  double goal_timeout_;
  double pause_time_;

  // ── Results ──
  std::vector<StepResult> results_;

  // ── Command helpers ──
  void sendVelocity(double vx, double omega);
  void sendGoal(double x, double y, double theta, bool reverse = false);
  void sendTrajectory(const std::vector<std::array<double, 2>>& waypoints,
                      double heading = NAN);
  void stopRobot();

  // ── Pose helpers ──
  double distanceTo(double x, double y) const;
  double headingError(double target_theta) const;
  double driftFromOrigin() const;
  void captureOrigin();
  void logPose(const char* label);

  // ── Angle math ──
  static double normalizeAngle(double a);

  // ── Result reporting ──
  void recordResult(const std::string& name, bool passed,
                    double pos_err, double heading_err,
                    double duration, const std::string& note = "");
  void printSummary();

  // ── Timer access for subclasses ──
  rclcpp::TimerBase::SharedPtr tick_timer_;
  std::chrono::steady_clock::time_point phase_start_;

  double elapsed() const;
  void resetPhaseTimer();

  // ── Pre-test setup (IMU tare + drive config + pose reset) ──
  // Call startSetup() once from WAIT_STATUS, then poll setupComplete() each tick.
  void startSetup();
  bool setupComplete();

  // ── Publishers ──
  rclcpp::Publisher<mcu_msgs::msg::DriveBase>::SharedPtr drive_cmd_pub_;

 private:
  void onDriveStatus(const mcu_msgs::msg::DriveBase::SharedPtr msg);
  rclcpp::Subscription<mcu_msgs::msg::DriveBase>::SharedPtr drive_status_sub_;

  // ── Service clients for pre-test setup ──
  rclcpp::Client<mcu_msgs::srv::Reset>::SharedPtr imu_tare_client_;
  rclcpp::Client<mcu_msgs::srv::SetDriveConfig>::SharedPtr drive_config_client_;
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pose_reset_pub_;

  // Setup state machine
  enum class SetupState { IDLE, TARE_WAIT, CONFIG_WAIT, POSE_RESET, DONE };
  SetupState setup_state_ = SetupState::IDLE;

  rclcpp::Client<mcu_msgs::srv::Reset>::SharedFuture tare_future_;
  rclcpp::Client<mcu_msgs::srv::SetDriveConfig>::SharedFuture config_future_;
  bool tare_sent_ = false;
  bool config_sent_ = false;

  // Drive config parameters (0 = don't change)
  double cfg_wheel_kp_;
  double cfg_wheel_ki_;
  double cfg_wheel_kd_;
  double cfg_wheel_i_min_;
  double cfg_wheel_i_max_;
  double cfg_pose_k_linear_;
  double cfg_pose_k_angular_;
  double cfg_max_linear_vel_;
  double cfg_max_angular_vel_;
  double cfg_max_linear_accel_;
  double cfg_max_angular_accel_;
};

}  // namespace secbot_drive_tests
