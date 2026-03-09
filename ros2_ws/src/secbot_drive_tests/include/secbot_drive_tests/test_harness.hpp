#pragma once
/**
 * @file test_harness.hpp
 * @brief Common test harness for drive base integration tests
 *
 * Provides shared infrastructure: drive_base publisher/subscriber,
 * pose tracking, goal helpers, result reporting, and a phase-based
 * state machine framework.
 */

#include <chrono>
#include <cmath>
#include <functional>
#include <string>
#include <vector>

#include <mcu_msgs/msg/drive_base.hpp>
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

  // ── Publishers ──
  rclcpp::Publisher<mcu_msgs::msg::DriveBase>::SharedPtr drive_cmd_pub_;

 private:
  void onDriveStatus(const mcu_msgs::msg::DriveBase::SharedPtr msg);
  rclcpp::Subscription<mcu_msgs::msg::DriveBase>::SharedPtr drive_status_sub_;
};

}  // namespace secbot_drive_tests
