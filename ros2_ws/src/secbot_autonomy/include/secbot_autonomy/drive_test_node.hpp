#pragma once
/**
 * @file drive_test_node.hpp
 * @brief Drive integration test node using DRIVE_GOAL (closed-loop PID)
 *
 * Two modes:
 *   1) Forward test: goal forward, goal back, verify direction integration
 *   2) Back-and-forth: infinite loop with optional turn goals, verify IMU + encoder accuracy
 *
 * Uses the Teensy's internal PID (DRIVE_GOAL mode), not open-loop velocity.
 */

#include <chrono>
#include <cmath>
#include <mcu_msgs/msg/drive_base.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace secbot {

class DriveTestNode : public rclcpp::Node {
 public:
  DriveTestNode();

 private:
  void tick();
  void sendGoal(double x, double y, double theta);
  void stopRobot();
  void onDriveStatus(const mcu_msgs::msg::DriveBase::SharedPtr msg);
  void onImu(const sensor_msgs::msg::Imu::SharedPtr msg);
  void logPose(const char* label);
  bool reachedGoal();

  enum class Phase : uint8_t {
    WAIT_FOR_STATUS,
    GYRO_CALIBRATE,
    GOAL_FORWARD,
    PAUSE_AFTER_FORWARD,
    GOAL_BACKWARD,
    PAUSE_AFTER_BACKWARD,
    GOAL_TURN_CW,
    PAUSE_AFTER_TURN_CW,
    GOAL_TURN_CCW,
    PAUSE_AFTER_TURN_CCW,
    CYCLE_SUMMARY,
    DONE
  };

  double distance_;
  double turn_angle_;
  bool with_turn_;
  bool loop_;
  double pause_time_;
  double goal_tolerance_;
  double goal_timeout_;

  Phase phase_ = Phase::WAIT_FOR_STATUS;
  std::chrono::steady_clock::time_point phase_start_;
  int cycle_ = 0;

  // Current goal target
  double goal_x_ = 0.0;
  double goal_y_ = 0.0;
  double goal_theta_ = 0.0;

  // Pose from drive_base/status
  bool has_status_ = false;
  double robot_x_ = 0.0;
  double robot_y_ = 0.0;
  double robot_theta_ = 0.0;
  double robot_vx_ = 0.0;
  double robot_omega_ = 0.0;

  // Origin (captured at start)
  double origin_x_ = 0.0;
  double origin_y_ = 0.0;
  double origin_theta_ = 0.0;

  // Gyro offset calibration
  double gyro_sum_x_ = 0.0;
  double gyro_sum_y_ = 0.0;
  double gyro_sum_z_ = 0.0;
  double gyro_min_x_ = 1e9, gyro_max_x_ = -1e9;
  double gyro_min_y_ = 1e9, gyro_max_y_ = -1e9;
  double gyro_min_z_ = 1e9, gyro_max_z_ = -1e9;
  int gyro_samples_ = 0;
  double calibrate_time_;

  rclcpp::TimerBase::SharedPtr tick_timer_;
  rclcpp::Publisher<mcu_msgs::msg::DriveBase>::SharedPtr drive_cmd_pub_;
  rclcpp::Subscription<mcu_msgs::msg::DriveBase>::SharedPtr drive_status_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
};

}  // namespace secbot
