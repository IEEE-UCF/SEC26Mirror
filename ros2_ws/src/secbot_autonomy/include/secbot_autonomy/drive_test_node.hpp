#pragma once
/**
 * @file drive_test_node.hpp
 * @brief Drive integration test node
 *
 * Two modes:
 *   1) Forward test: send forward command, verify direction integration
 *   2) Back-and-forth: infinite loop with optional turns, verify IMU + encoder accuracy
 *
 * Publishes to drive_base/command, reads from drive_base/status.
 */

#include <chrono>
#include <cmath>
#include <mcu_msgs/msg/drive_base.hpp>
#include <rclcpp/rclcpp.hpp>

namespace secbot {

class DriveTestNode : public rclcpp::Node {
 public:
  DriveTestNode();

 private:
  void tick();
  void sendVelocity(double vx, double omega);
  void stopRobot();
  void onDriveStatus(const mcu_msgs::msg::DriveBase::SharedPtr msg);
  void logPose(const char* label);

  enum class Phase : uint8_t {
    WAIT_FOR_STATUS,
    FORWARD,
    STOP_AFTER_FORWARD,
    BACKWARD,
    STOP_AFTER_BACKWARD,
    TURN_CW,
    STOP_AFTER_TURN_CW,
    TURN_CCW,
    STOP_AFTER_TURN_CCW,
    CYCLE_SUMMARY,
    DONE
  };

  const char* phaseName(Phase p) const;

  double speed_;
  double leg_time_;
  double turn_speed_;
  bool with_turn_;
  bool loop_;
  double stop_time_;

  Phase phase_ = Phase::WAIT_FOR_STATUS;
  std::chrono::steady_clock::time_point phase_start_;
  int cycle_ = 0;

  bool has_status_ = false;
  double robot_x_ = 0.0;
  double robot_y_ = 0.0;
  double robot_theta_ = 0.0;
  double robot_vx_ = 0.0;
  double robot_omega_ = 0.0;

  double origin_x_ = 0.0;
  double origin_y_ = 0.0;
  double origin_theta_ = 0.0;

  rclcpp::TimerBase::SharedPtr tick_timer_;
  rclcpp::Publisher<mcu_msgs::msg::DriveBase>::SharedPtr drive_cmd_pub_;
  rclcpp::Subscription<mcu_msgs::msg::DriveBase>::SharedPtr drive_status_sub_;
};

}  // namespace secbot
