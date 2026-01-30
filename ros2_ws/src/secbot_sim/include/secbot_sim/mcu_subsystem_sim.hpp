/**
 * @file mcu_subsystem_sim.hpp
 * @brief MCU Subsystem Simulator for Drive, Battery, and Heartbeat
 * @date 2025-12-25
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <mcu_msgs/msg/drive_base.hpp>
#include <mcu_msgs/msg/battery_health.hpp>
#include <chrono>
#include <cmath>

namespace secbot_sim {

class McuSubsystemSimulator : public rclcpp::Node {
public:
  McuSubsystemSimulator();

private:
  // Drive subsystem state
  double pose_x_;
  double pose_y_;
  double pose_theta_;
  double vel_x_;
  double vel_omega_z_;
  rclcpp::Time last_update_time_;

  // Publishers
  rclcpp::Publisher<mcu_msgs::msg::DriveBase>::SharedPtr drive_status_pub_;
  rclcpp::Publisher<mcu_msgs::msg::BatteryHealth>::SharedPtr battery_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr heartbeat_pub_;

  // Subscribers
  rclcpp::Subscription<mcu_msgs::msg::DriveBase>::SharedPtr drive_command_sub_;

  // Timers
  rclcpp::TimerBase::SharedPtr update_timer_;
  rclcpp::TimerBase::SharedPtr drive_publish_timer_;
  rclcpp::TimerBase::SharedPtr battery_publish_timer_;
  rclcpp::TimerBase::SharedPtr heartbeat_publish_timer_;

  // Callbacks
  void driveCommandCallback(const mcu_msgs::msg::DriveBase::SharedPtr msg);
  void updateTimerCallback();
  void drivePublishCallback();
  void batteryPublishCallback();
  void heartbeatPublishCallback();
};

}  // namespace secbot_sim
