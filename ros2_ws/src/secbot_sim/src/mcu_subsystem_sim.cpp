/**
 * @file mcu_subsystem_sim.cpp
 * @brief MCU Subsystem Simulator Implementation
 * @date 2025-12-25
 */

#include "secbot_sim/mcu_subsystem_sim.hpp"

namespace secbot_sim {

McuSubsystemSimulator::McuSubsystemSimulator()
    : Node("mcu_subsystem_sim"),
      pose_x_(0.0),
      pose_y_(0.0),
      pose_theta_(0.0),
      vel_x_(0.0),
      vel_omega_z_(0.0) {

  RCLCPP_INFO(this->get_logger(), "Starting MCU Subsystem Simulator");

  // Initialize last update time
  last_update_time_ = this->now();

  // Create publishers
  drive_status_pub_ = this->create_publisher<mcu_msgs::msg::DriveBase>(
      "drive_base/status", 10);

  battery_pub_ = this->create_publisher<mcu_msgs::msg::BatteryHealth>(
      "/mcu_robot/battery_health", 10);

  heartbeat_pub_ = this->create_publisher<std_msgs::msg::String>(
      "/mcu_robot/heartbeat", 10);

  // Create subscriber
  drive_command_sub_ = this->create_subscription<mcu_msgs::msg::DriveBase>(
      "drive_base/command", 10,
      std::bind(&McuSubsystemSimulator::driveCommandCallback, this, std::placeholders::_1));

  // Create timers
  // Update timer @ 100 Hz (10 ms)
  update_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(10),
      std::bind(&McuSubsystemSimulator::updateTimerCallback, this));

  // Drive publish timer @ 1 Hz (1000 ms)
  drive_publish_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(1000),
      std::bind(&McuSubsystemSimulator::drivePublishCallback, this));

  // Battery publish timer @ 1 Hz (1000 ms)
  battery_publish_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(1000),
      std::bind(&McuSubsystemSimulator::batteryPublishCallback, this));

  // Heartbeat publish timer @ 5 Hz (200 ms)
  heartbeat_publish_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(200),
      std::bind(&McuSubsystemSimulator::heartbeatPublishCallback, this));

  RCLCPP_INFO(this->get_logger(), "MCU Subsystem Simulator initialized");
}

void McuSubsystemSimulator::driveCommandCallback(
    const mcu_msgs::msg::DriveBase::SharedPtr msg) {

  // Only handle DRIVE_VECTOR mode (velocity commands)
  if (msg->drive_mode == mcu_msgs::msg::DriveBase::DRIVE_VECTOR) {
    vel_x_ = msg->goal_velocity.linear.x;
    vel_omega_z_ = msg->goal_velocity.angular.z;

    RCLCPP_DEBUG(this->get_logger(),
                 "Drive command received: vx=%.2f, omega=%.2f",
                 vel_x_, vel_omega_z_);
  } else if (msg->drive_mode == mcu_msgs::msg::DriveBase::DRIVE_GOAL) {
    // For DRIVE_GOAL mode, we could implement a simple controller
    // For now, just log that we received it
    RCLCPP_WARN_ONCE(this->get_logger(),
                     "DRIVE_GOAL mode not implemented in simulator, use DRIVE_VECTOR");
  }
}

void McuSubsystemSimulator::updateTimerCallback() {
  // Get current time and calculate dt
  rclcpp::Time current_time = this->now();
  double dt = (current_time - last_update_time_).seconds();
  last_update_time_ = current_time;

  // Integrate velocity to update pose (simple dead reckoning)
  // For a differential drive robot:
  // dx = v * cos(theta) * dt
  // dy = v * sin(theta) * dt
  // dtheta = omega * dt

  pose_x_ += vel_x_ * std::cos(pose_theta_) * dt;
  pose_y_ += vel_x_ * std::sin(pose_theta_) * dt;
  pose_theta_ += vel_omega_z_ * dt;

  // Normalize theta to [-pi, pi]
  while (pose_theta_ > M_PI) pose_theta_ -= 2.0 * M_PI;
  while (pose_theta_ < -M_PI) pose_theta_ += 2.0 * M_PI;
}

void McuSubsystemSimulator::drivePublishCallback() {
  auto msg = mcu_msgs::msg::DriveBase();

  // Set header
  msg.header.stamp = this->now();
  msg.header.frame_id = "odom";

  // Set transform (pose)
  msg.transform.header.stamp = msg.header.stamp;
  msg.transform.child_frame_id = "base_link";
  msg.transform.transform.translation.x = pose_x_;
  msg.transform.transform.translation.y = pose_y_;
  msg.transform.transform.translation.z = 0.0;

  // Convert theta to quaternion
  double qz = std::sin(pose_theta_ / 2.0);
  double qw = std::cos(pose_theta_ / 2.0);
  msg.transform.transform.rotation.x = 0.0;
  msg.transform.transform.rotation.y = 0.0;
  msg.transform.transform.rotation.z = qz;
  msg.transform.transform.rotation.w = qw;

  // Set twist (velocity)
  msg.twist.linear.x = vel_x_;
  msg.twist.linear.y = 0.0;
  msg.twist.linear.z = 0.0;
  msg.twist.angular.x = 0.0;
  msg.twist.angular.y = 0.0;
  msg.twist.angular.z = vel_omega_z_;

  // Set drive mode (we're always in DRIVE_VECTOR mode for simulation)
  msg.drive_mode = mcu_msgs::msg::DriveBase::DRIVE_VECTOR;

  drive_status_pub_->publish(msg);

  RCLCPP_DEBUG(this->get_logger(),
               "Drive status published: x=%.2f, y=%.2f, theta=%.2f",
               pose_x_, pose_y_, pose_theta_);
}

void McuSubsystemSimulator::batteryPublishCallback() {
  auto msg = mcu_msgs::msg::BatteryHealth();

  // Set header
  msg.header.stamp = this->now();
  msg.header.frame_id = "base_link";

  // Static battery values
  msg.voltage = 12.0;           // 12V
  msg.shunt_voltage = 0.0;      // No shunt voltage in simulation
  msg.current = 1.0;            // 1A
  msg.temperature = 25.0;       // 25°C
  msg.power = 12.0;             // 12W
  msg.energy = 0.0;             // Not simulated
  msg.charge_use = 0.0;         // Not simulated

  battery_pub_->publish(msg);

  RCLCPP_DEBUG(this->get_logger(), "Battery health published");
}

void McuSubsystemSimulator::heartbeatPublishCallback() {
  auto msg = std_msgs::msg::String();
  msg.data = "HEARTBEAT";

  heartbeat_pub_->publish(msg);

  RCLCPP_DEBUG(this->get_logger(), "Heartbeat published");
}

}  // namespace secbot_sim

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<secbot_sim::McuSubsystemSimulator>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
