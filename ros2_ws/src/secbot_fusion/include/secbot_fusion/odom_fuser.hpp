/**
 * @file odom_fuser.hpp
 * @brief Uses encoders and IMU yaw to build odometry.
 * @author Trevor Cannon
 * @date 1/25/2026
 */

#pragma once

#include "mcu_msgs/msg/drive_base.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

namespace secbot_fusion {

class OdomFuser {
public:
  OdomFuser() : {};

  nav_msgs::msg::Odometry
  convert_to_odom(const mcu_msgs::msg : DriveBase::SharedPtr msg);

private:
}
} // namespace secbot_fusion