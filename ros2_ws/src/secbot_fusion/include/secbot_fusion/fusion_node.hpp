/**
 * @file fusion_node.hpp
 * @brief Wires everything and publishes nav_msgs/Odometry.
 * @author Trevor Cannon
 * @date 1/25/2026
 */

#pragma once

#include "mcu_msgs/msg/drive_base.hpp"
#include "nav_msgs/msg/odometry"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/sensor_msgs/imu.hpp"

#include "ekf.hpp"
#include "odom_fuser.hpp"
#include "tf_broadcaster.hpp"

namespace secbot_fusion {
class FusionNode : public rclcpp::Node {
public:
  FusionNode() : Node("fusion_node") {}

private:
}
} // namespace secbot_fusion
