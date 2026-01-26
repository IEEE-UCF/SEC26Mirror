/**
 * @file fusion_node.hpp
 * @brief Wires everything and publishes nav_msgs/Odometry.
 * @author Trevor Cannon
 * @date 1/25/2026
 */

#pragma once

#include <memory>

#include "mcu_msgs/msg/drive_base.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "secbot_fusion/odom_fuser.hpp"
#include "secbot_fusion/tf_broadcaster.hpp"

namespace secbot_fusion {
class FusionNode : public rclcpp::Node {
 public:
  FusionNode() : Node("fusion_node") {
    drive_base_sub_ = create_subscription<mcu_msgs::msg::DriveBase>(
        "drive_base/status", rclcpp::QoS(10),
        std::bind(&FusionNode::drive_base_callback, this,
                  std::placeholders::_1));
    odom_pub_ =
        create_publisher<nav_msgs::msg::Odometry>("/odom", rclcpp::QoS(10));
  }

 private:
  rclcpp::Subscription<mcu_msgs::msg::DriveBase>::SharedPtr drive_base_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

  void drive_base_callback(const mcu_msgs::msg::DriveBase msg) {};

  nav_msgs::msg::Odometry convert_to_odom(
      const mcu_msgs::msg::DriveBase::SharedPtr msg) {
    nav_msgs::msg::Odometry odom;

    odom.header.stamp = msg->header.stamp;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";

    odom.pose.pose.position.x = msg->transform.transform.translation.x;
    odom.pose.pose.position.y = msg->transform.transform.translation.y;
    odom.pose.pose.position.z = msg->transform.transform.translation.z;

    odom.pose.pose.orientation = msg->transform.transform.rotation;

    odom.twist.twist = msg->twist;

    return odom;
  };

  void handle_drive_base(const mcu_msgs::msg::DriveBase msg) {
    nav_msgs::msg::Odometry odom = odom_fuser_->convert_to_odom(msg);
    odom_pub_->publish(odom);
  };
};
}  // namespace secbot_fusion
