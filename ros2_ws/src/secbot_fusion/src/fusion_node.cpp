/**
 * @file fusion_node.cpp
 * @brief Converts drive-base message into odom, publishes to /odom topic
 * @author Trevor Cannon
 * @date 1/25/2026
 */

#include <memory>

#include "mcu_msgs/msg/drive_base.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

namespace secbot_fusion {
class FusionNode : public rclcpp::Node {
public:
  FusionNode() : Node("fusion_node") {
    drive_base_sub_ = create_subscription<mcu_msgs::msg::DriveBase>(
        "/drive_base/status", rclcpp::QoS(10),
        std::bind(&FusionNode::drive_base_callback, this,
                  std::placeholders::_1));
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom/unfiltered",
                                                          rclcpp::QoS(10));
  }

private:
  rclcpp::Subscription<mcu_msgs::msg::DriveBase>::SharedPtr drive_base_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

  void drive_base_callback(const mcu_msgs::msg::DriveBase::SharedPtr msg) {
    nav_msgs::msg::Odometry odom = convert_to_odom(msg);
    odom_pub_->publish(odom);
  };

  nav_msgs::msg::Odometry
  convert_to_odom(const mcu_msgs::msg::DriveBase::SharedPtr msg) {
    nav_msgs::msg::Odometry odom;

    odom.header.stamp = msg->header.stamp;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";

    odom.pose.pose.position.x = msg->transform.transform.translation.x;
    odom.pose.pose.position.y = msg->transform.transform.translation.y;
    odom.pose.pose.position.z = msg->transform.transform.translation.z;

    odom.pose.pose.orientation = msg->transform.transform.rotation;

    odom.twist.twist = msg->twist;

    odom.twist.covariance[0] = 0.01;  // vx
    odom.twist.covariance[7] = 0.01;  // vy
    odom.twist.covariance[35] = 0.01; // v_yaw

    odom.pose.covariance[0] = 0.1;  // x
    odom.pose.covariance[7] = 0.1;  // y
    odom.pose.covariance[35] = 0.1; // z

    return odom;
  };
};
} // namespace secbot_fusion

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<secbot_fusion::FusionNode>());
  rclcpp::shutdown();
  return 0;
}