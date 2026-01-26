#include "odom_fuser.hpp"

nav_msgs::msg::Odometry secbot_fusion::OdomFuser::convert_to_odom(
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
}