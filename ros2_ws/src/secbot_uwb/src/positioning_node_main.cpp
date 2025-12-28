// Copyright 2025 SEC26 Team
// UWB Positioning Node Main

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "secbot_uwb/positioning_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<secbot_uwb::UWBPositioningNode>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
