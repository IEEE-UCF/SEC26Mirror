#ifndef _UWB_SIMULATION_
#define _UWB_SIMULATION_

#include <gz/msgs/odometry.pb.h>
#include <stdio.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <gz/transport/Node.hh>
#include <random>
#include <string>
#include <unordered_map>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tinyxml2.hpp"
#include "uwb_interfaces/msg/uwb_data.hpp"
#include "uwb_interfaces/msg/uwb_distance.hpp"
class UWBSimulation : public rclcpp::Node {
 public:
  UWBSimulation();

 private:
  void topic_callback(const gz::msgs::Odometry &_msg);
  void uwb_simulate(double x, double y, double z);
  void load_config();

  gz::transport::Node subscribeNode;

  std::string labelName;

  std::string target_frame_;

  std::unordered_map<int, geometry_msgs::msg::Point> anchorPoseMap;

  std::default_random_engine tandomGenerator;

  rclcpp::Publisher<uwb_interfaces::msg::UWBData>::SharedPtr msgPublisher_;
};

#endif