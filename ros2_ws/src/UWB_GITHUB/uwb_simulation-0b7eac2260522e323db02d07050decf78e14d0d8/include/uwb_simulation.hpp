#ifndef UWB_SIMULATION_HPP
#define UWB_SIMULATION_HPP

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <gz/msgs/odometry.pb.h>
#include <gz/transport/Node.hh>
#include <mcu_msgs/msg/uwb_range.hpp>
#include <mcu_msgs/msg/uwb_ranging.hpp>
#include <random>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <unordered_map>

#include "tinyxml2.hpp"

class UWBSimulation : public rclcpp::Node {
 public:
  UWBSimulation();

 private:
  void topic_callback(const gz::msgs::Odometry& msg);
  void uwb_simulate(double x, double y, double z);
  void load_config();

  gz::transport::Node gz_node_;
  std::string label_name_;
  std::unordered_map<int, geometry_msgs::msg::Point> anchor_positions_;
  std::default_random_engine rng_;
  rclcpp::Publisher<mcu_msgs::msg::UWBRanging>::SharedPtr ranging_pub_;

  static constexpr uint8_t DRONE_TAG_ID = 20;
  static constexpr double NOISE_STDDEV_M = 0.1;
  static constexpr float SIM_TEMPERATURE_C = 25.0f;
  static constexpr float SIM_SIGNAL_STRENGTH = -40.0f;
};

#endif  // UWB_SIMULATION_HPP
