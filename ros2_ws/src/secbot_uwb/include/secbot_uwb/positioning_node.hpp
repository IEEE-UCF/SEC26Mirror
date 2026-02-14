// Copyright 2025 SEC26 Team
// UWB Positioning Node

#ifndef SECBOT_UWB__POSITIONING_NODE_HPP_
#define SECBOT_UWB__POSITIONING_NODE_HPP_

#include <array>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "mcu_msgs/msg/uwb_range.hpp"
#include "mcu_msgs/msg/uwb_ranging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "secbot_uwb/beacon_config.hpp"

namespace secbot_uwb {

class UWBPositioningNode : public rclcpp::Node {
 public:
  UWBPositioningNode();
  ~UWBPositioningNode() = default;

 private:
  // Configuration
  void declareParameters();
  void loadConfiguration();
  void logConfiguration();

  // Callbacks
  void rangingCallback(const mcu_msgs::msg::UWBRanging::SharedPtr msg);

  // Positioning algorithms
  std::array<double, 3> getBeaconPosition(int beacon_id) const;
  bool trilaterate(const std::map<int, double> &ranges, bool enable_3d,
                   std::array<double, 3> &position, double &residual);

  void publishPose(int tag_id, const std::array<double, 3> &position,
                   double residual,
                   const builtin_interfaces::msg::Time &timestamp,
                   const TagConfig &tag_config);

  // Parameters
  std::string map_frame_;
  std::string odom_frame_;

  int min_beacons_2d_;
  int min_beacons_3d_;
  double max_residual_;
  double outlier_threshold_;
  double robot_height_;

  double base_xy_variance_;
  double base_z_variance_;
  bool scale_with_residual_;

  // Configuration storage
  std::map<int, BeaconConfig> beacons_;
  std::map<int, TagConfig> tags_;

  // ROS2 communication
  rclcpp::Subscription<mcu_msgs::msg::UWBRanging>::SharedPtr ranging_sub_;
  std::map<int, rclcpp::Publisher<
                    geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr>
      pose_publishers_;

  // Data storage
  std::map<int, std::map<int, double>> tag_ranges_;
};
}  // namespace secbot_uwb

#endif  // SECBOT_UWB__POSITIONING_NODE_HPP_
