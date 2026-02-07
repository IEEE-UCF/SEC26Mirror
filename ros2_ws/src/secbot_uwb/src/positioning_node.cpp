// Copyright 2025 SEC26 Team
// UWB Positioning Node Implementation

// refactor by trevor cannon on 2-1-2026

#include "secbot_uwb/positioning_node.hpp"

#include <Eigen/Dense>
#include <cmath>
#include <functional>
#include <memory>

namespace secbot_uwb {

UWBPositioningNode::UWBPositioningNode() : Node("uwb_positioning_node") {
  RCLCPP_INFO(this->get_logger(), "Initializing UWB Positioning Node");

  declareParameters();
  loadConfiguration();

  // Create ranging subscription
  ranging_sub_ = this->create_subscription<mcu_msgs::msg::UWBRanging>(
      "mcu_uwb/ranging", 10,
      std::bind(&UWBPositioningNode::rangingCallback, this,
                std::placeholders::_1));

  logConfiguration();
  RCLCPP_INFO(this->get_logger(), "UWB Positioning Node started");
}

void UWBPositioningNode::declareParameters() {
  // Frame IDs
  this->declare_parameter("map_frame", "map");
  this->declare_parameter("odom_frame", "odom");

  // Beacon and tag configuration (will be loaded from YAML)
  this->declare_parameter("beacons", rclcpp::PARAMETER_NOT_SET);
  this->declare_parameter("tags", rclcpp::PARAMETER_NOT_SET);

  // Positioning parameters
  this->declare_parameter("positioning.min_beacons_2d", 3);
  this->declare_parameter("positioning.min_beacons_3d", 4);
  this->declare_parameter("positioning.max_residual", 0.5);
  this->declare_parameter("positioning.outlier_threshold", 2.0);
  this->declare_parameter("positioning.robot_height", 0.20);

  // Covariance parameters
  this->declare_parameter("covariance.base_xy_variance", 0.05);
  this->declare_parameter("covariance.base_z_variance", 0.10);
  this->declare_parameter("covariance.scale_with_residual", true);

  // Publishing parameters
  this->declare_parameter("publishing.rate_hz", 20.0);
  this->declare_parameter("publishing.publish_diagnostics", true);
}

void UWBPositioningNode::loadConfiguration() {
  // Load frames
  map_frame_ = this->get_parameter("map_frame").as_string();
  odom_frame_ = this->get_parameter("odom_frame").as_string();

  // Load positioning parameters
  min_beacons_2d_ = this->get_parameter("positioning.min_beacons_2d").as_int();
  min_beacons_3d_ = this->get_parameter("positioning.min_beacons_3d").as_int();
  max_residual_ = this->get_parameter("positioning.max_residual").as_double();
  outlier_threshold_ =
      this->get_parameter("positioning.outlier_threshold").as_double();
  robot_height_ = this->get_parameter("positioning.robot_height").as_double();

  // Load covariance parameters
  base_xy_variance_ =
      this->get_parameter("covariance.base_xy_variance").as_double();
  base_z_variance_ =
      this->get_parameter("covariance.base_z_variance").as_double();
  scale_with_residual_ =
      this->get_parameter("covariance.scale_with_residual").as_bool();

  // Load beacon configurations
  // In C++, we need to declare parameters individually
  // Check for known beacon IDs (10, 11, 12, 13)
  std::vector<int> beacon_ids = {10, 11, 12, 13, 14, 15};

  for (int beacon_id : beacon_ids) {
    std::string prefix = "beacons." + std::to_string(beacon_id) + ".";

    // Check if this beacon exists by checking if type parameter is set
    if (!this->has_parameter(prefix + "type")) {
      continue;
    }

    try {
      std::string type_str = this->get_parameter(prefix + "type").as_string();
      BeaconType type =
          (type_str == "moving") ? BeaconType::MOVING : BeaconType::STATIONARY;

      // Load position
      std::array<double, 3> position = {
          this->get_parameter(prefix + "position.x").as_double(),
          this->get_parameter(prefix + "position.y").as_double(),
          this->get_parameter(prefix + "position.z").as_double()};

      // Load known axes
      std::vector<std::string> known_axes =
          this->get_parameter(prefix + "known_axes").as_string_array();

      // Load optional parameters
      std::string odometry_topic;
      bool use_odometry_fusion = false;
      std::string description;

      if (this->has_parameter(prefix + "odometry_topic")) {
        odometry_topic =
            this->get_parameter(prefix + "odometry_topic").as_string();
      }
      if (this->has_parameter(prefix + "use_odometry_fusion")) {
        use_odometry_fusion =
            this->get_parameter(prefix + "use_odometry_fusion").as_bool();
      }
      if (this->has_parameter(prefix + "description")) {
        description = this->get_parameter(prefix + "description").as_string();
      }

      // Create beacon config
      beacons_.emplace(beacon_id,
                       BeaconConfig(beacon_id, type, position, known_axes,
                                    odometry_topic, use_odometry_fusion,
                                    description));

    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Error loading beacon %d: %s", beacon_id,
                   e.what());
    }
  }

  // Load tag configurations
  std::vector<int> tag_ids = {12, 20, 21, 22, 23};

  for (int tag_id : tag_ids) {
    std::string prefix = "tags." + std::to_string(tag_id) + ".";

    // Check if this tag exists
    if (!this->has_parameter(prefix + "description")) {
      continue;
    }

    try {
      std::string description =
          this->get_parameter(prefix + "description").as_string();

      std::string publish_topic;
      if (this->has_parameter(prefix + "publish_topic")) {
        publish_topic =
            this->get_parameter(prefix + "publish_topic").as_string();
      }

      bool enable_3d = false;
      if (this->has_parameter(prefix + "enable_3d_positioning")) {
        enable_3d =
            this->get_parameter(prefix + "enable_3d_positioning").as_bool();
      }

      tags_.emplace(tag_id,
                    TagConfig(tag_id, description, publish_topic, enable_3d));

    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Error loading tag %d: %s", tag_id,
                   e.what());
    }
  }
}

void UWBPositioningNode::logConfiguration() {
  RCLCPP_INFO(this->get_logger(), "=== UWB Configuration ===");
  RCLCPP_INFO(this->get_logger(), "Beacons configured: %zu", beacons_.size());

  for (const auto &[beacon_id, beacon] : beacons_) {
    std::string type_str =
        (beacon.getType() == BeaconType::STATIONARY) ? "stationary" : "moving";
    auto pos = beacon.getPosition();
    RCLCPP_INFO(this->get_logger(), "  Beacon %d (%s): pos=[%.2f, %.2f, %.2f]",
                beacon_id, type_str.c_str(), pos[0], pos[1], pos[2]);
  }

  RCLCPP_INFO(this->get_logger(), "Tags configured: %zu", tags_.size());
  for (const auto &[tag_id, tag] : tags_) {
    RCLCPP_INFO(this->get_logger(), "  Tag %d: %s, 3D=%s", tag_id,
                tag.getDescription().c_str(),
                tag.enable3D() ? "true" : "false");
  }

  RCLCPP_INFO(this->get_logger(), "========================");
}

std::array<double, 3>
UWBPositioningNode::getBeaconPosition(int beacon_id) const {
  auto it = beacons_.find(beacon_id);
  if (it == beacons_.end()) {
    return {0.0, 0.0, 0.0};
  }

  return it->second.getPosition();
}

void UWBPositioningNode::rangingCallback(
    const mcu_msgs::msg::UWBRanging::SharedPtr msg) {
  int tag_id = msg->tag_id;

  // Check if tag is configured
  auto tag_it = tags_.find(tag_id);
  if (tag_it == tags_.end()) {
    // Tag not configured, skip
    return;
  }

  const auto &tag_config = tag_it->second;

  // Extract valid ranges
  std::map<int, double> ranges;
  for (const auto &range_msg : msg->ranges) {
    if (range_msg.valid) {
      int beacon_id = range_msg.anchor_id;
      double distance_m = range_msg.distance / 100.0; // Convert cm to meters
      ranges[beacon_id] = distance_m;
    }
  }

  // Check minimum beacons
  int min_beacons = tag_config.enable3D() ? min_beacons_3d_ : min_beacons_2d_;

  if (static_cast<int>(ranges.size()) < min_beacons) {
    RCLCPP_WARN(this->get_logger(),
                "Tag %d: Insufficient beacons (%zu/%d min) for positioning",
                tag_id, ranges.size(), min_beacons);
    return;
  }

  // Store ranges
  tag_ranges_[tag_id] = ranges;

  // Trilaterate
  std::array<double, 3> position;
  double residual;

  if (trilaterate(ranges, tag_config.enable3D(), position, residual)) {
    // Outlier rejection: remove worst beacon if error exceeds threshold
    if (ranges.size() > static_cast<size_t>(min_beacons)) {
      double worst_error = 0.0;
      int worst_id = -1;

      for (const auto &[beacon_id, dist] : ranges) {
        auto bp = getBeaconPosition(beacon_id);
        double dx = position[0] - bp[0];
        double dy = position[1] - bp[1];
        double dz = position[2] - bp[2];
        double predicted = std::sqrt(dx * dx + dy * dy + dz * dz);
        double error = std::abs(predicted - dist);
        if (error > worst_error) {
          worst_error = error;
          worst_id = beacon_id;
        }
      }

      if (worst_error > outlier_threshold_ && worst_id >= 0) {
        RCLCPP_WARN(this->get_logger(),
                    "Tag %d: Rejecting beacon %d (error=%.3fm > %.3fm threshold)",
                    tag_id, worst_id, worst_error, outlier_threshold_);
        ranges.erase(worst_id);
        if (!trilaterate(ranges, tag_config.enable3D(), position, residual)) {
          return;
        }
      }
    }

    // Check residual
    if (residual > max_residual_) {
      RCLCPP_WARN(
          this->get_logger(),
          "Tag %d: High residual error (%.3fm), position may be inaccurate",
          tag_id, residual);
    }

    // Publish pose
    publishPose(tag_id, position, residual, msg->header.stamp, tag_config);
  }
}

bool UWBPositioningNode::trilaterate(const std::map<int, double> &ranges,
                                     bool enable_3d,
                                     std::array<double, 3> &position,
                                     double &residual) {
  // Get beacon positions
  std::map<int, std::array<double, 3>> beacon_positions;
  for (const auto &[beacon_id, distance] : ranges) {
    beacon_positions[beacon_id] = getBeaconPosition(beacon_id);
  }

  std::vector<int> valid_beacons;
  for (const auto &[beacon_id, pos] : beacon_positions) {
    valid_beacons.push_back(beacon_id);
  }

  int min_required = enable_3d ? min_beacons_3d_ : min_beacons_2d_;
  if (static_cast<int>(valid_beacons.size()) < min_required) {
    RCLCPP_WARN(this->get_logger(),
                "Insufficient known beacon positions: %zu/%d",
                valid_beacons.size(), min_required);
    return false;
  }

  // Use first beacon as reference
  int ref_id = valid_beacons[0];
  auto ref_pos = beacon_positions[ref_id];
  double ref_dist = ranges.at(ref_id);

  // Build system of equations
  int n_vars = enable_3d ? 3 : 2;
  int n_equations = valid_beacons.size() - 1;

  Eigen::MatrixXd A(n_equations, n_vars);
  Eigen::VectorXd b(n_equations);

  int row = 0;
  for (size_t i = 1; i < valid_beacons.size(); ++i) {
    int beacon_id = valid_beacons[i];
    auto beacon_pos = beacon_positions[beacon_id];
    double dist = ranges.at(beacon_id);

    // Row of A: 2(a_ref - a_i)
    A(row, 0) = 2.0 * (ref_pos[0] - beacon_pos[0]);
    A(row, 1) = 2.0 * (ref_pos[1] - beacon_pos[1]);

    if (enable_3d)
      A(row, 2) = 2.0 * (ref_pos[2] - beacon_pos[2]);

    // Right-hand side
    double ref_norm_sq = ref_pos[0] * ref_pos[0] + ref_pos[1] * ref_pos[1] +
                         ref_pos[2] * ref_pos[2];
    double beacon_norm_sq = beacon_pos[0] * beacon_pos[0] +
                            beacon_pos[1] * beacon_pos[1] +
                            beacon_pos[2] * beacon_pos[2];

    b(row) = ref_norm_sq - beacon_norm_sq + dist * dist - ref_dist * ref_dist;

    if (!enable_3d) {
      // The term 2*z*(z_ref - z_i) was on the Left Side.
      // We move it to the Right Side (b), so we SUBTRACT it.
      double z_correction = 2.0 * robot_height_ * (ref_pos[2] - beacon_pos[2]);
      b(row) -= z_correction;
    }

    ++row;
  }

  // Solve using least squares
  Eigen::Vector3d solution = A.colPivHouseholderQr().solve(b);

  position[0] = solution(0);
  position[1] = solution(1);

  if (enable_3d) {
    position[2] = solution(2);
  } else {
    position[2] = 0.0f;
  }

  // Calculate residual
  double error_sum = 0.0;
  for (const auto &beacon_id : valid_beacons) {
    auto beacon_pos = beacon_positions[beacon_id];
    double dx = position[0] - beacon_pos[0];
    double dy = position[1] - beacon_pos[1];
    double dz = position[2] - beacon_pos[2];
    double predicted_dist = std::sqrt(dx * dx + dy * dy + dz * dz);
    double error = predicted_dist - ranges.at(beacon_id);
    error_sum += error * error;
  }
  residual = std::sqrt(error_sum / valid_beacons.size());

  return true;
}

void UWBPositioningNode::publishPose(
    int tag_id, const std::array<double, 3> &position, double residual,
    const builtin_interfaces::msg::Time &timestamp,
    const TagConfig &tag_config) {
  // Create publisher if not exists
  if (pose_publishers_.find(tag_id) == pose_publishers_.end()) {
    pose_publishers_[tag_id] =
        this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            tag_config.getPublishTopic(), 10);

    RCLCPP_INFO(this->get_logger(), "Created publisher for tag %d (%s) on %s",
                tag_id, tag_config.getDescription().c_str(),
                tag_config.getPublishTopic().c_str());
  }

  // Create pose message
  auto pose_msg = geometry_msgs::msg::PoseWithCovarianceStamped();
  pose_msg.header.stamp = timestamp;
  pose_msg.header.frame_id = map_frame_;

  // Set position
  pose_msg.pose.pose.position.x = position[0];
  pose_msg.pose.pose.position.y = position[1];
  pose_msg.pose.pose.position.z = position[2];

  // Set orientation (identity quaternion)
  pose_msg.pose.pose.orientation.w = 1.0;
  pose_msg.pose.pose.orientation.x = 0.0;
  pose_msg.pose.pose.orientation.y = 0.0;
  pose_msg.pose.pose.orientation.z = 0.0;

  // Set covariance
  double xy_var = base_xy_variance_;
  double z_var = base_z_variance_;

  if (scale_with_residual_ && residual > 0) {
    double scale_factor = 1.0 + (residual / max_residual_);
    xy_var *= scale_factor;
    z_var *= scale_factor;
  }

  std::fill(pose_msg.pose.covariance.begin(), pose_msg.pose.covariance.end(),
            0.0);
  pose_msg.pose.covariance[0] = xy_var; // var(x)
  pose_msg.pose.covariance[7] = xy_var; // var(y)
  pose_msg.pose.covariance[14] = z_var; // var(z)

  // Publish
  pose_publishers_[tag_id]->publish(pose_msg);
}
} // namespace secbot_uwb

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<secbot_uwb::UWBPositioningNode>());
  rclcpp::shutdown();
  return 0;
}