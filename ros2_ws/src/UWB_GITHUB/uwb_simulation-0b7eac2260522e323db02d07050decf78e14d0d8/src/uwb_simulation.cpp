#include "uwb_simulation.hpp"

UWBSimulation::UWBSimulation() : Node("uwb_simulation_node") {
  load_config();

  this->declare_parameter("label_name", "x500_0");
  label_name_ = this->get_parameter("label_name").as_string();
  RCLCPP_INFO(this->get_logger(), "label_name: %s", label_name_.c_str());

  // Subscribe to Gazebo ground truth odometry via gz::transport
  std::string gz_topic = "/model/" + label_name_ + "/odometry";
  RCLCPP_INFO(this->get_logger(), "Subscribing to Gazebo topic: %s",
              gz_topic.c_str());

  if (!gz_node_.Subscribe(gz_topic, &UWBSimulation::topic_callback, this)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to subscribe to [%s]",
                 gz_topic.c_str());
  }

  // Publish on the topic that positioning_node subscribes to
  ranging_pub_ =
      this->create_publisher<mcu_msgs::msg::UWBRanging>("mcu_uwb/ranging", 10);
  RCLCPP_INFO(this->get_logger(), "Publishing UWBRanging on /mcu_uwb/ranging");
}

void UWBSimulation::topic_callback(const gz::msgs::Odometry& msg) {
  double x = msg.pose().position().x();
  double y = msg.pose().position().y();
  double z = msg.pose().position().z();
  uwb_simulate(x, y, z);
}

void UWBSimulation::load_config() {
  std::string pkg_dir =
      ament_index_cpp::get_package_share_directory("uwb_simulation");
  std::string config_path = pkg_dir + "/config/anchor.xml";

  tinyxml2::XMLDocument doc;
  if (doc.LoadFile(config_path.c_str()) != 0) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load anchor config: %s",
                 config_path.c_str());
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Loaded anchor config: %s",
              config_path.c_str());

  tinyxml2::XMLElement* anchor =
      doc.RootElement()->FirstChildElement("anchor");

  while (anchor) {
    int id = atoi(anchor->FirstAttribute()->Value());
    tinyxml2::XMLElement* attr = anchor->FirstChildElement();

    geometry_msgs::msg::Point p;
    p.set__x(std::stod(attr->GetText()));
    attr = attr->NextSiblingElement();
    p.set__y(std::stod(attr->GetText()));
    attr = attr->NextSiblingElement();
    p.set__z(std::stod(attr->GetText()));

    RCLCPP_INFO(this->get_logger(), "Anchor %d: [%.2f, %.2f, %.2f]", id, p.x,
                p.y, p.z);
    anchor_positions_[id] = p;
    anchor = anchor->NextSiblingElement();
  }
}

void UWBSimulation::uwb_simulate(double x, double y, double z) {
  // Compute true distances to each anchor (meters)
  std::unordered_map<int, double> true_dist;
  for (const auto& [id, pos] : anchor_positions_) {
    true_dist[id] =
        std::sqrt(std::pow(x - pos.x, 2) + std::pow(y - pos.y, 2) +
                  std::pow(z - pos.z, 2));
  }

  // Add gaussian noise
  std::normal_distribution<double> noise(0.0, NOISE_STDDEV_M);
  std::unordered_map<int, double> noisy_dist;
  for (const auto& [id, dist] : true_dist) {
    noisy_dist[id] = dist + noise(rng_);
  }

  // Build UWBRanging message matching mcu_msgs format
  auto now = this->get_clock()->now();
  mcu_msgs::msg::UWBRanging ranging_msg;
  ranging_msg.header.stamp = now;
  ranging_msg.header.frame_id = "map";
  ranging_msg.tag_id = DRONE_TAG_ID;
  ranging_msg.temperature = SIM_TEMPERATURE_C;

  for (const auto& [id, dist] : noisy_dist) {
    mcu_msgs::msg::UWBRange range;
    range.header.stamp = now;
    range.header.frame_id = "map";
    range.tag_id = DRONE_TAG_ID;
    range.anchor_id = static_cast<uint8_t>(id);
    range.distance = static_cast<float>(dist * 100.0);  // meters -> cm
    range.signal_strength = SIM_SIGNAL_STRENGTH;
    range.clock_offset = 0;
    range.tx_timestamp = static_cast<uint64_t>(now.nanoseconds());
    range.rx_timestamp = static_cast<uint64_t>(now.nanoseconds());
    range.valid = true;
    range.error_code = 0;
    ranging_msg.ranges.push_back(range);
  }

  ranging_msg.num_anchors = static_cast<uint8_t>(ranging_msg.ranges.size());
  ranging_pub_->publish(ranging_msg);
}
