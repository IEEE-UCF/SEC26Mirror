#include <rclcpp/rclcpp.hpp>
// #include <gz/msgs/odometry.pb.h>
#include <gz/msgs/pose.pb.h>
#include <gz/msgs/pose_v.pb.h>
#include <stdio.h>

#include <gz/transport/Node.hh>
#include <mcu_msgs/msg/uwb_anchor_info.hpp>
#include <mcu_msgs/msg/uwb_range.hpp>
#include <mcu_msgs/msg/uwb_ranging.hpp>
#include <mutex>
#include <random>
#include <string>

class UWB_Tag_Sim : public rclcpp::Node {
 public:
  UWB_Tag_Sim();

 private:
  gz::transport::Node subscription_to_world;
  gz::msgs::Pose_V latest_position_of_objects;

  void gazeboworldstats_executor_callback(const gz::msgs::Pose_V &msg);
  void rostwo_executor_callback();
  // gz.msgs.Pose
  // gz.msgs.Pose_V
  std::mutex msg_mutex;
  bool has_new_data = false;
  rclcpp::TimerBase::SharedPtr wall_timer;
  rclcpp::Publisher<mcu_msgs::msg::UWBAnchorInfo>::SharedPtr
      anchor_info_publisher;
  std::string tag_name =
      "Drone";  // if you need to know the names of the tags use gz topic -l
  std::default_random_engine tandomGenerator;
  rclcpp::Publisher<mcu_msgs::msg::UWBRanging>::SharedPtr
      uwb_ranges_array_publisher;
};