#ifndef _UWB_SIMULATION_
#define _UWB_SIMULATION_

#include "rclcpp/rclcpp.hpp"
#include "tinyxml2.hpp"
#include <unordered_map>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <random>
#include "uwb_interfaces/msg/uwb_data.hpp"
#include "uwb_interfaces/msg/uwb_distance.hpp"
#include <string>
#include <gz/msgs/odometry.pb.h>
#include <gz/transport/Node.hh>

#include <geometry_msgs/msg/point.hpp>
#include <atomic>
#include <mutex>


class UWBSimulation : public rclcpp::Node
{
public:
    UWBSimulation();

private:
    // Tag + Anchor callbacks
    void tag_topic_callback(const gz::msgs::Odometry &_msg);
    void anchor_topic_callback(const gz::msgs::Odometry &_msg);

    void uwb_simulate(double tagX, double tagY, double tagZ);

    // ONE gz node for all subscriptions
    gz::transport::Node gzNode_;

    std::string labelName;            // tag model name
    std::string anchorModelName_;     // anchor model name
    int anchorId_{1};

    geometry_msgs::msg::Point anchorPose_{};
    std::atomic<bool> anchorReady_{false};
    std::mutex anchorMutex_;

    std::default_random_engine tandomGenerator;

    rclcpp::Publisher<uwb_interfaces::msg::UWBData>::SharedPtr msgPublisher_;
};

#endif