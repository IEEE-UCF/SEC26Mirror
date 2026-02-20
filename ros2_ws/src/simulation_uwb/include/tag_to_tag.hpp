#ifndef _UWB_SIMULATION_
#define _UWB_SIMULATION_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <random>
#include <std_msgs/msg/float64.hpp>
#include <string>
#include <stdio.h>
#include <gz/msgs/odometry.pb.h>
//#include <gz/transport/Node.hpp>
#include <gz/transport/Node.hh>

class UWBSimulationToTag : public rclcpp::Node
{
public:
    UWBSimulationToTag();

private:
    void src_topic_callback(const gz::msgs::Odometry &_msg);
    void dst_topic_callback(const gz::msgs::Odometry &_msg);
    void simulate();

    gz::transport::Node subscribeNodeSrc;
    gz::transport::Node subscribeNodeDst;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr msgPublisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    geometry_msgs::msg::Point pSrc;
    geometry_msgs::msg::Point pDst;

    std::default_random_engine tandomGenerator;
};

#endif