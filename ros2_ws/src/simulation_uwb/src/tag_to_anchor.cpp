#include "tag_to_anchor.hpp"
#include "rclcpp/rclcpp.hpp"


int main(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UWBSimulation>());
    rclcpp::shutdown();

    return 0;
}

UWBSimulation::UWBSimulation() : Node("uwb_simulation_node")
{
    // TAG + ANCHOR params
    this->declare_parameter("label_name", "black_block2");
    this->declare_parameter("anchor_model", "black_block1");
    this->declare_parameter("anchor_id", 1);

    labelName = this->get_parameter("label_name").as_string();
    anchorModelName_ = this->get_parameter("anchor_model").as_string();
    anchorId_ = this->get_parameter("anchor_id").as_int();

    RCLCPP_INFO(this->get_logger(), "tag(label_name): %s", labelName.c_str());
    RCLCPP_INFO(this->get_logger(), "anchor_model: %s (id=%d)", anchorModelName_.c_str(), anchorId_);

    // Publisher (create early so we know we got here)
    std::string publishTopic = "/uwbData/" + labelName;
    msgPublisher_ = this->create_publisher<uwb_interfaces::msg::UWBData>(publishTopic, 10);
    RCLCPP_INFO(this->get_logger(), "publish topic: %s", publishTopic.c_str());

    // Subscribe to TAG odometry
    std::string tagTopic = "/model/" + labelName + "/odometry";
    RCLCPP_INFO(this->get_logger(), "subscribe tag topic: %s", tagTopic.c_str());

    if (!gzNode_.Subscribe(tagTopic, &UWBSimulation::tag_topic_callback, this))
    {
        RCLCPP_ERROR(this->get_logger(), "Error subscribing to tag topic [%s].", tagTopic.c_str());
    }

    // Subscribe to ANCHOR odometry
    std::string anchorTopic = "/model/" + anchorModelName_ + "/odometry";
    RCLCPP_INFO(this->get_logger(), "subscribe anchor topic: %s", anchorTopic.c_str());

    if (!gzNode_.Subscribe(anchorTopic, &UWBSimulation::anchor_topic_callback, this))
    {
        RCLCPP_ERROR(this->get_logger(), "Error subscribing to anchor topic [%s].", anchorTopic.c_str());
    }
}

void UWBSimulation::anchor_topic_callback(const gz::msgs::Odometry &_msg)
{
    std::lock_guard<std::mutex> lock(anchorMutex_);
    anchorPose_.x = _msg.pose().position().x();
    anchorPose_.y = _msg.pose().position().y();
    anchorPose_.z = _msg.pose().position().z();
    anchorReady_.store(true, std::memory_order_release);
}

void UWBSimulation::tag_topic_callback(const gz::msgs::Odometry &_msg)
{
    if (!anchorReady_.load(std::memory_order_acquire))
        return;

    double x = _msg.pose().position().x();
    double y = _msg.pose().position().y();
    double z = _msg.pose().position().z();

    uwb_simulate(x, y, z);
}

void UWBSimulation::uwb_simulate(double x, double y, double z)
{
    geometry_msgs::msg::Point anchorCopy;
    {
        std::lock_guard<std::mutex> lock(anchorMutex_);
        anchorCopy = anchorPose_;
    }

    double realDistance = std::sqrt(
        (x - anchorCopy.x) * (x - anchorCopy.x) +
        (y - anchorCopy.y) * (y - anchorCopy.y) +
        (z - anchorCopy.z) * (z - anchorCopy.z));

    std::normal_distribution<double> distribution_normal(0., 0.1);
    double simDistance = realDistance + distribution_normal(tandomGenerator);

    uwb_interfaces::msg::UWBData msg;

    uwb_interfaces::msg::UWBDistance d;
    d.dest = anchorId_;          // black_block1 is anchor id 1
    d.distance = simDistance;

    msg.distances.push_back(d);
    msgPublisher_->publish(msg);
}