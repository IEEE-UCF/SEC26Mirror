#include "tag_to_tag.hpp"

int main(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UWBSimulationToTag>());
    rclcpp::shutdown();

    return 0;
}

UWBSimulationToTag::UWBSimulationToTag() : Node("uwb_simulation_node")
{
    this->declare_parameter("src_name", "x500_0");
    this->declare_parameter("dst_name", "x500_1");

    std::string srcName = this->get_parameter("src_name").get_parameter_value().get<std::string>();
    std::string dstName = this->get_parameter("dst_name").get_parameter_value().get<std::string>();
    std::string subscribeTopicSrc = "/model/" + srcName + "/odometry";
    std::string subscribeTopicDst = "/model/" + dstName + "/odometry";
    std::string publishTopic = "/uwbDataToTag/" + srcName + "/" + dstName;
    msgPublisher_ = this->create_publisher<std_msgs::msg::Float64>(publishTopic, 10);

    RCLCPP_INFO(this->get_logger(), "src_name: %s", srcName.c_str());
    RCLCPP_INFO(this->get_logger(), "dst_name: %s", dstName.c_str());
    RCLCPP_INFO(this->get_logger(), "subscribe src topic : %s", subscribeTopicSrc.c_str());
    RCLCPP_INFO(this->get_logger(), "subscribe dst topic : %s", subscribeTopicDst.c_str());
    RCLCPP_INFO(this->get_logger(), "publish topic : %s", publishTopic.c_str());

    if (!subscribeNodeSrc.Subscribe(subscribeTopicSrc, &UWBSimulationToTag::src_topic_callback, this))
    {
        RCLCPP_ERROR(this->get_logger(), "Error subscribing to topic [%s].", subscribeTopicSrc.c_str());
    }

    if (!subscribeNodeDst.Subscribe(subscribeTopicDst, &UWBSimulationToTag::dst_topic_callback, this))
    {
        RCLCPP_ERROR(this->get_logger(), "Error subscribing to topic [%s].", subscribeTopicDst.c_str());
    }
    
    using namespace std::chrono_literals;
    timer_ = this->create_wall_timer(
        20ms, std::bind(&UWBSimulationToTag::simulate, this));
}

void UWBSimulationToTag::src_topic_callback(const gz::msgs::Odometry &_msg)
{
    pSrc.x = _msg.pose().position().x();
    pSrc.y = _msg.pose().position().y();
    pSrc.z = _msg.pose().position().z();

    // RCLCPP_INFO(this->get_logger(), "position: %f %f %f", x, y, z);

    // this->uwb_simulate(x, y, z);
}
void UWBSimulationToTag::dst_topic_callback(const gz::msgs::Odometry &_msg)
{
    pDst.x = _msg.pose().position().x();
    pDst.y = _msg.pose().position().y();
    pDst.z = _msg.pose().position().z();
}
void UWBSimulationToTag::simulate()
{

    double realDistance = sqrtf(
        pow((pSrc.x - pDst.x), 2) +
        pow((pSrc.y - pDst.y), 2) +
        pow((pSrc.z - pDst.z), 2));

    std::normal_distribution<double> distribution_normal(0., 0.1);

    double simDistance = realDistance + distribution_normal(tandomGenerator);

    RCLCPP_INFO(this->get_logger(), "real distance : %f sim distance : %f.", realDistance, simDistance);

    std_msgs::msg::Float64 msg;

    msg.data = simDistance;

    msgPublisher_->publish(msg);
}
