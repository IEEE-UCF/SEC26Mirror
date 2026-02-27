#include "uwb_location.hpp"

UWBLocation::UWBLocation() : Node("uwb_location")
{
    this->declare_parameter("label_name", "x500_0");
    std::string labelName =
        this->get_parameter("label_name").get_parameter_value().get<std::string>();

    this->load_anchors_pos();

    std::string subscribeTopicName = "/uwbData/" + labelName;
    RCLCPP_INFO(this->get_logger(), "subscribe topic : %s", subscribeTopicName.c_str());

    subscription_ = this->create_subscription<uwb_interfaces::msg::UWBData>(
        subscribeTopicName, 10, std::bind(&UWBLocation::topic_callback, this, std::placeholders::_1));

    std::string publishTopic = "/uwbLocationRes/" + labelName;
    msgPublisher_ = this->create_publisher<geometry_msgs::msg::Point>(publishTopic, 10);
    RCLCPP_INFO(this->get_logger(), "publish topic : %s", publishTopic.c_str());
}

void UWBLocation::topic_callback(const uwb_interfaces::msg::UWBData::SharedPtr msg)
{

    // std::string labelName = msg->label_name;

    std::unordered_map<int, double> uwbDistance;
    for (long unsigned int i = 0; i < msg->distances.size(); i++)
    {
        uwbDistance[msg->distances[i].anchor_id] = msg->distances[i].distance;
    }

    Position3D estimatedRes;
    LOCATOR_RETURN calState;
    calState = calculate_pos_robust_ransac(anchorPoseMap, uwbDistance, estimatedRes);
    if (calState == CALCULATE_SUCCESS)
    {
        // RCLCPP_INFO(this->get_logger(), "uwb location success. res: x: %f, y: %f, z:%f ",
        //             estimatedRes.x, estimatedRes.y, estimatedRes.z);
        geometry_msgs::msg::Point data;
        data.set__x(estimatedRes.x);
        data.set__y(estimatedRes.y);
        data.set__z(estimatedRes.z);

        msgPublisher_->publish(data);
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "uwb location failed (type: %d). ", calState);
    }
}

void UWBLocation::load_anchors_pos()
{
    std::string packageShareDirectory = ament_index_cpp::get_package_share_directory("uwb_locate");

    std::string anchorConfigFilePath = packageShareDirectory + "/config/anchor.xml";

    tinyxml2::XMLDocument doc;

    if (doc.LoadFile(anchorConfigFilePath.c_str()) != 0)
    {
        RCLCPP_INFO(this->get_logger(), "load anchor config file failed.");
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "load anchor config file successed.");
    }

    tinyxml2::XMLElement *anchor = doc.RootElement()->FirstChildElement("anchor");

    while (anchor)
    {
        int id = atoi(anchor->FirstAttribute()->Value());

        tinyxml2::XMLElement *attr = anchor->FirstChildElement();

        // geometry_msgs::msg::Point p;
        // p.set__x(std::stod(attr->GetText()));
        // attr = attr->NextSiblingElement();
        // p.set__y(std::stod(attr->GetText()));
        // attr = attr->NextSiblingElement();
        // p.set__z(std::stod(attr->GetText()));

        Position3D p;
        p.x = std::stod(attr->GetText());
        attr = attr->NextSiblingElement();
        p.y = std::stod(attr->GetText());
        attr = attr->NextSiblingElement();
        p.z = std::stod(attr->GetText());

        anchorPoseMap[id] = p;

        RCLCPP_INFO(this->get_logger(), "load anchor id: %d position:%f %f %f", id, anchorPoseMap[id].x, anchorPoseMap[id].y, anchorPoseMap[id].z);

        anchor = anchor->NextSiblingElement();
    }
}
