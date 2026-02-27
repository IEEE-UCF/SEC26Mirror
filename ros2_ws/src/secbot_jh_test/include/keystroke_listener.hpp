#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/char.hpp>
class keystroke_subscriber : public rclcpp::Node{
    public:
    keystroke_subscriber():Node("key_stroke_subscriber"){
        auto keystroke_checker = [this](std_msgs::msg::Char::UniquePtr received_msg)->void{//callback function for subscriber
            RCLCPP_INFO(this->get_logger(), "Key pressed: %c",received_msg->data);
        };
        keystroke_subscriber_object = this->create_subscription<std_msgs::msg::Char>("keystroke_topic",10,keystroke_checker);
    };
    private:

    rclcpp::Subscription<std_msgs::msg::Char>::SharedPtr keystroke_subscriber_object;
};