#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class firstSubscriber: public rclcpp::Node{
    public:
    firstSubscriber(): Node("name_of_subscriber_node"){
        auto executor_callback = 
        [this](std_msgs::msg::String::UniquePtr msg)->void{
            RCLCPP_INFO(this->get_logger(),"I hear: '%s'", msg->data.c_str());//also small note the data variable you see there that is inside
            //the msg file remember do ros2 interface show std_msgs/msg/String to see it...
        };
        subscription_object = this->create_subscription<std_msgs::msg::String>("my_first_topic",10,executor_callback);
    }
    private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_object;
};
/*
So nearly identical to the publisher's code. There is NO timer because the subscriber responds whenever data IS published
to the topic(literally like a Interrupt Service Routine[ISR] from FreeRTOS)
The executor_callback function receives the string messaga data published over the topic and simply writes it to the console using
the RCLCPP_INFO macro

REMEMBER the topic name and message type used BY THE PUBLISHER AND SUBSCRIBER must match to allow them to communicate
*/