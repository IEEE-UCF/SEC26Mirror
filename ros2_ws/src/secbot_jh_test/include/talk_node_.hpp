//check out in the terminal ros2 interface show std_msgs/msg/String
//if you can you can also look at the list of standard messages developed by ROS
//ros2 interface list | grep std_msgs
#include <chrono>
#include <memory>
#include <string>
#include <std_msgs/msg/string.hpp>
#include <rclcpp/rclcpp.hpp> //like the ros1 ros/ros.h header file used for
//intializing ros nodes, creating handles for ros systems, 
//if you are to create your own setup you need to include your dependecies on the
//package xml file and the cmakelist.txt file

//apparently this namespace is for the use of time to represent
//hours,minutes,seconds,miliseconds,microseconds and nanoseconds
using namespace std::chrono_literals;

/*
This example creates a subclass of Node and uses a fancy C++11 lambda function
to shorten the callback syntax(the execeutor), at the expense of making the code a little
scary at first
*/


class firstPublisher : public rclcpp::Node{//creates the node class by inheriting rclcpp::Node
//everytime we say the word 'this' we are referencing to the node
    public:
    //constructor gives name of the Node(hit ctrl+click to the files you'll see...) and then initializes the private member 'count' to the value zero
    firstPublisher() : Node("name_of_publisher"),count(0){
        publisher_object = this->create_publisher<std_msgs::msg::String>("my_first_topic",10);//publisher_object..type=String...name="my_first_topic"...size=10 bytes?
        auto executor_callback = 
        [this]()->void{
            auto message  = std_msgs::msg::String();
            message.data = "Hello, World! "+ std::to_string(this->count++);
            RCLCPP_INFO(this->get_logger(), "Publishng: '%s'", message.data.c_str());
            this->publisher_object->publish(message);
        };
        timer = this->create_wall_timer(500ms,executor_callback);//call the executor every 500ms
    }
    private:
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_object;//a publisher of type String...so I guess it can only send strings...
    size_t count;
};
/*
Inside the constructor, the publisher is initialized with the string message type, given a topic
name, and the required queue size to limit messages in the event of a backup.(literally just like FreeRTOS and THEIR QUEUES!!)

The executor_callback function creates new message of type String, sets its data with the desired string and publishes it.//does every 1/2 a second
The RCLCPP_INFO macro ensures every published message is printed to the console. At last, timer is initialized, which causes the
executor_callback function to be execueted twice a second
*/
