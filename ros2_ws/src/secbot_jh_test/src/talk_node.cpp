#include "talk_node_.hpp"
int main(int argc, char *argv[]){
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<firstPublisher>());
    rclcpp::shutdown();
    return 0;
}
/*After setting the preliminaries in the header file for the publisher(including the publisher_object, the message, the executor callback function, plus the timer...)
we can allow the node to live here in the .cpp file and allow it to be executed..
rclcpp::init initializes ROS2, and rclcpp::spin starts processing data from the node, INCLUDING callbacks from the timer
*/