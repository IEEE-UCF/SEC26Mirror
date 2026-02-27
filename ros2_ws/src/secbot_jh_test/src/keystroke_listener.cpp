#include "keystroke_listener.hpp"
int main(int argc, char *argv[]){
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<keystroke_subscriber>());
    rclcpp::shutdown();
    return 0;
}