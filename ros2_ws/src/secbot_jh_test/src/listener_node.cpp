#include "listener_node.hpp"
int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<firstSubscriber>());
  rclcpp::shutdown();
}
/*
The main function is roughly the same, except now it spins the firstSubscriber
node. For the publisher node, spinning meant starting the timer, but for the
subscriber it simply means preparing to receive messages whenever they come.

*/