#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <thread>
#include <termios.h>
#include <std_msgs/msg/char.hpp>
// #include <std_msgs/msg/string.hpp>
// extern "C"{
// #include <bits/stdc++.h>
// }
class keystroke_publisher : public rclcpp::Node{
    public:
    keystroke_publisher(): Node("key_stroke_publisher"), char_userinput('0'){
        keystroke_publisher_object = this->create_publisher<std_msgs::msg::Char>("keystroke_topic",10);   
    //since this is a simple keystroke publisher we DO NOT need to periodically call an executor callback function
    //whatever user types in gets immedately placed in the topic
    }
    
    void input_function();
    private:
    // rclcpp::TimerBase::SharedPtr timer;
    char char_userinput;
    rclcpp::Publisher<std_msgs::msg::Char>::SharedPtr keystroke_publisher_object;//not necessary for now..unless this node wants to recieve 
    //msgs from another node...
};