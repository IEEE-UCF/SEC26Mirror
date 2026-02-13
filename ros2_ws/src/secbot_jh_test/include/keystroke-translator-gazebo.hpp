#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <iostream>
// #include <thread>
// #include <termios.h>
#include <std_msgs/msg/char.hpp>
// #include <geometry_msgs/msg/twist.hpp> //mayebe not yet??
#include <geometry_msgs/msg/pose.hpp>//doing ros2 interface show geometry_msgs/msg/Pose
#include <geometry_msgs/msg/twist.hpp>

//gives the standard message....although for now we focus on x,y, z
/*
Point position
        float64 x
        float64 y
        float64 z
*/
class keystroke_translator: public rclcpp::Node{
    public:
    keystroke_translator(): Node("keystroke_translator"), current_x_velocity(0.0), current_y_velocity(0.0),current_z_velocity(0.0){

        // position_publisher = this->create_publisher<geometry_msgs::msg::Pose>("drone_position",10);
        velocity_publisher = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel",10);
        auto keystroke_callback = [this](std_msgs::msg::Char::UniquePtr received_msg)->void{//callback function for subscriber
            RCLCPP_INFO(this->get_logger(), "Key pressed: %c",received_msg->data);
            // auto position_message = geometry_msgs::msg::Pose();
            auto velocity_message = geometry_msgs::msg::Twist();
            switch(received_msg->data){
                case('w'):
                current_x_velocity = .20;
                RCLCPP_INFO(this->get_logger(),"Position x now: %f",current_x_velocity);
                // position_message.position.x = current_x;
                velocity_message.linear.x = current_x_velocity;
                velocity_message.linear.y = current_y_velocity;
                velocity_message.linear.z = current_z_velocity;
                // position_message.position.y = current_y;
                // position_message.position.z = current_z;
                this->velocity_publisher->publish(velocity_message);
                break;
                case('s'):
                current_x_velocity = -.20;
                RCLCPP_INFO(this->get_logger(),"Position x now: %f",current_x_velocity);
                velocity_message.linear.x = current_x_velocity;
                velocity_message.linear.y = current_y_velocity;
                velocity_message.linear.z = current_z_velocity;
                // position_message.position.x = current_x;
                // position_message.position.y = current_y;
                // position_message.position.z = current_z;
                this->velocity_publisher->publish(velocity_message);
                break;
                case('a'):
                current_y_velocity = -.20;
                RCLCPP_INFO(this->get_logger(),"Position y now: %f", current_y_velocity);
                velocity_message.linear.x = current_x_velocity;
                velocity_message.linear.y = current_y_velocity;
                velocity_message.linear.z = current_z_velocity;
                // position_message.position.y = current_y;
                // position_message.position.x = current_x;
                // position_message.position.z = current_z;
                this->velocity_publisher->publish(velocity_message);
                break;
                case('d'):
                current_y_velocity = .20;
                RCLCPP_INFO(this->get_logger(),"Position y now: %f", current_y_velocity);
                velocity_message.linear.x = current_x_velocity;
                velocity_message.linear.y = current_y_velocity;
                velocity_message.linear.z = current_z_velocity;
                // position_message.position.y = current_y;
                // position_message.position.x = current_x;
                // position_message.position.z = current_z;
                this->velocity_publisher->publish(velocity_message);
                break;
                case('i'):
                current_z_velocity = .20;
                RCLCPP_INFO(this->get_logger(),"Position z now: %f", current_z_velocity);
                velocity_message.linear.x = current_x_velocity;
                velocity_message.linear.y = current_y_velocity;
                velocity_message.linear.z = current_z_velocity;
                // position_message.position.z = current_z;
                // position_message.position.y = current_y;
                // position_message.position.x = current_x;
                this->velocity_publisher->publish(velocity_message);
                break;
                case('k'):
                current_z_velocity = -.20;
                RCLCPP_INFO(this->get_logger(),"Position y now: %f", current_z_velocity);
                velocity_message.linear.x = current_x_velocity;
                velocity_message.linear.y = current_y_velocity;
                velocity_message.linear.z = current_z_velocity;
                // position_message.position.z = current_z_;
                // position_message.position.y = current_y;
                // position_message.position.x = current_x;
                this->velocity_publisher->publish(velocity_message);
                break;
                case('b'):
                current_z_velocity = current_z_velocity-.09;
                current_x_velocity = 0.0;
                current_y_velocity = 0.0;
                RCLCPP_INFO(this->get_logger(),"not moving!");
                velocity_message.linear.x = current_x_velocity;
                velocity_message.linear.y = current_y_velocity;
                velocity_message.linear.z = current_z_velocity;

                // RCLCPP_INFO(this->get_logger,"PLEASE INPUT CORRECT keys!!");
                break;
            }
        };
        keystroke_subscriber = this->create_subscription<std_msgs::msg::Char>("keystroke_topic",10,keystroke_callback);
    }
    private:
    // auto position_message = geometry_msgs::msg::Pose();

    float current_x_velocity;
    float current_y_velocity;
    float current_z_velocity;
    rclcpp::Subscription<std_msgs::msg::Char>::SharedPtr keystroke_subscriber;
    // rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr position_publisher;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher;//seems like the only way we can move in this sim
};
/*
if you want to double check if your bridge is a ok run the simulation FIRST(yes this means launching the sdf world and the drone)
then run these commands
ubuntu@5cf5fa2ae5f6:~/ros2_workspaces$ gz topic -l

/clock
/gazebo/resource_paths
/gui/track
/model/first_drone/cmd_vel
/stats
/world/car_world/clock
/world/car_world/dynamic_pose/info
/world/car_world/pose/info
/world/car_world/scene/deletion
/world/car_world/scene/info
/world/car_world/state
/world/car_world/stats
/gui/currently_tracked
/world/car_world/light_config
/world/car_world/material_color

ubuntu@5cf5fa2ae5f6:~/ros2_workspaces$ gz topic -i -t /model/first_drone/cmd_vel
Publishers [Address, Message Type]:
  tcp://172.17.0.2:44525, gz.msgs.Twist
Subscribers [Address, Message Type]:
  tcp://172.17.0.2:42433, gz.msgs.Twist

as for me i am just checking to see if my bridge obtained the correct topic name and that gazebo has a publisher(ROS2) and a 
subscriber(gazebo itself)
  */