/**
 * @file drone_teleop_node.cpp
 * @brief RC-to-drone teleop bridge.
 *
 * Subscribes to /mcu_robot/rc (FlySky via robot Teensy) and translates
 * stick/switch inputs into drone commands:
 *
 *   SWA  → arm / disarm  (toggle on rising edge)
 *   SWB  → takeoff / land (toggle on rising edge, only while armed)
 *   SWC  → tare IMU yaw  (rising edge)
 *   Right stick X → roll   (cmd_vel linear.y)
 *   Right stick Y → pitch  (cmd_vel linear.x)
 *   Left stick Y  → altitude rate (cmd_vel linear.z)
 *   Left stick X  → yaw rate (cmd_vel angular.z)
 *
 * RC channels are normalized to [-255, 255].  A deadzone is applied and
 * values are mapped to [-1, 1] for cmd_vel.
 *
 * Usage:
 *   ros2 run secbot_drone_teleop drone_teleop_node
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <mcu_msgs/msg/rc.hpp>
#include <mcu_msgs/srv/drone_arm.hpp>
#include <mcu_msgs/srv/drone_takeoff.hpp>
#include <mcu_msgs/srv/drone_land.hpp>
#include <mcu_msgs/srv/drone_tare.hpp>

class DroneTeleopNode : public rclcpp::Node {
 public:
  DroneTeleopNode() : Node("drone_teleop") {
    // Parameters
    declare_parameter("deadzone", 20);
    declare_parameter("takeoff_altitude", 0.5);
    declare_parameter("cmd_vel_rate_hz", 20.0);

    deadzone_ = get_parameter("deadzone").as_int();
    takeoff_alt_ = get_parameter("takeoff_altitude").as_double();

    // RC subscriber — micro-ROS publishes with BEST_EFFORT reliability
    auto qos = rclcpp::QoS(10).best_effort();
    rc_sub_ = create_subscription<mcu_msgs::msg::RC>(
        "/mcu_robot/rc", qos,
        std::bind(&DroneTeleopNode::rcCallback, this, std::placeholders::_1));

    // Drone cmd_vel publisher
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(
        "/mcu_drone/cmd_vel", 10);

    // Service clients
    arm_client_ = create_client<mcu_msgs::srv::DroneArm>("/mcu_drone/arm");
    takeoff_client_ =
        create_client<mcu_msgs::srv::DroneTakeoff>("/mcu_drone/takeoff");
    land_client_ = create_client<mcu_msgs::srv::DroneLand>("/mcu_drone/land");
    tare_client_ = create_client<mcu_msgs::srv::DroneTare>("/mcu_drone/tare");

    // Publish cmd_vel at fixed rate
    double rate = get_parameter("cmd_vel_rate_hz").as_double();
    cmd_vel_timer_ = create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / rate)),
        std::bind(&DroneTeleopNode::publishCmdVel, this));

    RCLCPP_INFO(get_logger(),
                "Drone teleop ready. SWA=arm, SWB=takeoff/land, SWC=tare, sticks=fly");
  }

 private:
  void rcCallback(const mcu_msgs::msg::RC::SharedPtr msg) {
    // Store stick values for cmd_vel publishing
    // RC range [-255, 255] → normalize to [-1, 1] with deadzone
    rx_ = applyDeadzone(msg->rx);
    ry_ = applyDeadzone(msg->ry);
    lx_ = applyDeadzone(msg->lx);
    ly_ = applyDeadzone(msg->ly);

    // SWA rising edge → toggle arm/disarm
    bool swa_now = msg->swa;
    if (swa_now && !prev_swa_) {
      callArm(!armed_);
    }
    prev_swa_ = swa_now;

    // SWB rising edge → toggle takeoff/land (only when armed)
    bool swb_now = msg->swb;
    if (swb_now && !prev_swb_ && armed_) {
      if (!flying_) {
        callTakeoff(takeoff_alt_);
      } else {
        callLand();
      }
    }
    prev_swb_ = swb_now;

    // SWC rising edge → tare IMU yaw
    bool swc_now = msg->swc;
    if (swc_now && !prev_swc_) {
      callTare();
    }
    prev_swc_ = swc_now;

    last_rc_time_ = now();
  }

  void publishCmdVel() {
    // Don't publish if no recent RC data (>500ms stale)
    if ((now() - last_rc_time_).seconds() > 0.5) {
      return;
    }

    // Only publish cmd_vel while flying
    if (!flying_) return;

    auto msg = geometry_msgs::msg::Twist();
    msg.linear.x = ry_;    // right stick Y → pitch (forward/back)
    msg.linear.y = rx_;    // right stick X → roll (left/right)
    msg.linear.z = ly_;    // left stick Y  → altitude rate
    msg.angular.z = lx_;   // left stick X  → yaw rate
    cmd_vel_pub_->publish(msg);
  }

  float applyDeadzone(int32_t raw) {
    if (std::abs(raw) < deadzone_) return 0.0f;
    // Map [deadzone, 255] → [0, 1], preserve sign
    float sign = (raw > 0) ? 1.0f : -1.0f;
    float magnitude =
        static_cast<float>(std::abs(raw) - deadzone_) / (255.0f - deadzone_);
    return sign * std::min(magnitude, 1.0f);
  }

  void callArm(bool arm) {
    if (!arm_client_->service_is_ready()) {
      RCLCPP_WARN(get_logger(), "Arm service not available");
      return;
    }
    auto req = std::make_shared<mcu_msgs::srv::DroneArm::Request>();
    req->arm = arm;
    auto future = arm_client_->async_send_request(
        req, [this, arm](rclcpp::Client<mcu_msgs::srv::DroneArm>::SharedFuture
                             result) {
          auto resp = result.get();
          if (resp->success) {
            armed_ = arm;
            if (!arm) flying_ = false;
            RCLCPP_INFO(get_logger(), "%s: %s",
                        arm ? "ARM" : "DISARM", resp->message.c_str());
          } else {
            RCLCPP_WARN(get_logger(), "%s failed: %s",
                        arm ? "ARM" : "DISARM", resp->message.c_str());
          }
        });
  }

  void callTakeoff(double altitude) {
    if (!takeoff_client_->service_is_ready()) {
      RCLCPP_WARN(get_logger(), "Takeoff service not available");
      return;
    }
    auto req = std::make_shared<mcu_msgs::srv::DroneTakeoff::Request>();
    req->target_altitude = altitude;
    auto future = takeoff_client_->async_send_request(
        req,
        [this](rclcpp::Client<mcu_msgs::srv::DroneTakeoff>::SharedFuture
                   result) {
          auto resp = result.get();
          if (resp->success) {
            flying_ = true;
            RCLCPP_INFO(get_logger(), "TAKEOFF: %s", resp->message.c_str());
          } else {
            RCLCPP_WARN(get_logger(), "TAKEOFF failed: %s",
                        resp->message.c_str());
          }
        });
  }

  void callLand() {
    if (!land_client_->service_is_ready()) {
      RCLCPP_WARN(get_logger(), "Land service not available");
      return;
    }
    auto req = std::make_shared<mcu_msgs::srv::DroneLand::Request>();
    auto future = land_client_->async_send_request(
        req, [this](rclcpp::Client<mcu_msgs::srv::DroneLand>::SharedFuture
                         result) {
          auto resp = result.get();
          if (resp->success) {
            flying_ = false;
            RCLCPP_INFO(get_logger(), "LAND: %s", resp->message.c_str());
          } else {
            RCLCPP_WARN(get_logger(), "LAND failed: %s",
                        resp->message.c_str());
          }
        });
  }

  void callTare() {
    if (!tare_client_->service_is_ready()) {
      RCLCPP_WARN(get_logger(), "Tare service not available");
      return;
    }
    auto req = std::make_shared<mcu_msgs::srv::DroneTare::Request>();
    auto future = tare_client_->async_send_request(
        req, [this](rclcpp::Client<mcu_msgs::srv::DroneTare>::SharedFuture
                         result) {
          auto resp = result.get();
          if (resp->success) {
            RCLCPP_INFO(get_logger(), "TARE: %s", resp->message.c_str());
          } else {
            RCLCPP_WARN(get_logger(), "TARE failed: %s",
                        resp->message.c_str());
          }
        });
  }

  // Subscriptions / publishers
  rclcpp::Subscription<mcu_msgs::msg::RC>::SharedPtr rc_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::TimerBase::SharedPtr cmd_vel_timer_;

  // Service clients
  rclcpp::Client<mcu_msgs::srv::DroneArm>::SharedPtr arm_client_;
  rclcpp::Client<mcu_msgs::srv::DroneTakeoff>::SharedPtr takeoff_client_;
  rclcpp::Client<mcu_msgs::srv::DroneLand>::SharedPtr land_client_;
  rclcpp::Client<mcu_msgs::srv::DroneTare>::SharedPtr tare_client_;

  // RC stick state (normalized [-1, 1])
  float rx_ = 0.0f, ry_ = 0.0f, lx_ = 0.0f, ly_ = 0.0f;

  // Switch state tracking (rising-edge detection)
  bool prev_swa_ = false;
  bool prev_swb_ = false;
  bool prev_swc_ = false;

  // Drone state
  bool armed_ = false;
  bool flying_ = false;

  // Config
  int deadzone_ = 20;
  double takeoff_alt_ = 0.5;

  rclcpp::Time last_rc_time_{0, 0, RCL_ROS_TIME};
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DroneTeleopNode>());
  rclcpp::shutdown();
  return 0;
}
