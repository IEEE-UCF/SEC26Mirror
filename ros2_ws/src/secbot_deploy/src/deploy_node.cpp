/**
 * @file deploy_node.cpp
 * @brief ROS2 bridge between MCU deploy trigger and host deploy orchestrator.
 *
 * Subscribes to /mcu_robot/deploy/trigger from the Teensy (via micro-ROS),
 * writes a trigger file for the host-side deploy-orchestrator.py, polls the
 * status file, and publishes phase updates back to the MCU for OLED and LED
 * feedback.
 */

#include <chrono>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <functional>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class DeployNode : public rclcpp::Node {
 public:
  DeployNode() : Node("deploy_node") {
    // Directories for IPC files (host-mounted via Docker volume)
    deploy_dir_ =
        this->declare_parameter("deploy_dir", "/home/ubuntu/scripts/.deploy");

    // QoS: best-effort to match micro-ROS subscriptions
    auto qos = rclcpp::QoS(10).best_effort();

    // Subscribe to MCU deploy trigger
    trigger_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/mcu_robot/deploy/trigger", qos,
        std::bind(&DeployNode::triggerCallback, this, std::placeholders::_1));

    // Publishers for MCU feedback
    status_pub_ =
        this->create_publisher<std_msgs::msg::String>(
            "/mcu_robot/deploy/status", qos);
    oled_pub_ =
        this->create_publisher<std_msgs::msg::String>(
            "/mcu_robot/lcd/append", qos);

    // Poll orchestrator status file every 500ms
    status_timer_ = this->create_wall_timer(
        500ms, std::bind(&DeployNode::pollStatus, this));

    RCLCPP_INFO(this->get_logger(), "Deploy node started, IPC dir: %s",
                deploy_dir_.c_str());
  }

 private:
  void triggerCallback(const std_msgs::msg::String::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Deploy trigger received: %s",
                msg->data.c_str());

    // Write trigger file for host orchestrator
    std::string trigger_path = deploy_dir_ + "/trigger";
    std::ofstream ofs(trigger_path, std::ios::trunc);
    if (!ofs.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to write trigger file: %s",
                   trigger_path.c_str());
      return;
    }

    if (msg->data.find("start:") == 0) {
      std::string remainder = msg->data.substr(6);  // e.g. "robot" or "robot:force"
      bool force = false;
      std::string target = remainder;
      auto force_pos = remainder.rfind(":force");
      if (force_pos != std::string::npos &&
          force_pos + 6 == remainder.size()) {
        target = remainder.substr(0, force_pos);
        force = true;
      }
      ofs << "target=" << target << "\n";
      ofs << "branch=prod\n";
      ofs << "online=auto\n";
      if (force) {
        ofs << "force=true\n";
      }
      auto now = std::chrono::system_clock::now();
      auto epoch = std::chrono::duration_cast<std::chrono::seconds>(
                       now.time_since_epoch())
                       .count();
      ofs << "timestamp=" << epoch << "\n";
      RCLCPP_INFO(this->get_logger(), "Trigger: target=%s, force=%s",
                  target.c_str(), force ? "true" : "false");
    } else if (msg->data == "cancel") {
      ofs << "cancel\n";
      // OLED feedback is handled by MCU DeploySubsystem directly
    }

    ofs.close();
    last_phase_.clear();
    last_message_.clear();
  }

  void pollStatus() {
    std::string status_path = deploy_dir_ + "/status";
    std::ifstream ifs(status_path);
    if (!ifs.is_open()) return;

    std::string phase, message;
    std::string line;
    while (std::getline(ifs, line)) {
      if (line.find("phase=") == 0) {
        phase = line.substr(6);
      } else if (line.find("message=") == 0) {
        message = line.substr(8);
      }
    }
    ifs.close();

    if (phase.empty()) return;

    bool phase_changed = (phase != last_phase_);
    bool message_changed = (message != last_message_);

    if (!phase_changed && !message_changed) return;

    if (phase_changed) {
      last_phase_ = phase;
      // Publish phase to MCU for LED control
      auto status_msg = std_msgs::msg::String();
      status_msg.data = "phase:" + phase;
      status_pub_->publish(status_msg);
    }

    if (message_changed) {
      last_message_ = message;
      // Publish message to MCU for per-device LED updates
      if (!message.empty()) {
        auto msg_update = std_msgs::msg::String();
        msg_update.data = "msg:" + message;
        status_pub_->publish(msg_update);
      }
      // Publish message to OLED
      if (!message.empty()) {
        auto oled_msg = std_msgs::msg::String();
        oled_msg.data = message;
        oled_pub_->publish(oled_msg);
      }
    }

    RCLCPP_INFO(this->get_logger(), "Deploy status: phase=%s msg=%s",
                phase.c_str(), message.c_str());
  }

  std::string deploy_dir_;
  std::string last_phase_;
  std::string last_message_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr trigger_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr oled_pub_;
  rclcpp::TimerBase::SharedPtr status_timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DeployNode>());
  rclcpp::shutdown();
  return 0;
}
