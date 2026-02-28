/**
 * @file health_node.cpp
 * @author Rafeed Khan
 * @brief System health monitor and MCU heartbeat watchdog
 *
 * Subscribes to all critical MCU and ROS2 topics, detects timeouts and faults,
 * publishes aggregated diagnostics on /diagnostics (standard DiagnosticArray)
 * and a stupidly simple /system/ok bool
 *
 * MCU firmware publishes:
 *   /mcu_robot/heartbeat (std_msgs/String) @ 5 Hz best-effort
 *   /mcu_robot/mcu_state (mcu_msgs/McuState) @ around 50 Hz best-effort
 * /mcu_robot/battery_health (mcu_msgs/BatteryHealth) @ 1 Hz best-effort
 *
 * ROS2 nodes publish:
 *   /odom (nav_msgs/Odometry) @ 10 Hz
 *   /autonomy/task_status (secbot_msgs/TaskStatus) @ varies
 *
 * In simulation mode (use_sim=true), MCU monitoring is disabled.
 */

#include <chrono>
#include <cstdio>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <mcu_msgs/msg/battery_health.hpp>
#include <mcu_msgs/msg/mcu_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <secbot_msgs/msg/task_status.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <vector>

using namespace std::chrono_literals;

namespace secbot_health {

// MCU state constants (MUSSSSTTTTT match mcu_msgs/McuState.msg)
namespace McuStateCode {
constexpr uint8_t PRE_INIT = 0;
constexpr uint8_t INIT = 1;
constexpr uint8_t INIT_FAIL = 2;
constexpr uint8_t INIT_SUCCESS = 3;
constexpr uint8_t ARMED = 4;
constexpr uint8_t RUNNING = 5;
constexpr uint8_t STOPPED = 6;
constexpr uint8_t RESET = 7;
}  // namespace McuStateCode

// DiagnosticStatus level aliases
namespace Level {
constexpr uint8_t OK = diagnostic_msgs::msg::DiagnosticStatus::OK;
constexpr uint8_t WARN = diagnostic_msgs::msg::DiagnosticStatus::WARN;
constexpr uint8_t ERROR = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
constexpr uint8_t STALE = diagnostic_msgs::msg::DiagnosticStatus::STALE;
}  // namespace Level

/// Format a float to a string with fixed precision
static std::string fmtFloat(double val, int precision = 2) {
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%.*f", precision, val);
  return buf;
}

/// Return human-readable MCU state name
static const char* mcuStateName(uint8_t state) {
  switch (state) {
    case McuStateCode::PRE_INIT:
      return "PRE_INIT";
    case McuStateCode::INIT:
      return "INIT";
    case McuStateCode::INIT_FAIL:
      return "INIT_FAIL";
    case McuStateCode::INIT_SUCCESS:
      return "INIT_SUCCESS";
    case McuStateCode::ARMED:
      return "ARMED";
    case McuStateCode::RUNNING:
      return "RUNNING";
    case McuStateCode::STOPPED:
      return "STOPPED";
    case McuStateCode::RESET:
      return "RESET";
    default:
      return "UNKNOWN";
  }
}

// HealthNode

class HealthNode : public rclcpp::Node {
 public:
  HealthNode() : Node("health_node") {
    //  Parameters
    if (!this->has_parameter("use_sim")) {
      this->declare_parameter("use_sim", false);
    }
    this->declare_parameter("heartbeat_timeout_sec", 1.0);
    this->declare_parameter("mcu_state_timeout_sec", 2.0);
    this->declare_parameter("battery_timeout_sec", 5.0);
    this->declare_parameter("battery_warn_voltage", 11.0);
    this->declare_parameter("battery_error_voltage", 10.5);
    this->declare_parameter("odom_timeout_sec", 3.0);
    this->declare_parameter("autonomy_timeout_sec", 3.0);
    this->declare_parameter("publish_rate_hz", 2.0);

    use_sim_ = this->get_parameter("use_sim").as_bool();
    heartbeat_timeout_ =
        this->get_parameter("heartbeat_timeout_sec").as_double();
    mcu_state_timeout_ =
        this->get_parameter("mcu_state_timeout_sec").as_double();
    battery_timeout_ = this->get_parameter("battery_timeout_sec").as_double();
    battery_warn_v_ = this->get_parameter("battery_warn_voltage").as_double();
    battery_error_v_ = this->get_parameter("battery_error_voltage").as_double();
    odom_timeout_ = this->get_parameter("odom_timeout_sec").as_double();
    autonomy_timeout_ = this->get_parameter("autonomy_timeout_sec").as_double();
    double rate_hz = this->get_parameter("publish_rate_hz").as_double();

    auto now = this->now();

    // MCU subscribers
    // MCU firmware publishes with best-effort QoS, so we must match!!
    auto best_effort = rclcpp::QoS(10).best_effort();

    if (!use_sim_) {
      heartbeat_time_ = now;
      mcu_state_time_ = now;
      battery_time_ = now;

      heartbeat_sub_ = this->create_subscription<std_msgs::msg::String>(
          "/mcu_robot/heartbeat", best_effort,
          [this](const std_msgs::msg::String::SharedPtr /*msg*/) {
            heartbeat_time_ = this->now();
            heartbeat_received_ = true;
          });

      mcu_state_sub_ = this->create_subscription<mcu_msgs::msg::McuState>(
          "/mcu_robot/mcu_state", best_effort,
          [this](const mcu_msgs::msg::McuState::SharedPtr msg) {
            mcu_state_time_ = this->now();
            mcu_state_received_ = true;
            mcu_state_ = msg->state;
          });

      battery_sub_ = this->create_subscription<mcu_msgs::msg::BatteryHealth>(
          "/mcu_robot/battery_health", best_effort,
          [this](const mcu_msgs::msg::BatteryHealth::SharedPtr msg) {
            battery_time_ = this->now();
            battery_received_ = true;
            battery_voltage_ = msg->voltage;
            battery_current_ = msg->current;
            battery_temp_ = msg->temperature;
          });
    }

    //  ROS2 node liveness monitors (both real and sim)
    odom_time_ = now;
    autonomy_time_ = now;

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, [this](const nav_msgs::msg::Odometry::SharedPtr /*msg*/) {
          odom_time_ = this->now();
          odom_received_ = true;
        });

    autonomy_sub_ = this->create_subscription<secbot_msgs::msg::TaskStatus>(
        "/autonomy/task_status", 10,
        [this](const secbot_msgs::msg::TaskStatus::SharedPtr /*msg*/) {
          autonomy_time_ = this->now();
          autonomy_received_ = true;
        });

    //  Publishers
    diag_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
        "/diagnostics", 10);
    ok_pub_ = this->create_publisher<std_msgs::msg::Bool>("/system/ok", 10);

    //  Health check timer
    auto period_ms = static_cast<int>(1000.0 / rate_hz);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(period_ms),
                                     std::bind(&HealthNode::checkHealth, this));

    RCLCPP_INFO(this->get_logger(),
                "Health node initialized (sim=%s, rate=%.0fHz)",
                use_sim_ ? "true" : "false", rate_hz);
  }

 private:
  //  Main health check (runs at publish_rate_hz)
  void checkHealth() {
    auto now = this->now();

    diagnostic_msgs::msg::DiagnosticArray diag_array;
    diag_array.header.stamp = now;

    bool all_ok = true;

    //  MCU monitors (real hardware only)
    if (!use_sim_) {
      auto hb =
          checkTopic("MCU Heartbeat", "/mcu_robot/heartbeat", heartbeat_time_,
                     heartbeat_timeout_, heartbeat_received_, now);
      diag_array.status.push_back(hb);
      if (hb.level != Level::OK) all_ok = false;

      auto mcu = checkMcuState(now);
      diag_array.status.push_back(mcu);
      if (mcu.level != Level::OK) all_ok = false;

      auto batt = checkBattery(now);
      diag_array.status.push_back(batt);
      if (batt.level != Level::OK) all_ok = false;
    }

    //  ROS2 node monitors
    auto odom = checkTopic("Navigation (/odom)", "/odom", odom_time_,
                           odom_timeout_, odom_received_, now);
    diag_array.status.push_back(odom);
    if (odom.level == Level::ERROR) all_ok = false;

    auto auton = checkTopic("Autonomy", "/autonomy/task_status", autonomy_time_,
                            autonomy_timeout_, autonomy_received_, now);
    diag_array.status.push_back(auton);
    if (auton.level == Level::ERROR) all_ok = false;

    //  Publish
    diag_pub_->publish(diag_array);

    auto ok_msg = std_msgs::msg::Bool();
    ok_msg.data = all_ok;
    ok_pub_->publish(ok_msg);
  }

  //  Generic topic liveness check
  diagnostic_msgs::msg::DiagnosticStatus checkTopic(
      const std::string& name, const std::string& topic,
      const rclcpp::Time& last_time, double timeout, bool ever_received,
      const rclcpp::Time& now) {
    diagnostic_msgs::msg::DiagnosticStatus s;
    s.name = name;
    s.hardware_id = "secbot";

    addKV(s, "topic", topic);

    if (!ever_received) {
      s.level = Level::STALE;
      s.message = "Waiting for first message";
      return s;
    }

    double age = (now - last_time).seconds();
    addKV(s, "age_sec", fmtFloat(age, 1));

    if (age > timeout) {
      s.level = Level::ERROR;
      s.message =
          "TIMEOUT (" + fmtFloat(age, 1) + "s > " + fmtFloat(timeout, 1) + "s)";
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "%s: TIMEOUT (%.1fs since last msg)", name.c_str(),
                           age);
    } else {
      s.level = Level::OK;
      s.message = "Active";
    }

    return s;
  }

  //  MCU lifecycle state check
  diagnostic_msgs::msg::DiagnosticStatus checkMcuState(
      const rclcpp::Time& now) {
    diagnostic_msgs::msg::DiagnosticStatus s;
    s.name = "MCU Lifecycle";
    s.hardware_id = "teensy41";

    if (!mcu_state_received_) {
      s.level = Level::STALE;
      s.message = "Waiting for first MCU state";
      return s;
    }

    double age = (now - mcu_state_time_).seconds();
    addKV(s, "age_sec", fmtFloat(age, 1));
    addKV(s, "state", mcuStateName(mcu_state_));

    if (age > mcu_state_timeout_) {
      s.level = Level::ERROR;
      s.message = "MCU state timeout";
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "MCU state TIMEOUT (%.1fs)", age);
      return s;
    }

    switch (mcu_state_) {
      case McuStateCode::RUNNING:
        s.level = Level::OK;
        s.message = "Running";
        break;
      case McuStateCode::ARMED:
        s.level = Level::OK;
        s.message = "Armed (awaiting begin)";
        break;
      case McuStateCode::INIT:
      case McuStateCode::INIT_SUCCESS:
        s.level = Level::WARN;
        s.message = std::string("Initializing: ") + mcuStateName(mcu_state_);
        break;
      case McuStateCode::INIT_FAIL:
        s.level = Level::ERROR;
        s.message = "INIT FAILED";
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                              "MCU INIT FAILED!");
        break;
      case McuStateCode::STOPPED:
        s.level = Level::WARN;
        s.message = "Stopped";
        break;
      case McuStateCode::RESET:
        s.level = Level::WARN;
        s.message = "Resetting";
        break;
      default:
        s.level = Level::WARN;
        s.message = std::string("Unknown state: ") + std::to_string(mcu_state_);
        break;
    }

    return s;
  }

  //  Battery voltage check
  diagnostic_msgs::msg::DiagnosticStatus checkBattery(const rclcpp::Time& now) {
    diagnostic_msgs::msg::DiagnosticStatus s;
    s.name = "Battery";
    s.hardware_id = "power_board";

    if (!battery_received_) {
      s.level = Level::STALE;
      s.message = "Waiting for battery data";
      return s;
    }

    double age = (now - battery_time_).seconds();
    addKV(s, "age_sec", fmtFloat(age, 1));

    if (age > battery_timeout_) {
      s.level = Level::ERROR;
      s.message = "Battery data timeout";
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "Battery data TIMEOUT (%.1fs)", age);
      return s;
    }

    addKV(s, "voltage_v", fmtFloat(battery_voltage_));
    addKV(s, "current_a", fmtFloat(battery_current_));
    addKV(s, "temperature_c", fmtFloat(battery_temp_));

    if (battery_voltage_ < battery_error_v_) {
      s.level = Level::ERROR;
      s.message = "CRITICAL: " + fmtFloat(battery_voltage_) + "V";
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                            "BATTERY CRITICAL: %.2fV (limit %.2fV)",
                            battery_voltage_, battery_error_v_);
    } else if (battery_voltage_ < battery_warn_v_) {
      s.level = Level::WARN;
      s.message = "Low: " + fmtFloat(battery_voltage_) + "V";
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
                           "Battery low: %.2fV (warn %.2fV)", battery_voltage_,
                           battery_warn_v_);
    } else {
      s.level = Level::OK;
      s.message = fmtFloat(battery_voltage_) + "V";
    }

    return s;
  }

  //  Small helper to add key-value pair to DiagnosticStatus
  static void addKV(diagnostic_msgs::msg::DiagnosticStatus& s,
                    const std::string& key, const std::string& value) {
    diagnostic_msgs::msg::KeyValue kv;
    kv.key = key;
    kv.value = value;
    s.values.push_back(kv);
  }

  //  Parameters
  bool use_sim_;
  double heartbeat_timeout_;
  double mcu_state_timeout_;
  double battery_timeout_;
  double battery_warn_v_;
  double battery_error_v_;
  double odom_timeout_;
  double autonomy_timeout_;

  //  MCU monitoring state
  rclcpp::Time heartbeat_time_{0, 0, RCL_ROS_TIME};
  bool heartbeat_received_ = false;

  rclcpp::Time mcu_state_time_{0, 0, RCL_ROS_TIME};
  bool mcu_state_received_ = false;
  uint8_t mcu_state_ = 0;

  rclcpp::Time battery_time_{0, 0, RCL_ROS_TIME};
  bool battery_received_ = false;
  float battery_voltage_ = 0.0f;
  float battery_current_ = 0.0f;
  float battery_temp_ = 0.0f;

  //  ROS2 node monitoring state
  rclcpp::Time odom_time_{0, 0, RCL_ROS_TIME};
  bool odom_received_ = false;

  rclcpp::Time autonomy_time_{0, 0, RCL_ROS_TIME};
  bool autonomy_received_ = false;

  //  Subscribers
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr heartbeat_sub_;
  rclcpp::Subscription<mcu_msgs::msg::McuState>::SharedPtr mcu_state_sub_;
  rclcpp::Subscription<mcu_msgs::msg::BatteryHealth>::SharedPtr battery_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<secbot_msgs::msg::TaskStatus>::SharedPtr autonomy_sub_;

  //  Publishers
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ok_pub_;

  //  Timer
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace secbot_health

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<secbot_health::HealthNode>());
  rclcpp::shutdown();
  return 0;
}
