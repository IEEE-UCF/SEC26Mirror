#include "secbot_drive_tests/test_harness.hpp"

#include <geometry_msgs/msg/pose.hpp>

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace secbot_drive_tests {

DriveTestHarness::DriveTestHarness(const std::string& node_name)
    : Node(node_name) {
  // ── Common test parameters ──
  this->declare_parameter("goal_tolerance", 0.03);
  this->declare_parameter("heading_tolerance", 0.1);
  this->declare_parameter("goal_timeout", 10.0);
  this->declare_parameter("pause_time", 1.0);

  goal_tolerance_ = this->get_parameter("goal_tolerance").as_double();
  heading_tolerance_ = this->get_parameter("heading_tolerance").as_double();
  goal_timeout_ = this->get_parameter("goal_timeout").as_double();
  pause_time_ = this->get_parameter("pause_time").as_double();

  // ── Drive config parameters (0 = don't change on MCU) ──
  this->declare_parameter("wheel_kp", 0.0);
  this->declare_parameter("wheel_ki", 0.0);
  this->declare_parameter("wheel_kd", 0.0);
  this->declare_parameter("wheel_i_min", 0.0);
  this->declare_parameter("wheel_i_max", 0.0);
  this->declare_parameter("pose_k_linear", 0.0);
  this->declare_parameter("pose_k_angular", 0.0);
  this->declare_parameter("max_linear_vel", 0.0);
  this->declare_parameter("max_angular_vel", 0.0);
  this->declare_parameter("max_linear_accel", 0.0);
  this->declare_parameter("max_angular_accel", 0.0);

  cfg_wheel_kp_ = this->get_parameter("wheel_kp").as_double();
  cfg_wheel_ki_ = this->get_parameter("wheel_ki").as_double();
  cfg_wheel_kd_ = this->get_parameter("wheel_kd").as_double();
  cfg_wheel_i_min_ = this->get_parameter("wheel_i_min").as_double();
  cfg_wheel_i_max_ = this->get_parameter("wheel_i_max").as_double();
  cfg_pose_k_linear_ = this->get_parameter("pose_k_linear").as_double();
  cfg_pose_k_angular_ = this->get_parameter("pose_k_angular").as_double();
  cfg_max_linear_vel_ = this->get_parameter("max_linear_vel").as_double();
  cfg_max_angular_vel_ = this->get_parameter("max_angular_vel").as_double();
  cfg_max_linear_accel_ = this->get_parameter("max_linear_accel").as_double();
  cfg_max_angular_accel_ = this->get_parameter("max_angular_accel").as_double();

  // ── Publishers / Subscribers ──
  drive_cmd_pub_ =
      this->create_publisher<mcu_msgs::msg::DriveBase>("drive_base/command", 10);

  drive_status_sub_ = this->create_subscription<mcu_msgs::msg::DriveBase>(
      "drive_base/status", rclcpp::SensorDataQoS(),
      std::bind(&DriveTestHarness::onDriveStatus, this, _1));

  // ── Pose reset publisher ──
  pose_reset_pub_ =
      this->create_publisher<geometry_msgs::msg::Pose>("drive_base/reset_pose", 10);

  // ── Service clients ──
  imu_tare_client_ =
      this->create_client<mcu_msgs::srv::Reset>("/mcu_robot/imu/tare");
  drive_config_client_ =
      this->create_client<mcu_msgs::srv::SetDriveConfig>("/mcu_robot/drive/config");
}

void DriveTestHarness::onDriveStatus(
    const mcu_msgs::msg::DriveBase::SharedPtr msg) {
  auto& t = msg->transform.transform.translation;
  auto& r = msg->transform.transform.rotation;
  robot_x_ = t.x;
  robot_y_ = t.y;
  robot_theta_ = 2.0 * std::atan2(r.z, r.w);
  robot_vx_ = msg->twist.linear.x;
  robot_omega_ = msg->twist.angular.z;
  has_status_ = true;
}

// ── Pre-test setup ──

void DriveTestHarness::startSetup() {
  setup_state_ = SetupState::TARE_WAIT;
  tare_sent_ = false;
  config_sent_ = false;
  RCLCPP_INFO(this->get_logger(), "[SETUP] Starting pre-test initialization...");
}

bool DriveTestHarness::setupComplete() {
  switch (setup_state_) {
    case SetupState::IDLE:
      return false;

    case SetupState::TARE_WAIT: {
      // Step 1: Call IMU tare
      if (!tare_sent_) {
        if (!imu_tare_client_->wait_for_service(0s)) {
          RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                               "[SETUP] Waiting for /mcu_robot/imu/tare service...");
          return false;
        }
        auto req = std::make_shared<mcu_msgs::srv::Reset::Request>();
        tare_future_ = imu_tare_client_->async_send_request(req);
        tare_sent_ = true;
        RCLCPP_INFO(this->get_logger(), "[SETUP] IMU tare request sent");
      }

      if (tare_future_.wait_for(0s) == std::future_status::ready) {
        auto result = tare_future_.get();
        if (result->success) {
          RCLCPP_INFO(this->get_logger(), "[SETUP] IMU tare SUCCESS");
        } else {
          RCLCPP_WARN(this->get_logger(), "[SETUP] IMU tare returned false");
        }
        setup_state_ = SetupState::CONFIG_WAIT;
      }
      return false;
    }

    case SetupState::CONFIG_WAIT: {
      // Step 2: Send drive config (if any non-zero params)
      bool has_config = cfg_wheel_kp_ != 0.0 || cfg_wheel_ki_ != 0.0 ||
                        cfg_wheel_kd_ != 0.0 || cfg_wheel_i_min_ != 0.0 ||
                        cfg_wheel_i_max_ != 0.0 || cfg_pose_k_linear_ != 0.0 ||
                        cfg_pose_k_angular_ != 0.0 || cfg_max_linear_vel_ != 0.0 ||
                        cfg_max_angular_vel_ != 0.0 || cfg_max_linear_accel_ != 0.0 ||
                        cfg_max_angular_accel_ != 0.0;

      if (!has_config) {
        RCLCPP_INFO(this->get_logger(), "[SETUP] No drive config overrides, skipping");
        setup_state_ = SetupState::POSE_RESET;
        return false;
      }

      if (!config_sent_) {
        if (!drive_config_client_->wait_for_service(0s)) {
          RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                               "[SETUP] Waiting for /mcu_robot/drive/config service...");
          return false;
        }
        auto req = std::make_shared<mcu_msgs::srv::SetDriveConfig::Request>();
        req->wheel_kp = static_cast<float>(cfg_wheel_kp_);
        req->wheel_ki = static_cast<float>(cfg_wheel_ki_);
        req->wheel_kd = static_cast<float>(cfg_wheel_kd_);
        req->wheel_i_min = static_cast<float>(cfg_wheel_i_min_);
        req->wheel_i_max = static_cast<float>(cfg_wheel_i_max_);
        req->pose_k_linear = static_cast<float>(cfg_pose_k_linear_);
        req->pose_k_angular = static_cast<float>(cfg_pose_k_angular_);
        req->max_linear_vel = static_cast<float>(cfg_max_linear_vel_);
        req->max_angular_vel = static_cast<float>(cfg_max_angular_vel_);
        req->max_linear_accel = static_cast<float>(cfg_max_linear_accel_);
        req->max_angular_accel = static_cast<float>(cfg_max_angular_accel_);
        config_future_ = drive_config_client_->async_send_request(req);
        config_sent_ = true;
        RCLCPP_INFO(this->get_logger(), "[SETUP] Drive config request sent");
      }

      if (config_future_.wait_for(0s) == std::future_status::ready) {
        auto result = config_future_.get();
        if (result->success) {
          RCLCPP_INFO(this->get_logger(), "[SETUP] Drive config applied: %s",
                      result->message.c_str());
        } else {
          RCLCPP_WARN(this->get_logger(), "[SETUP] Drive config FAILED: %s",
                      result->message.c_str());
        }
        setup_state_ = SetupState::POSE_RESET;
      }
      return false;
    }

    case SetupState::POSE_RESET: {
      // Step 3: Reset pose to origin (0,0,0)
      auto msg = geometry_msgs::msg::Pose();
      msg.position.x = 0.0;
      msg.position.y = 0.0;
      msg.position.z = 0.0;
      msg.orientation.x = 0.0;
      msg.orientation.y = 0.0;
      msg.orientation.z = 0.0;
      msg.orientation.w = 1.0;
      pose_reset_pub_->publish(msg);
      RCLCPP_INFO(this->get_logger(), "[SETUP] Pose reset to (0, 0, 0)");

      // Log all active config for reference
      RCLCPP_INFO(this->get_logger(), "[SETUP] Config params:");
      if (cfg_wheel_kp_ != 0.0)
        RCLCPP_INFO(this->get_logger(), "  wheel_kp=%.4f", cfg_wheel_kp_);
      if (cfg_wheel_ki_ != 0.0)
        RCLCPP_INFO(this->get_logger(), "  wheel_ki=%.4f", cfg_wheel_ki_);
      if (cfg_wheel_kd_ != 0.0)
        RCLCPP_INFO(this->get_logger(), "  wheel_kd=%.4f", cfg_wheel_kd_);
      if (cfg_wheel_i_min_ != 0.0)
        RCLCPP_INFO(this->get_logger(), "  wheel_i_min=%.4f", cfg_wheel_i_min_);
      if (cfg_wheel_i_max_ != 0.0)
        RCLCPP_INFO(this->get_logger(), "  wheel_i_max=%.4f", cfg_wheel_i_max_);
      if (cfg_pose_k_linear_ != 0.0)
        RCLCPP_INFO(this->get_logger(), "  pose_k_linear=%.4f", cfg_pose_k_linear_);
      if (cfg_pose_k_angular_ != 0.0)
        RCLCPP_INFO(this->get_logger(), "  pose_k_angular=%.4f", cfg_pose_k_angular_);
      if (cfg_max_linear_vel_ != 0.0)
        RCLCPP_INFO(this->get_logger(), "  max_linear_vel=%.4f", cfg_max_linear_vel_);
      if (cfg_max_angular_vel_ != 0.0)
        RCLCPP_INFO(this->get_logger(), "  max_angular_vel=%.4f", cfg_max_angular_vel_);
      if (cfg_max_linear_accel_ != 0.0)
        RCLCPP_INFO(this->get_logger(), "  max_linear_accel=%.4f", cfg_max_linear_accel_);
      if (cfg_max_angular_accel_ != 0.0)
        RCLCPP_INFO(this->get_logger(), "  max_angular_accel=%.4f", cfg_max_angular_accel_);

      bool all_default = cfg_wheel_kp_ == 0.0 && cfg_wheel_ki_ == 0.0 &&
                         cfg_wheel_kd_ == 0.0 && cfg_wheel_i_min_ == 0.0 &&
                         cfg_wheel_i_max_ == 0.0 && cfg_pose_k_linear_ == 0.0 &&
                         cfg_pose_k_angular_ == 0.0 && cfg_max_linear_vel_ == 0.0 &&
                         cfg_max_angular_vel_ == 0.0 && cfg_max_linear_accel_ == 0.0 &&
                         cfg_max_angular_accel_ == 0.0;
      if (all_default) {
        RCLCPP_INFO(this->get_logger(), "  (all defaults — no overrides)");
      }

      setup_state_ = SetupState::DONE;
      RCLCPP_INFO(this->get_logger(), "[SETUP] Pre-test initialization complete");
      return false;  // return false one more time so next tick captures fresh pose
    }

    case SetupState::DONE:
      return true;
  }
  return false;
}

// ── Command helpers ──

void DriveTestHarness::sendVelocity(double vx, double omega) {
  auto msg = mcu_msgs::msg::DriveBase();
  msg.drive_mode = mcu_msgs::msg::DriveBase::DRIVE_VECTOR;
  msg.goal_velocity.linear.x = vx;
  msg.goal_velocity.angular.z = omega;
  drive_cmd_pub_->publish(msg);
}

void DriveTestHarness::sendGoal(double x, double y, double theta, bool reverse) {
  auto msg = mcu_msgs::msg::DriveBase();
  msg.drive_mode = mcu_msgs::msg::DriveBase::DRIVE_GOAL;
  msg.goal_transform.transform.translation.x = x;
  msg.goal_transform.transform.translation.y = y;
  msg.goal_transform.transform.translation.z = reverse ? -1.0 : 0.0;

  float half = static_cast<float>(theta) * 0.5f;
  msg.goal_transform.transform.rotation.x = 0.0;
  msg.goal_transform.transform.rotation.y = 0.0;
  msg.goal_transform.transform.rotation.z = std::sin(half);
  msg.goal_transform.transform.rotation.w = std::cos(half);

  drive_cmd_pub_->publish(msg);
}

void DriveTestHarness::sendTrajectory(
    const std::vector<std::array<double, 2>>& waypoints, double heading) {
  auto msg = mcu_msgs::msg::DriveBase();
  msg.drive_mode = mcu_msgs::msg::DriveBase::DRIVE_TRAJ;

  nav_msgs::msg::Path path;
  path.header.stamp = this->get_clock()->now();
  path.header.frame_id = "odom";

  for (const auto& wp : waypoints) {
    geometry_msgs::msg::PoseStamped ps;
    ps.header = path.header;
    ps.pose.position.x = wp[0];
    ps.pose.position.y = wp[1];
    ps.pose.position.z = 0.0;
    ps.pose.orientation.w = 1.0;
    path.poses.push_back(std::move(ps));
  }

  // Encode final heading in last pose's quaternion if provided
  if (!std::isnan(heading) && !path.poses.empty()) {
    float half = static_cast<float>(heading) * 0.5f;
    auto& last = path.poses.back();
    last.pose.orientation.z = std::sin(half);
    last.pose.orientation.w = std::cos(half);
  }

  msg.goal_path = path;

  // Also set goal_transform to last waypoint for the MCU
  if (!waypoints.empty()) {
    auto& last_wp = waypoints.back();
    msg.goal_transform.transform.translation.x = last_wp[0];
    msg.goal_transform.transform.translation.y = last_wp[1];
  }

  drive_cmd_pub_->publish(msg);
}

void DriveTestHarness::stopRobot() {
  sendVelocity(0.0, 0.0);
}

// ── Pose helpers ──

double DriveTestHarness::distanceTo(double x, double y) const {
  return std::hypot(robot_x_ - x, robot_y_ - y);
}

double DriveTestHarness::headingError(double target_theta) const {
  return std::abs(normalizeAngle(robot_theta_ - target_theta));
}

double DriveTestHarness::driftFromOrigin() const {
  return std::hypot(robot_x_ - origin_x_, robot_y_ - origin_y_);
}

void DriveTestHarness::captureOrigin() {
  origin_x_ = robot_x_;
  origin_y_ = robot_y_;
  origin_theta_ = robot_theta_;
}

void DriveTestHarness::logPose(const char* label) {
  double theta_deg = robot_theta_ * 180.0 / M_PI;
  double omega_deg = robot_omega_ * 180.0 / M_PI;
  RCLCPP_INFO(this->get_logger(),
              "[%s] pose=(%.4f, %.4f, %.1fdeg) vel=(%.3f m/s, %.1f deg/s)",
              label, robot_x_, robot_y_, theta_deg, robot_vx_, omega_deg);
}

double DriveTestHarness::normalizeAngle(double a) {
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

// ── Result reporting ──

void DriveTestHarness::recordResult(const std::string& name, bool passed,
                                     double pos_err, double heading_err,
                                     double duration, const std::string& note) {
  results_.push_back({name, passed, pos_err, heading_err, duration, note});

  const char* status = passed ? "PASS" : "FAIL";
  RCLCPP_INFO(this->get_logger(),
              "  [%s] %s — pos_err=%.4fm heading_err=%.1fdeg dt=%.1fs %s",
              status, name.c_str(), pos_err, heading_err * 180.0 / M_PI,
              duration, note.c_str());
}

void DriveTestHarness::printSummary() {
  int pass = 0, fail = 0;
  for (const auto& r : results_) {
    if (r.passed) pass++;
    else fail++;
  }

  RCLCPP_INFO(this->get_logger(), "");
  RCLCPP_INFO(this->get_logger(),
              "╔══════════════════════════════════════════════╗");
  RCLCPP_INFO(this->get_logger(),
              "║          DRIVE TEST SUMMARY                 ║");
  RCLCPP_INFO(this->get_logger(),
              "╠══════════════════════════════════════════════╣");

  for (const auto& r : results_) {
    const char* status = r.passed ? "PASS" : "FAIL";
    RCLCPP_INFO(this->get_logger(),
                "║ [%s] %-38s ║", status, r.name.c_str());
  }

  RCLCPP_INFO(this->get_logger(),
              "╠══════════════════════════════════════════════╣");
  RCLCPP_INFO(this->get_logger(),
              "║ Total: %d passed, %d failed / %zu tests       ║",
              pass, fail, results_.size());
  RCLCPP_INFO(this->get_logger(),
              "╚══════════════════════════════════════════════╝");

  if (fail > 0) {
    RCLCPP_WARN(this->get_logger(), "%d test(s) FAILED", fail);
  } else {
    RCLCPP_INFO(this->get_logger(), "All tests PASSED");
  }
}

// ── Timer helpers ──

double DriveTestHarness::elapsed() const {
  auto now = std::chrono::steady_clock::now();
  return std::chrono::duration<double>(now - phase_start_).count();
}

void DriveTestHarness::resetPhaseTimer() {
  phase_start_ = std::chrono::steady_clock::now();
}

}  // namespace secbot_drive_tests
