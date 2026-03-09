/**
 * @file drive_test_node.cpp
 * @brief Drive integration test using DRIVE_GOAL (Teensy internal PID)
 *
 * Sends goal positions via drive_base/command with DRIVE_GOAL mode.
 * The Teensy's closed-loop PID drives to each goal, no open-loop timing.
 *
 * Two modes:
 *   loop=false (default):
 *     Goals forward then back to origin. Verifies direction integration.
 *
 *   loop=true:
 *     Infinite forward/back goals with optional turns.
 *     Verifies encoder and IMU accuracy by printing drift each cycle.
 *
 * Usage:
 *   ros2 run secbot_autonomy drive_test_node
 *   ros2 run secbot_autonomy drive_test_node --ros-args -p loop:=true -p with_turn:=true
 *   ros2 run secbot_autonomy drive_test_node --ros-args -p loop:=true -p reverse:=true
 *
 * Parameters:
 *   distance       (double, 0.5)   goal distance in meters
 *   turn_angle     (double, 1.571) turn goal in radians (default 90deg)
 *   with_turn      (bool, false)   add CW/CCW turn goals
 *   loop           (bool, false)   infinite back-and-forth mode
 *   reverse        (bool, false)   drive backwards to backward goals
 *   pause_time     (double, 1.0)   pause between goals in seconds
 *   goal_tolerance (double, 0.03)  position tolerance in meters
 *   goal_timeout   (double, 10.0)  timeout per goal in seconds
 */

#include "secbot_autonomy/drive_test_node.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace secbot {

DriveTestNode::DriveTestNode() : Node("drive_test_node") {
  this->declare_parameter("distance", 0.5);
  this->declare_parameter("turn_angle", M_PI / 2.0);
  this->declare_parameter("with_turn", false);
  this->declare_parameter("loop", false);
  this->declare_parameter("pause_time", 1.0);
  this->declare_parameter("goal_tolerance", 0.03);
  this->declare_parameter("goal_timeout", 10.0);
  this->declare_parameter("calibrate_time", 3.0);
  this->declare_parameter("reverse", false);

  distance_ = this->get_parameter("distance").as_double();
  turn_angle_ = this->get_parameter("turn_angle").as_double();
  with_turn_ = this->get_parameter("with_turn").as_bool();
  loop_ = this->get_parameter("loop").as_bool();
  reverse_ = this->get_parameter("reverse").as_bool();
  pause_time_ = this->get_parameter("pause_time").as_double();
  goal_tolerance_ = this->get_parameter("goal_tolerance").as_double();
  goal_timeout_ = this->get_parameter("goal_timeout").as_double();
  calibrate_time_ = this->get_parameter("calibrate_time").as_double();

  drive_cmd_pub_ =
      this->create_publisher<mcu_msgs::msg::DriveBase>("drive_base/command", 10);

  drive_status_sub_ = this->create_subscription<mcu_msgs::msg::DriveBase>(
      "drive_base/status", rclcpp::SensorDataQoS(),
      std::bind(&DriveTestNode::onDriveStatus, this, _1));

  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/mcu_robot/imu/data", rclcpp::SensorDataQoS(),
      std::bind(&DriveTestNode::onImu, this, _1));

  tick_timer_ = this->create_wall_timer(100ms, std::bind(&DriveTestNode::tick, this));

  if (loop_) {
    RCLCPP_INFO(this->get_logger(),
                "================================================");
    RCLCPP_INFO(this->get_logger(),
                "  DRIVE TEST: Verify encoder + IMU accuracy");
    RCLCPP_INFO(this->get_logger(),
                "  Mode: DRIVE_GOAL back-and-forth%s%s",
                with_turn_ ? " + turns" : "",
                reverse_ ? " (REVERSE)" : "");
    RCLCPP_INFO(this->get_logger(),
                "  Distance: %.2f m, tolerance: %.3f m",
                distance_, goal_tolerance_);
    RCLCPP_INFO(this->get_logger(),
                "  Uses Teensy internal PID (closed-loop)");
    RCLCPP_INFO(this->get_logger(),
                "  Drift from origin should stay near zero");
    RCLCPP_INFO(this->get_logger(),
                "  Ctrl-C to stop");
    RCLCPP_INFO(this->get_logger(),
                "================================================");
  } else {
    RCLCPP_INFO(this->get_logger(),
                "================================================");
    RCLCPP_INFO(this->get_logger(),
                "  DRIVE TEST: Verify direction integration");
    RCLCPP_INFO(this->get_logger(),
                "  Goal: forward %.2f m, then back to origin", distance_);
    RCLCPP_INFO(this->get_logger(),
                "  Uses Teensy internal PID (closed-loop)");
    RCLCPP_INFO(this->get_logger(),
                "  Check: heading stays ~0, Y drift stays ~0");
    RCLCPP_INFO(this->get_logger(),
                "================================================");
  }
  RCLCPP_INFO(this->get_logger(), "Waiting for drive_base/status...");
}

void DriveTestNode::onDriveStatus(
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

void DriveTestNode::onImu(const sensor_msgs::msg::Imu::SharedPtr msg) {
  if (phase_ != Phase::GYRO_CALIBRATE) return;
  double gx = msg->angular_velocity.x;
  double gy = msg->angular_velocity.y;
  double gz = msg->angular_velocity.z;
  gyro_sum_x_ += gx;
  gyro_sum_y_ += gy;
  gyro_sum_z_ += gz;
  if (gx < gyro_min_x_) gyro_min_x_ = gx;
  if (gx > gyro_max_x_) gyro_max_x_ = gx;
  if (gy < gyro_min_y_) gyro_min_y_ = gy;
  if (gy > gyro_max_y_) gyro_max_y_ = gy;
  if (gz < gyro_min_z_) gyro_min_z_ = gz;
  if (gz > gyro_max_z_) gyro_max_z_ = gz;
  gyro_samples_++;
}

void DriveTestNode::sendGoal(double x, double y, double theta, bool reverse) {
  goal_x_ = x;
  goal_y_ = y;
  goal_theta_ = theta;

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

void DriveTestNode::stopRobot() {
  auto msg = mcu_msgs::msg::DriveBase();
  msg.drive_mode = mcu_msgs::msg::DriveBase::DRIVE_VECTOR;
  msg.goal_velocity.linear.x = 0.0;
  msg.goal_velocity.angular.z = 0.0;
  drive_cmd_pub_->publish(msg);
}

bool DriveTestNode::reachedGoal() {
  double dx = robot_x_ - goal_x_;
  double dy = robot_y_ - goal_y_;
  return std::hypot(dx, dy) < goal_tolerance_;
}

void DriveTestNode::logPose(const char* label) {
  double theta_deg = robot_theta_ * 180.0 / M_PI;
  double omega_deg = robot_omega_ * 180.0 / M_PI;
  RCLCPP_INFO(this->get_logger(),
              "[%s] pose=(%.4f, %.4f, %.1fdeg) vel=(%.3f m/s, %.1f deg/s)",
              label, robot_x_, robot_y_, theta_deg, robot_vx_, omega_deg);
}

void DriveTestNode::tick() {
  auto now = std::chrono::steady_clock::now();
  double elapsed =
      std::chrono::duration<double>(now - phase_start_).count();

  auto enterPhase = [&](Phase p) {
    phase_ = p;
    phase_start_ = now;
  };

  switch (phase_) {
    case Phase::WAIT_FOR_STATUS: {
      if (!has_status_) return;
      origin_x_ = robot_x_;
      origin_y_ = robot_y_;
      origin_theta_ = robot_theta_;
      RCLCPP_INFO(this->get_logger(), "Got status, recording origin");
      logPose("ORIGIN");

      RCLCPP_INFO(this->get_logger(),
                  "Measuring gyro offset for %.1fs, keep robot still...",
                  calibrate_time_);
      gyro_sum_x_ = gyro_sum_y_ = gyro_sum_z_ = 0.0;
      gyro_min_x_ = gyro_min_y_ = gyro_min_z_ = 1e9;
      gyro_max_x_ = gyro_max_y_ = gyro_max_z_ = -1e9;
      gyro_samples_ = 0;
      enterPhase(Phase::GYRO_CALIBRATE);
      break;
    }

    case Phase::GYRO_CALIBRATE: {
      if (elapsed < calibrate_time_) return;

      RCLCPP_INFO(this->get_logger(),
                  "========== Gyro Offset Results ==========");
      if (gyro_samples_ == 0) {
        RCLCPP_WARN(this->get_logger(),
                    "  No IMU samples received on /mcu_robot/imu/data");
      } else {
        double avg_x = gyro_sum_x_ / gyro_samples_;
        double avg_y = gyro_sum_y_ / gyro_samples_;
        double avg_z = gyro_sum_z_ / gyro_samples_;
        double range_x = gyro_max_x_ - gyro_min_x_;
        double range_y = gyro_max_y_ - gyro_min_y_;
        double range_z = gyro_max_z_ - gyro_min_z_;
        RCLCPP_INFO(this->get_logger(), "  Samples:  %d (%.0f Hz)",
                    gyro_samples_, gyro_samples_ / calibrate_time_);
        RCLCPP_INFO(this->get_logger(),
                    "  Avg X:    %+.6f rad/s (%+.4f deg/s)", avg_x,
                    avg_x * 180.0 / M_PI);
        RCLCPP_INFO(this->get_logger(),
                    "  Avg Y:    %+.6f rad/s (%+.4f deg/s)", avg_y,
                    avg_y * 180.0 / M_PI);
        RCLCPP_INFO(this->get_logger(),
                    "  Avg Z:    %+.6f rad/s (%+.4f deg/s)", avg_z,
                    avg_z * 180.0 / M_PI);
        RCLCPP_INFO(this->get_logger(),
                    "  Range X:  %.6f rad/s (noise)", range_x);
        RCLCPP_INFO(this->get_logger(),
                    "  Range Y:  %.6f rad/s (noise)", range_y);
        RCLCPP_INFO(this->get_logger(),
                    "  Range Z:  %.6f rad/s (noise)", range_z);
        double drift_deg_per_min = avg_z * 180.0 / M_PI * 60.0;
        RCLCPP_INFO(this->get_logger(),
                    "  Z drift:  %+.2f deg/min (yaw bias)", drift_deg_per_min);
      }
      RCLCPP_INFO(this->get_logger(),
                  "=========================================");

      cycle_ = 1;
      RCLCPP_INFO(this->get_logger(), "--- Cycle %d ---", cycle_);
      RCLCPP_INFO(this->get_logger(),
                  "  DRIVE_GOAL -> forward %.2f m to (%.3f, %.3f)",
                  distance_, origin_x_ + distance_, origin_y_);
      sendGoal(origin_x_ + distance_, origin_y_, origin_theta_);
      enterPhase(Phase::GOAL_FORWARD);
      break;
    }

    case Phase::GOAL_FORWARD: {
      // Keep refreshing the goal (MCU times out after 500ms)
      sendGoal(goal_x_, goal_y_, goal_theta_);
      if (reachedGoal()) {
        RCLCPP_INFO(this->get_logger(), "  Reached forward goal");
        logPose("AT_GOAL_FWD");
        stopRobot();
        enterPhase(Phase::PAUSE_AFTER_FORWARD);
      } else if (elapsed > goal_timeout_) {
        RCLCPP_WARN(this->get_logger(), "  Forward goal TIMED OUT (%.1fs)", goal_timeout_);
        logPose("TIMEOUT_FWD");
        stopRobot();
        enterPhase(Phase::PAUSE_AFTER_FORWARD);
      }
      break;
    }

    case Phase::PAUSE_AFTER_FORWARD: {
      stopRobot();
      if (elapsed >= pause_time_) {
        if (!loop_) {
          RCLCPP_INFO(this->get_logger(),
                      "  DRIVE_GOAL%s -> back to origin (%.3f, %.3f)",
                      reverse_ ? " (REVERSE)" : "",
                      origin_x_, origin_y_);
          sendGoal(origin_x_, origin_y_, origin_theta_, reverse_);
          enterPhase(Phase::GOAL_BACKWARD);
        } else {
          RCLCPP_INFO(this->get_logger(),
                      "  DRIVE_GOAL%s -> back to origin (%.3f, %.3f)",
                      reverse_ ? " (REVERSE)" : "",
                      origin_x_, origin_y_);
          sendGoal(origin_x_, origin_y_, origin_theta_, reverse_);
          enterPhase(Phase::GOAL_BACKWARD);
        }
      }
      break;
    }

    case Phase::GOAL_BACKWARD: {
      sendGoal(goal_x_, goal_y_, goal_theta_, reverse_);
      if (reachedGoal()) {
        RCLCPP_INFO(this->get_logger(), "  Reached backward goal");
        logPose("AT_GOAL_BWD");
        stopRobot();
        enterPhase(Phase::PAUSE_AFTER_BACKWARD);
      } else if (elapsed > goal_timeout_) {
        RCLCPP_WARN(this->get_logger(), "  Backward goal TIMED OUT (%.1fs)", goal_timeout_);
        logPose("TIMEOUT_BWD");
        stopRobot();
        enterPhase(Phase::PAUSE_AFTER_BACKWARD);
      }
      break;
    }

    case Phase::PAUSE_AFTER_BACKWARD: {
      stopRobot();
      if (elapsed >= pause_time_) {
        if (with_turn_) {
          double target_theta = origin_theta_ - turn_angle_;
          RCLCPP_INFO(this->get_logger(),
                      "  DRIVE_GOAL -> turn CW %.0f deg",
                      turn_angle_ * 180.0 / M_PI);
          sendGoal(robot_x_, robot_y_, target_theta);
          enterPhase(Phase::GOAL_TURN_CW);
        } else {
          enterPhase(Phase::CYCLE_SUMMARY);
        }
      }
      break;
    }

    case Phase::GOAL_TURN_CW: {
      sendGoal(goal_x_, goal_y_, goal_theta_);
      // For turns, check heading instead of position
      double dtheta = std::abs(robot_theta_ - goal_theta_);
      if (dtheta < 0.1) {
        RCLCPP_INFO(this->get_logger(), "  Reached CW turn goal");
        logPose("AT_TURN_CW");
        stopRobot();
        enterPhase(Phase::PAUSE_AFTER_TURN_CW);
      } else if (elapsed > goal_timeout_) {
        RCLCPP_WARN(this->get_logger(), "  CW turn TIMED OUT");
        logPose("TIMEOUT_CW");
        stopRobot();
        enterPhase(Phase::PAUSE_AFTER_TURN_CW);
      }
      break;
    }

    case Phase::PAUSE_AFTER_TURN_CW: {
      stopRobot();
      if (elapsed >= pause_time_) {
        RCLCPP_INFO(this->get_logger(),
                    "  DRIVE_GOAL -> turn CCW back to %.0f deg",
                    origin_theta_ * 180.0 / M_PI);
        sendGoal(robot_x_, robot_y_, origin_theta_);
        enterPhase(Phase::GOAL_TURN_CCW);
      }
      break;
    }

    case Phase::GOAL_TURN_CCW: {
      sendGoal(goal_x_, goal_y_, goal_theta_);
      double dtheta = std::abs(robot_theta_ - goal_theta_);
      if (dtheta < 0.1) {
        RCLCPP_INFO(this->get_logger(), "  Reached CCW turn goal");
        logPose("AT_TURN_CCW");
        stopRobot();
        enterPhase(Phase::PAUSE_AFTER_TURN_CCW);
      } else if (elapsed > goal_timeout_) {
        RCLCPP_WARN(this->get_logger(), "  CCW turn TIMED OUT");
        logPose("TIMEOUT_CCW");
        stopRobot();
        enterPhase(Phase::PAUSE_AFTER_TURN_CCW);
      }
      break;
    }

    case Phase::PAUSE_AFTER_TURN_CCW: {
      stopRobot();
      if (elapsed >= pause_time_) {
        enterPhase(Phase::CYCLE_SUMMARY);
      }
      break;
    }

    case Phase::CYCLE_SUMMARY: {
      double dx = robot_x_ - origin_x_;
      double dy = robot_y_ - origin_y_;
      double drift = std::hypot(dx, dy);
      double dtheta = (robot_theta_ - origin_theta_) * 180.0 / M_PI;

      RCLCPP_INFO(this->get_logger(), "========== Cycle %d Results ==========", cycle_);

      if (!loop_) {
        double heading_deg = robot_theta_ * 180.0 / M_PI;
        RCLCPP_INFO(this->get_logger(), "  Final position drift: %.4f m (should be ~0)", drift);
        RCLCPP_INFO(this->get_logger(), "  Delta X:              %+.4f m", dx);
        RCLCPP_INFO(this->get_logger(), "  Delta Y:              %+.4f m (should be ~0)", dy);
        RCLCPP_INFO(this->get_logger(), "  Heading:              %+.1f deg (should be ~0)", heading_deg);
        if (std::abs(dy) > 0.05 || std::abs(heading_deg) > 5.0) {
          RCLCPP_WARN(this->get_logger(), "  >> Direction integration looks OFF");
        } else {
          RCLCPP_INFO(this->get_logger(), "  >> Direction integration looks GOOD");
        }
        RCLCPP_INFO(this->get_logger(), "======================================");
        enterPhase(Phase::DONE);
      } else {
        RCLCPP_INFO(this->get_logger(), "  Position drift:  %.4f m (should be ~0)", drift);
        RCLCPP_INFO(this->get_logger(), "  Delta X:         %+.4f m", dx);
        RCLCPP_INFO(this->get_logger(), "  Delta Y:         %+.4f m", dy);
        RCLCPP_INFO(this->get_logger(), "  Heading drift:   %+.1f deg (should be ~0)", dtheta);
        if (drift > 0.1) {
          RCLCPP_WARN(this->get_logger(), "  >> Drift is growing, check encoder/IMU calibration");
        } else {
          RCLCPP_INFO(this->get_logger(), "  >> Drift is low, encoder/IMU looking accurate");
        }
        RCLCPP_INFO(this->get_logger(), "======================================");
        cycle_++;
        RCLCPP_INFO(this->get_logger(), "--- Cycle %d ---", cycle_);
        RCLCPP_INFO(this->get_logger(),
                    "  DRIVE_GOAL -> forward %.2f m to (%.3f, %.3f)",
                    distance_, origin_x_ + distance_, origin_y_);
        sendGoal(origin_x_ + distance_, origin_y_, origin_theta_);
        enterPhase(Phase::GOAL_FORWARD);
      }
      break;
    }

    case Phase::DONE: {
      stopRobot();
      if (elapsed > 1.0) {
        RCLCPP_INFO(this->get_logger(), "=== Forward direction test complete ===");
        tick_timer_->cancel();
      }
      break;
    }
  }
}

}  // namespace secbot

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<secbot::DriveTestNode>());
  rclcpp::shutdown();
  return 0;
}
