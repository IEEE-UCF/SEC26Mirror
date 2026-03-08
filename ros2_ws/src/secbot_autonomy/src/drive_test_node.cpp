/**
 * @file drive_test_node.cpp
 * @brief Drive integration test node
 *
 * Two modes controlled by the 'loop' parameter:
 *
 *   loop=false (default):
 *     Forward-only test. Sends a forward velocity command and checks
 *     that the robot integrates direction correctly.
 *
 *   loop=true:
 *     Infinite back-and-forth with optional turns. Verifies encoder
 *     and IMU accuracy by printing drift from origin each cycle.
 *
 * Usage:
 *   ros2 run secbot_autonomy drive_test_node
 *   ros2 run secbot_autonomy drive_test_node --ros-args -p loop:=true -p with_turn:=true
 *
 * Parameters:
 *   speed      (double, 0.2)   linear speed m/s
 *   leg_time   (double, 3.0)   seconds per forward/backward leg
 *   turn_speed (double, 1.0)   turn speed rad/s
 *   with_turn  (bool, false)   add CW/CCW turn segments
 *   loop       (bool, false)   infinite back-and-forth mode
 *   stop_time  (double, 1.0)   pause between segments
 */

#include "secbot_autonomy/drive_test_node.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace secbot {

DriveTestNode::DriveTestNode() : Node("drive_test_node") {
  this->declare_parameter("speed", 0.2);
  this->declare_parameter("leg_time", 3.0);
  this->declare_parameter("turn_speed", 1.0);
  this->declare_parameter("with_turn", false);
  this->declare_parameter("loop", false);
  this->declare_parameter("stop_time", 1.0);

  speed_ = this->get_parameter("speed").as_double();
  leg_time_ = this->get_parameter("leg_time").as_double();
  turn_speed_ = this->get_parameter("turn_speed").as_double();
  with_turn_ = this->get_parameter("with_turn").as_bool();
  loop_ = this->get_parameter("loop").as_bool();
  stop_time_ = this->get_parameter("stop_time").as_double();

  drive_cmd_pub_ =
      this->create_publisher<mcu_msgs::msg::DriveBase>("drive_base/command", 10);

  drive_status_sub_ = this->create_subscription<mcu_msgs::msg::DriveBase>(
      "drive_base/status", rclcpp::SensorDataQoS(),
      std::bind(&DriveTestNode::onDriveStatus, this, _1));

  tick_timer_ = this->create_wall_timer(100ms, std::bind(&DriveTestNode::tick, this));

  if (loop_) {
    RCLCPP_INFO(this->get_logger(),
                "================================================");
    RCLCPP_INFO(this->get_logger(),
                "  DRIVE TEST: Verify encoder + IMU accuracy");
    RCLCPP_INFO(this->get_logger(),
                "  Mode: back-and-forth%s", with_turn_ ? " + turns" : "");
    RCLCPP_INFO(this->get_logger(),
                "  Speed: %.2f m/s, leg: %.1fs, turn: %.1f rad/s",
                speed_, leg_time_, turn_speed_);
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
                "  Sending forward at %.2f m/s for %.1fs", speed_, leg_time_);
    RCLCPP_INFO(this->get_logger(),
                "  Robot should move in a straight line");
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

void DriveTestNode::sendVelocity(double vx, double omega) {
  auto msg = mcu_msgs::msg::DriveBase();
  msg.drive_mode = mcu_msgs::msg::DriveBase::DRIVE_VECTOR;
  msg.goal_velocity.linear.x = vx;
  msg.goal_velocity.angular.z = omega;
  drive_cmd_pub_->publish(msg);
}

void DriveTestNode::stopRobot() { sendVelocity(0.0, 0.0); }

void DriveTestNode::logPose(const char* label) {
  double theta_deg = robot_theta_ * 180.0 / M_PI;
  double omega_deg = robot_omega_ * 180.0 / M_PI;
  RCLCPP_INFO(this->get_logger(),
              "[%s] pose=(%.4f, %.4f, %.1fdeg) vel=(%.3f m/s, %.1f deg/s)",
              label, robot_x_, robot_y_, theta_deg, robot_vx_, omega_deg);
}

const char* DriveTestNode::phaseName(Phase p) const {
  switch (p) {
    case Phase::WAIT_FOR_STATUS:    return "WAIT";
    case Phase::FORWARD:            return "FORWARD";
    case Phase::STOP_AFTER_FORWARD: return "STOP";
    case Phase::BACKWARD:           return "BACKWARD";
    case Phase::STOP_AFTER_BACKWARD:return "STOP";
    case Phase::TURN_CW:            return "TURN_CW";
    case Phase::STOP_AFTER_TURN_CW: return "STOP";
    case Phase::TURN_CCW:           return "TURN_CCW";
    case Phase::STOP_AFTER_TURN_CCW:return "STOP";
    case Phase::CYCLE_SUMMARY:      return "SUMMARY";
    case Phase::DONE:               return "DONE";
  }
  return "?";
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

      cycle_ = 1;
      RCLCPP_INFO(this->get_logger(), "--- Cycle %d ---", cycle_);
      RCLCPP_INFO(this->get_logger(),
                  "  Driving FORWARD at %.2f m/s for %.1fs...", speed_, leg_time_);
      enterPhase(Phase::FORWARD);
      break;
    }

    case Phase::FORWARD: {
      sendVelocity(speed_, 0.0);
      if (elapsed >= leg_time_) {
        logPose("END_FORWARD");
        enterPhase(Phase::STOP_AFTER_FORWARD);
      }
      break;
    }

    case Phase::STOP_AFTER_FORWARD: {
      stopRobot();
      if (elapsed >= stop_time_) {
        if (!loop_) {
          enterPhase(Phase::CYCLE_SUMMARY);
        } else {
          RCLCPP_INFO(this->get_logger(),
                      "  Driving BACKWARD at %.2f m/s for %.1fs...",
                      speed_, leg_time_);
          enterPhase(Phase::BACKWARD);
        }
      }
      break;
    }

    case Phase::BACKWARD: {
      sendVelocity(-speed_, 0.0);
      if (elapsed >= leg_time_) {
        logPose("END_BACKWARD");
        enterPhase(Phase::STOP_AFTER_BACKWARD);
      }
      break;
    }

    case Phase::STOP_AFTER_BACKWARD: {
      stopRobot();
      if (elapsed >= stop_time_) {
        if (with_turn_) {
          double turn_time = (M_PI / 2.0) / turn_speed_;
          RCLCPP_INFO(this->get_logger(),
                      "  Turning CW 90deg at %.1f rad/s for %.2fs...",
                      turn_speed_, turn_time);
          enterPhase(Phase::TURN_CW);
        } else {
          enterPhase(Phase::CYCLE_SUMMARY);
        }
      }
      break;
    }

    case Phase::TURN_CW: {
      sendVelocity(0.0, -turn_speed_);
      double turn_time = (M_PI / 2.0) / turn_speed_;
      if (elapsed >= turn_time) {
        logPose("END_TURN_CW");
        enterPhase(Phase::STOP_AFTER_TURN_CW);
      }
      break;
    }

    case Phase::STOP_AFTER_TURN_CW: {
      stopRobot();
      if (elapsed >= stop_time_) {
        double turn_time = (M_PI / 2.0) / turn_speed_;
        RCLCPP_INFO(this->get_logger(),
                    "  Turning CCW 90deg at %.1f rad/s for %.2fs...",
                    turn_speed_, turn_time);
        enterPhase(Phase::TURN_CCW);
      }
      break;
    }

    case Phase::TURN_CCW: {
      sendVelocity(0.0, turn_speed_);
      double turn_time = (M_PI / 2.0) / turn_speed_;
      if (elapsed >= turn_time) {
        logPose("END_TURN_CCW");
        enterPhase(Phase::STOP_AFTER_TURN_CCW);
      }
      break;
    }

    case Phase::STOP_AFTER_TURN_CCW: {
      stopRobot();
      if (elapsed >= stop_time_) {
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
        // Forward-only: check direction integration
        double expected = speed_ * leg_time_;
        double heading_deg = robot_theta_ * 180.0 / M_PI;
        double move_angle = std::atan2(dy, dx) * 180.0 / M_PI;
        RCLCPP_INFO(this->get_logger(), "  Distance traveled: %.4f m", drift);
        RCLCPP_INFO(this->get_logger(), "  Expected distance: %.4f m", expected);
        RCLCPP_INFO(this->get_logger(), "  Delta X:           %+.4f m", dx);
        RCLCPP_INFO(this->get_logger(), "  Delta Y:           %+.4f m (should be ~0)", dy);
        RCLCPP_INFO(this->get_logger(), "  Heading:           %+.1f deg (should be ~0)", heading_deg);
        RCLCPP_INFO(this->get_logger(), "  Move direction:    %+.1f deg (should be ~0)", move_angle);
        if (std::abs(dy) > 0.05 || std::abs(heading_deg) > 5.0) {
          RCLCPP_WARN(this->get_logger(), "  >> Direction integration looks OFF");
        } else {
          RCLCPP_INFO(this->get_logger(), "  >> Direction integration looks GOOD");
        }
        enterPhase(Phase::DONE);
      } else {
        // Back-and-forth: check drift accumulation
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
                    "  Driving FORWARD at %.2f m/s for %.1fs...", speed_, leg_time_);
        enterPhase(Phase::FORWARD);
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
