/**
 * @file test_velocity.cpp
 * @brief DRIVE_VECTOR mode tests — open-loop velocity commands
 *
 * Tests:
 *   1. Forward velocity — drive forward at 0.3 m/s for 2s, verify displacement
 *   2. Backward velocity — drive backward at -0.3 m/s for 2s
 *   3. Turn in place CW — angular velocity only (positive omega)
 *   4. Turn in place CCW — angular velocity only (negative omega)
 *   5. Arc turn — forward + angular combined
 *   6. Stop command — verify robot stops within tolerance
 *   7. Heading hold — drive straight, verify no yaw drift
 *
 * NOTE: The MCU clamps velocity to MAX_LINEAR_VEL (0.7 m/s) and applies
 *   jerk-limited ramping (a_max=0.7 m/s², j_max=3.05 m/s³), so it takes
 *   ~1s to reach full speed. Values above 0.7 are clamped on the Teensy.
 *   Use drive_time >= 3s for the robot to actually reach and sustain speed.
 *
 * Usage:
 *   ros2 run secbot_drive_tests test_velocity
 *   ros2 run secbot_drive_tests test_velocity --ros-args -p velocity:=0.5
 */

#include "secbot_drive_tests/test_harness.hpp"

using namespace std::chrono_literals;

namespace secbot_drive_tests {

class VelocityTest : public DriveTestHarness {
 public:
  VelocityTest() : DriveTestHarness("test_velocity") {
    // NOTE: MCU clamps linear vel to 0.7 m/s and angular to 6.0 rad/s.
    // Jerk-limited ramp takes ~1s to reach 0.5 m/s. Use drive_time >= 3s.
    this->declare_parameter("velocity", 0.5);
    this->declare_parameter("drive_time", 3.0);
    this->declare_parameter("angular_velocity", 1.5);

    velocity_ = this->get_parameter("velocity").as_double();
    drive_time_ = this->get_parameter("drive_time").as_double();
    angular_vel_ = this->get_parameter("angular_velocity").as_double();

    tick_timer_ = this->create_wall_timer(50ms, std::bind(&VelocityTest::tick, this));

    RCLCPP_INFO(this->get_logger(), "===================================");
    RCLCPP_INFO(this->get_logger(), "  VELOCITY MODE TEST SUITE");
    RCLCPP_INFO(this->get_logger(), "  vel=%.2f m/s  omega=%.2f rad/s",
                velocity_, angular_vel_);
    RCLCPP_INFO(this->get_logger(), "  drive_time=%.1fs", drive_time_);
    RCLCPP_INFO(this->get_logger(), "===================================");
  }

 private:
  enum class Phase {
    WAIT_STATUS,
    // Test 1: Forward
    FWD_DRIVE, FWD_STOP, FWD_SETTLE, FWD_RECORD,
    // Test 2: Backward
    BWD_DRIVE, BWD_STOP, BWD_SETTLE, BWD_RECORD,
    // Test 3: Turn CW
    CW_DRIVE, CW_STOP, CW_SETTLE, CW_RECORD,
    // Test 4: Turn CCW
    CCW_DRIVE, CCW_STOP, CCW_SETTLE, CCW_RECORD,
    // Test 5: Arc turn
    ARC_DRIVE, ARC_STOP, ARC_SETTLE, ARC_RECORD,
    // Test 6: Stop command
    STOP_ACCEL, STOP_CMD, STOP_SETTLE, STOP_RECORD,
    // Test 7: Heading hold
    HEADING_DRIVE, HEADING_STOP, HEADING_SETTLE, HEADING_RECORD,
    // Done
    SUMMARY, DONE
  };

  Phase phase_ = Phase::WAIT_STATUS;
  double velocity_;
  double drive_time_;
  double angular_vel_;

  // Snapshot state at test transitions
  double snap_x_, snap_y_, snap_theta_;

  void enterPhase(Phase p) {
    phase_ = p;
    resetPhaseTimer();
  }

  void snapshot() {
    snap_x_ = robot_x_;
    snap_y_ = robot_y_;
    snap_theta_ = robot_theta_;
  }

  void tick() {
    switch (phase_) {
      // ── Wait for drive_base/status ──
      case Phase::WAIT_STATUS:
        if (!has_status_) return;
        captureOrigin();
        RCLCPP_INFO(this->get_logger(), "Status received, starting tests...");
        logPose("ORIGIN");
        snapshot();
        RCLCPP_INFO(this->get_logger(), "\n--- Test 1: Forward velocity ---");
        enterPhase(Phase::FWD_DRIVE);
        break;

      // ═══════ Test 1: Forward ═══════
      case Phase::FWD_DRIVE:
        sendVelocity(velocity_, 0.0);
        if (elapsed() >= drive_time_) {
          stopRobot();
          enterPhase(Phase::FWD_STOP);
        }
        break;
      case Phase::FWD_STOP:
        stopRobot();
        if (elapsed() >= 0.5) enterPhase(Phase::FWD_SETTLE);
        break;
      case Phase::FWD_SETTLE:
        stopRobot();
        if (elapsed() >= 0.3) enterPhase(Phase::FWD_RECORD);
        break;
      case Phase::FWD_RECORD: {
        double dx = robot_x_ - snap_x_;
        double expected = velocity_ * drive_time_;
        double err = std::abs(dx - expected);
        double lateral_drift = std::abs(robot_y_ - snap_y_);
        bool passed = err < expected * 0.5 && lateral_drift < 0.1;
        recordResult("Forward velocity", passed, err,
                     headingError(snap_theta_), drive_time_,
                     "dx=" + std::to_string(dx) + " expected=" + std::to_string(expected));
        logPose("FWD_END");
        snapshot();
        RCLCPP_INFO(this->get_logger(), "\n--- Test 2: Backward velocity ---");
        enterPhase(Phase::BWD_DRIVE);
        break;
      }

      // ═══════ Test 2: Backward ═══════
      case Phase::BWD_DRIVE:
        sendVelocity(-velocity_, 0.0);
        if (elapsed() >= drive_time_) {
          stopRobot();
          enterPhase(Phase::BWD_STOP);
        }
        break;
      case Phase::BWD_STOP:
        stopRobot();
        if (elapsed() >= 0.5) enterPhase(Phase::BWD_SETTLE);
        break;
      case Phase::BWD_SETTLE:
        stopRobot();
        if (elapsed() >= 0.3) enterPhase(Phase::BWD_RECORD);
        break;
      case Phase::BWD_RECORD: {
        double dx = robot_x_ - snap_x_;
        double expected = -velocity_ * drive_time_;
        double err = std::abs(dx - expected);
        double lateral_drift = std::abs(robot_y_ - snap_y_);
        bool passed = err < std::abs(expected) * 0.5 && lateral_drift < 0.1;
        recordResult("Backward velocity", passed, err,
                     headingError(snap_theta_), drive_time_,
                     "dx=" + std::to_string(dx));
        logPose("BWD_END");
        snapshot();
        RCLCPP_INFO(this->get_logger(), "\n--- Test 3: Turn CW ---");
        enterPhase(Phase::CW_DRIVE);
        break;
      }

      // ═══════ Test 3: Turn CW ═══════
      case Phase::CW_DRIVE:
        sendVelocity(0.0, -angular_vel_);
        if (elapsed() >= drive_time_) {
          stopRobot();
          enterPhase(Phase::CW_STOP);
        }
        break;
      case Phase::CW_STOP:
        stopRobot();
        if (elapsed() >= 0.5) enterPhase(Phase::CW_SETTLE);
        break;
      case Phase::CW_SETTLE:
        stopRobot();
        if (elapsed() >= 0.3) enterPhase(Phase::CW_RECORD);
        break;
      case Phase::CW_RECORD: {
        double dtheta = normalizeAngle(robot_theta_ - snap_theta_);
        double expected = -angular_vel_ * drive_time_;
        double expected_norm = normalizeAngle(expected);
        double err = std::abs(normalizeAngle(dtheta - expected_norm));
        double pos_drift = std::hypot(robot_x_ - snap_x_, robot_y_ - snap_y_);
        // Turn in place should have minimal positional drift
        bool passed = err < 0.5 && pos_drift < 0.1;
        recordResult("Turn CW in place", passed, pos_drift, err, drive_time_,
                     "dtheta=" + std::to_string(dtheta * 180.0 / M_PI) + "deg");
        logPose("CW_END");
        snapshot();
        RCLCPP_INFO(this->get_logger(), "\n--- Test 4: Turn CCW ---");
        enterPhase(Phase::CCW_DRIVE);
        break;
      }

      // ═══════ Test 4: Turn CCW ═══════
      case Phase::CCW_DRIVE:
        sendVelocity(0.0, angular_vel_);
        if (elapsed() >= drive_time_) {
          stopRobot();
          enterPhase(Phase::CCW_STOP);
        }
        break;
      case Phase::CCW_STOP:
        stopRobot();
        if (elapsed() >= 0.5) enterPhase(Phase::CCW_SETTLE);
        break;
      case Phase::CCW_SETTLE:
        stopRobot();
        if (elapsed() >= 0.3) enterPhase(Phase::CCW_RECORD);
        break;
      case Phase::CCW_RECORD: {
        double dtheta = normalizeAngle(robot_theta_ - snap_theta_);
        double expected = angular_vel_ * drive_time_;
        double expected_norm = normalizeAngle(expected);
        double err = std::abs(normalizeAngle(dtheta - expected_norm));
        double pos_drift = std::hypot(robot_x_ - snap_x_, robot_y_ - snap_y_);
        bool passed = err < 0.5 && pos_drift < 0.1;
        recordResult("Turn CCW in place", passed, pos_drift, err, drive_time_,
                     "dtheta=" + std::to_string(dtheta * 180.0 / M_PI) + "deg");
        logPose("CCW_END");
        snapshot();
        RCLCPP_INFO(this->get_logger(), "\n--- Test 5: Arc turn ---");
        enterPhase(Phase::ARC_DRIVE);
        break;
      }

      // ═══════ Test 5: Arc turn (forward + angular) ═══════
      case Phase::ARC_DRIVE:
        sendVelocity(velocity_, angular_vel_ * 0.5);
        if (elapsed() >= drive_time_) {
          stopRobot();
          enterPhase(Phase::ARC_STOP);
        }
        break;
      case Phase::ARC_STOP:
        stopRobot();
        if (elapsed() >= 0.5) enterPhase(Phase::ARC_SETTLE);
        break;
      case Phase::ARC_SETTLE:
        stopRobot();
        if (elapsed() >= 0.3) enterPhase(Phase::ARC_RECORD);
        break;
      case Phase::ARC_RECORD: {
        double displacement = std::hypot(robot_x_ - snap_x_, robot_y_ - snap_y_);
        double dtheta = std::abs(normalizeAngle(robot_theta_ - snap_theta_));
        // Arc should produce both displacement and heading change
        bool has_displacement = displacement > 0.05;
        bool has_rotation = dtheta > 0.1;
        bool passed = has_displacement && has_rotation;
        recordResult("Arc turn", passed, displacement, dtheta, drive_time_,
                     "disp=" + std::to_string(displacement) +
                     " rot=" + std::to_string(dtheta * 180.0 / M_PI) + "deg");
        logPose("ARC_END");
        snapshot();
        RCLCPP_INFO(this->get_logger(), "\n--- Test 6: Stop command ---");
        enterPhase(Phase::STOP_ACCEL);
        break;
      }

      // ═══════ Test 6: Stop command ═══════
      case Phase::STOP_ACCEL:
        sendVelocity(velocity_, 0.0);
        if (elapsed() >= 1.0) {
          snapshot();
          stopRobot();
          enterPhase(Phase::STOP_CMD);
        }
        break;
      case Phase::STOP_CMD:
        stopRobot();
        if (elapsed() >= 0.5) enterPhase(Phase::STOP_SETTLE);
        break;
      case Phase::STOP_SETTLE:
        stopRobot();
        if (elapsed() >= 0.5) enterPhase(Phase::STOP_RECORD);
        break;
      case Phase::STOP_RECORD: {
        // After sending stop, robot should have near-zero velocity
        bool vel_stopped = std::abs(robot_vx_) < 0.05;
        bool omega_stopped = std::abs(robot_omega_) < 0.1;
        bool passed = vel_stopped && omega_stopped;
        recordResult("Stop command", passed, std::abs(robot_vx_),
                     std::abs(robot_omega_), 1.0,
                     "vx=" + std::to_string(robot_vx_) +
                     " omega=" + std::to_string(robot_omega_));
        logPose("STOP_END");
        snapshot();
        RCLCPP_INFO(this->get_logger(), "\n--- Test 7: Heading hold ---");
        enterPhase(Phase::HEADING_DRIVE);
        break;
      }

      // ═══════ Test 7: Heading hold ═══════
      case Phase::HEADING_DRIVE:
        sendVelocity(velocity_ * 0.5, 0.0);
        if (elapsed() >= drive_time_ * 1.5) {
          stopRobot();
          enterPhase(Phase::HEADING_STOP);
        }
        break;
      case Phase::HEADING_STOP:
        stopRobot();
        if (elapsed() >= 0.5) enterPhase(Phase::HEADING_SETTLE);
        break;
      case Phase::HEADING_SETTLE:
        stopRobot();
        if (elapsed() >= 0.3) enterPhase(Phase::HEADING_RECORD);
        break;
      case Phase::HEADING_RECORD: {
        double heading_drift = std::abs(normalizeAngle(robot_theta_ - snap_theta_));
        double lateral_drift = std::abs(robot_y_ - snap_y_);
        bool passed = heading_drift < 0.15 && lateral_drift < 0.05;
        recordResult("Heading hold", passed, lateral_drift, heading_drift,
                     drive_time_ * 1.5,
                     "lat_drift=" + std::to_string(lateral_drift) +
                     " heading_drift=" + std::to_string(heading_drift * 180.0 / M_PI) + "deg");
        logPose("HEADING_END");
        enterPhase(Phase::SUMMARY);
        break;
      }

      // ── Summary ──
      case Phase::SUMMARY:
        stopRobot();
        printSummary();
        enterPhase(Phase::DONE);
        break;

      case Phase::DONE:
        stopRobot();
        if (elapsed() > 1.0) tick_timer_->cancel();
        break;
    }
  }
};

}  // namespace secbot_drive_tests

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<secbot_drive_tests::VelocityTest>());
  rclcpp::shutdown();
  return 0;
}
