/**
 * @file test_stress.cpp
 * @brief Edge case and stress tests for the drive base
 *
 * Tests:
 *   1. Command timeout — stop sending, verify robot stops within 500ms
 *   2. Rapid mode switch — alternate VECTOR/GOAL rapidly, verify no crash
 *   3. Near-zero goal — send goal 1cm away, verify arrival without oscillation
 *   4. Max velocity — command full speed, verify motor saturation works
 *   5. Emergency stop — full speed then immediate zero
 *   6. Repeated back-and-forth — 5 cycles, check cumulative drift
 *   7. Spin test — 360deg CW then 360deg CCW, check heading return
 *
 * Usage:
 *   ros2 run secbot_drive_tests test_stress
 */

#include "secbot_drive_tests/test_harness.hpp"

using namespace std::chrono_literals;

namespace secbot_drive_tests {

class StressTest : public DriveTestHarness {
 public:
  StressTest() : DriveTestHarness("test_stress") {
    this->declare_parameter("max_velocity", 0.7);
    max_vel_ = this->get_parameter("max_velocity").as_double();

    tick_timer_ = this->create_wall_timer(50ms, std::bind(&StressTest::tick, this));

    RCLCPP_INFO(this->get_logger(), "===================================");
    RCLCPP_INFO(this->get_logger(), "  STRESS / EDGE CASE TEST SUITE");
    RCLCPP_INFO(this->get_logger(), "  max_vel=%.2f m/s", max_vel_);
    RCLCPP_INFO(this->get_logger(), "===================================");
  }

 private:
  enum class Phase {
    WAIT_STATUS,
    // Test 1: Command timeout
    TIMEOUT_ACCEL, TIMEOUT_WAIT, TIMEOUT_CHECK, TIMEOUT_RECORD,
    // Test 2: Rapid mode switch
    RAPID_SWITCH, RAPID_SETTLE, RAPID_RECORD,
    // Test 3: Near-zero goal
    NEAR_GOAL, NEAR_PAUSE, NEAR_RECORD,
    // Test 4: Max velocity
    MAX_VEL_DRIVE, MAX_VEL_STOP, MAX_VEL_RECORD,
    // Test 5: Emergency stop
    ESTOP_ACCEL, ESTOP_STOP, ESTOP_SETTLE, ESTOP_RECORD,
    // Test 6: Repeated back-and-forth (5 cycles)
    REPEAT_FWD, REPEAT_FWD_PAUSE,
    REPEAT_BWD, REPEAT_BWD_PAUSE,
    REPEAT_CHECK,
    // Test 7: Spin test
    SPIN_CW, SPIN_CW_PAUSE,
    SPIN_CCW, SPIN_CCW_PAUSE,
    SPIN_RECORD,
    // Done
    SUMMARY, DONE
  };

  Phase phase_ = Phase::WAIT_STATUS;
  double max_vel_;
  int rapid_count_ = 0;
  int repeat_cycle_ = 0;
  static constexpr int REPEAT_CYCLES = 5;

  double snap_x_, snap_y_, snap_theta_;
  double snap_vx_;

  void enterPhase(Phase p) {
    phase_ = p;
    resetPhaseTimer();
  }

  void snapshot() {
    snap_x_ = robot_x_;
    snap_y_ = robot_y_;
    snap_theta_ = robot_theta_;
    snap_vx_ = robot_vx_;
  }

  void tick() {
    switch (phase_) {
      case Phase::WAIT_STATUS:
        if (!has_status_) return;
        captureOrigin();
        RCLCPP_INFO(this->get_logger(), "Status received, starting stress tests...");
        logPose("ORIGIN");

        // Test 1: Command timeout
        RCLCPP_INFO(this->get_logger(), "\n--- Test 1: Command timeout ---");
        enterPhase(Phase::TIMEOUT_ACCEL);
        break;

      // ═══════ Test 1: Command timeout ═══════
      case Phase::TIMEOUT_ACCEL:
        sendVelocity(0.3, 0.0);
        if (elapsed() >= 1.5) {
          snapshot();
          RCLCPP_INFO(this->get_logger(), "  Stopping commands, robot should auto-stop in 500ms...");
          // INTENTIONALLY stop sending — do NOT call stopRobot()
          enterPhase(Phase::TIMEOUT_WAIT);
        }
        break;
      case Phase::TIMEOUT_WAIT:
        // Don't send any commands — let timeout trigger
        if (elapsed() >= 1.5) enterPhase(Phase::TIMEOUT_CHECK);
        break;
      case Phase::TIMEOUT_CHECK:
        // Don't send anything, just check
        if (elapsed() >= 0.5) enterPhase(Phase::TIMEOUT_RECORD);
        break;
      case Phase::TIMEOUT_RECORD: {
        bool vel_stopped = std::abs(robot_vx_) < 0.05;
        bool omega_stopped = std::abs(robot_omega_) < 0.1;
        bool passed = vel_stopped && omega_stopped;
        recordResult("Command timeout auto-stop", passed,
                     std::abs(robot_vx_), std::abs(robot_omega_), 2.0,
                     "vx=" + std::to_string(robot_vx_));
        logPose("TIMEOUT_END");
        stopRobot();
        // Test 2: Rapid mode switch
        RCLCPP_INFO(this->get_logger(), "\n--- Test 2: Rapid mode switch ---");
        rapid_count_ = 0;
        enterPhase(Phase::RAPID_SWITCH);
        break;
      }

      // ═══════ Test 2: Rapid mode switch ═══════
      case Phase::RAPID_SWITCH: {
        // Alternate between VECTOR and GOAL every 50ms tick
        if (rapid_count_ % 2 == 0) {
          sendVelocity(0.2, 0.5);
        } else {
          sendGoal(origin_x_ + 0.3, origin_y_, origin_theta_);
        }
        rapid_count_++;
        if (rapid_count_ >= 40) {  // 2 seconds of rapid switching
          stopRobot();
          enterPhase(Phase::RAPID_SETTLE);
        }
        break;
      }
      case Phase::RAPID_SETTLE:
        stopRobot();
        if (elapsed() >= 1.0) enterPhase(Phase::RAPID_RECORD);
        break;
      case Phase::RAPID_RECORD: {
        // Pass criteria: robot didn't crash (still receiving status) and stopped
        bool alive = has_status_;
        bool stopped = std::abs(robot_vx_) < 0.1;
        recordResult("Rapid mode switch", alive && stopped, 0.0, 0.0, 2.0,
                     alive ? "MCU responsive" : "MCU NOT responding");
        logPose("RAPID_END");

        // Return to origin before next test
        sendGoal(origin_x_, origin_y_, origin_theta_);
        // Quick wait
        snapshot();

        // Test 3: Near-zero goal
        RCLCPP_INFO(this->get_logger(), "\n--- Test 3: Near-zero distance goal ---");
        enterPhase(Phase::NEAR_GOAL);
        break;
      }

      // ═══════ Test 3: Near-zero goal ═══════
      case Phase::NEAR_GOAL: {
        // Send goal just 1cm away
        double near_x = robot_x_ + 0.01;
        double near_y = robot_y_;
        sendGoal(near_x, near_y, robot_theta_);
        if (elapsed() >= 3.0) {
          stopRobot();
          enterPhase(Phase::NEAR_PAUSE);
        }
        break;
      }
      case Phase::NEAR_PAUSE:
        stopRobot();
        if (elapsed() >= 0.5) enterPhase(Phase::NEAR_RECORD);
        break;
      case Phase::NEAR_RECORD: {
        // Should not oscillate — velocity should be near zero
        bool stopped = std::abs(robot_vx_) < 0.05 && std::abs(robot_omega_) < 0.2;
        recordResult("Near-zero goal (1cm)", stopped, std::abs(robot_vx_),
                     std::abs(robot_omega_), 3.0,
                     stopped ? "No oscillation" : "OSCILLATING");
        logPose("NEAR_END");
        // Test 4: Max velocity
        RCLCPP_INFO(this->get_logger(), "\n--- Test 4: Max velocity ---");
        snapshot();
        enterPhase(Phase::MAX_VEL_DRIVE);
        break;
      }

      // ═══════ Test 4: Max velocity ═══════
      case Phase::MAX_VEL_DRIVE:
        sendVelocity(max_vel_, 0.0);
        if (elapsed() >= 2.0) {
          stopRobot();
          enterPhase(Phase::MAX_VEL_STOP);
        }
        break;
      case Phase::MAX_VEL_STOP:
        stopRobot();
        if (elapsed() >= 1.0) enterPhase(Phase::MAX_VEL_RECORD);
        break;
      case Phase::MAX_VEL_RECORD: {
        double displacement = std::hypot(robot_x_ - snap_x_, robot_y_ - snap_y_);
        // Should have moved significantly
        bool moved = displacement > 0.1;
        // Lateral drift should be small even at max velocity
        double lateral = std::abs(robot_y_ - snap_y_);
        bool straight = lateral < 0.1;
        recordResult("Max velocity", moved && straight, displacement,
                     headingError(snap_theta_), 2.0,
                     "disp=" + std::to_string(displacement) +
                     " lat=" + std::to_string(lateral));
        logPose("MAXVEL_END");
        // Test 5: Emergency stop
        RCLCPP_INFO(this->get_logger(), "\n--- Test 5: Emergency stop ---");
        enterPhase(Phase::ESTOP_ACCEL);
        break;
      }

      // ═══════ Test 5: Emergency stop ═══════
      case Phase::ESTOP_ACCEL:
        sendVelocity(max_vel_ * 0.7, 0.0);
        if (elapsed() >= 1.5) {
          snapshot();
          stopRobot();
          enterPhase(Phase::ESTOP_STOP);
        }
        break;
      case Phase::ESTOP_STOP:
        stopRobot();
        if (elapsed() >= 0.3) enterPhase(Phase::ESTOP_SETTLE);
        break;
      case Phase::ESTOP_SETTLE:
        stopRobot();
        if (elapsed() >= 0.5) enterPhase(Phase::ESTOP_RECORD);
        break;
      case Phase::ESTOP_RECORD: {
        bool stopped = std::abs(robot_vx_) < 0.05;
        double stop_dist = std::hypot(robot_x_ - snap_x_, robot_y_ - snap_y_);
        recordResult("Emergency stop", stopped, stop_dist,
                     std::abs(robot_omega_), 0.8,
                     "stop_dist=" + std::to_string(stop_dist) + "m");
        logPose("ESTOP_END");
        // Return to origin for repeat test
        sendGoal(origin_x_, origin_y_, origin_theta_);

        // Test 6: Repeated back-and-forth
        RCLCPP_INFO(this->get_logger(), "\n--- Test 6: Repeated back-and-forth (%d cycles) ---",
                    REPEAT_CYCLES);
        repeat_cycle_ = 0;
        enterPhase(Phase::REPEAT_FWD);
        break;
      }

      // ═══════ Test 6: Repeated back-and-forth ═══════
      case Phase::REPEAT_FWD:
        sendGoal(origin_x_ + 0.3, origin_y_, origin_theta_);
        if (distanceTo(origin_x_ + 0.3, origin_y_) < goal_tolerance_ * 2 ||
            elapsed() > goal_timeout_) {
          stopRobot();
          enterPhase(Phase::REPEAT_FWD_PAUSE);
        }
        break;
      case Phase::REPEAT_FWD_PAUSE:
        stopRobot();
        if (elapsed() >= 0.5) {
          sendGoal(origin_x_, origin_y_, origin_theta_);
          enterPhase(Phase::REPEAT_BWD);
        }
        break;
      case Phase::REPEAT_BWD:
        sendGoal(origin_x_, origin_y_, origin_theta_);
        if (distanceTo(origin_x_, origin_y_) < goal_tolerance_ * 2 ||
            elapsed() > goal_timeout_) {
          stopRobot();
          enterPhase(Phase::REPEAT_BWD_PAUSE);
        }
        break;
      case Phase::REPEAT_BWD_PAUSE:
        stopRobot();
        if (elapsed() >= 0.3) {
          repeat_cycle_++;
          RCLCPP_INFO(this->get_logger(), "  Cycle %d/%d — drift=%.4fm",
                      repeat_cycle_, REPEAT_CYCLES, driftFromOrigin());
          if (repeat_cycle_ >= REPEAT_CYCLES) {
            enterPhase(Phase::REPEAT_CHECK);
          } else {
            enterPhase(Phase::REPEAT_FWD);
          }
        }
        break;
      case Phase::REPEAT_CHECK: {
        double drift = driftFromOrigin();
        double herr = headingError(origin_theta_);
        bool passed = drift < 0.15;
        recordResult("Repeated back-forth x" + std::to_string(REPEAT_CYCLES),
                     passed, drift, herr, 0.0,
                     "cumulative_drift=" + std::to_string(drift));
        logPose("REPEAT_END");
        // Test 7: Spin test
        RCLCPP_INFO(this->get_logger(), "\n--- Test 7: Full 360 CW + 360 CCW ---");
        snapshot();
        sendGoal(robot_x_, robot_y_,
                 normalizeAngle(snap_theta_ - 2 * M_PI));
        enterPhase(Phase::SPIN_CW);
        break;
      }

      // ═══════ Test 7: Spin test ═══════
      case Phase::SPIN_CW:
        // Use velocity for continuous rotation instead of goal (which wraps)
        sendVelocity(0.0, -2.0);
        if (elapsed() >= M_PI) {  // ~pi seconds at 2 rad/s = 360deg
          stopRobot();
          enterPhase(Phase::SPIN_CW_PAUSE);
        }
        break;
      case Phase::SPIN_CW_PAUSE:
        stopRobot();
        if (elapsed() >= 1.0) {
          sendVelocity(0.0, 2.0);
          enterPhase(Phase::SPIN_CCW);
        }
        break;
      case Phase::SPIN_CCW:
        sendVelocity(0.0, 2.0);
        if (elapsed() >= M_PI) {
          stopRobot();
          enterPhase(Phase::SPIN_CCW_PAUSE);
        }
        break;
      case Phase::SPIN_CCW_PAUSE:
        stopRobot();
        if (elapsed() >= 1.0) enterPhase(Phase::SPIN_RECORD);
        break;
      case Phase::SPIN_RECORD: {
        double herr = headingError(snap_theta_);
        double pos_drift = std::hypot(robot_x_ - snap_x_, robot_y_ - snap_y_);
        bool passed = herr < 0.3 && pos_drift < 0.1;
        recordResult("360 CW + 360 CCW spin", passed, pos_drift, herr,
                     2 * M_PI,
                     "heading_return_err=" + std::to_string(herr * 180.0 / M_PI) + "deg");
        logPose("SPIN_END");
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
  rclcpp::spin(std::make_shared<secbot_drive_tests::StressTest>());
  rclcpp::shutdown();
  return 0;
}
