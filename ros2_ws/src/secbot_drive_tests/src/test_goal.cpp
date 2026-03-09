/**
 * @file test_goal.cpp
 * @brief DRIVE_GOAL mode tests — closed-loop position control
 *
 * Tests:
 *   1. Forward goal — drive 0.5m forward, verify arrival
 *   2. Backward goal (reverse) — drive 0.5m backward using reverse flag
 *   3. Turn in place 90 CW — heading-only goal
 *   4. Turn in place 90 CCW — heading-only goal
 *   5. Turn 180 — stress test for heading wrap
 *   6. Diagonal goal — drive to (0.3, 0.3) from origin
 *   7. Square path — 4 sequential goals forming a square, return to origin
 *   8. Return to origin — verify cumulative drift after square
 *
 * Usage:
 *   ros2 run secbot_drive_tests test_goal
 *   ros2 run secbot_drive_tests test_goal --ros-args -p distance:=0.3
 */

#include "secbot_drive_tests/test_harness.hpp"

using namespace std::chrono_literals;

namespace secbot_drive_tests {

class GoalTest : public DriveTestHarness {
 public:
  GoalTest() : DriveTestHarness("test_goal") {
    this->declare_parameter("distance", 0.5);
    distance_ = this->get_parameter("distance").as_double();

    tick_timer_ = this->create_wall_timer(100ms, std::bind(&GoalTest::tick, this));

    RCLCPP_INFO(this->get_logger(), "===================================");
    RCLCPP_INFO(this->get_logger(), "  GOAL MODE TEST SUITE");
    RCLCPP_INFO(this->get_logger(), "  distance=%.2f m  tol=%.3f m",
                distance_, goal_tolerance_);
    RCLCPP_INFO(this->get_logger(), "===================================");
  }

 private:
  enum class Phase {
    WAIT_STATUS,
    // Test 1: Forward goal
    FWD_GOAL, FWD_PAUSE, FWD_RECORD,
    // Test 2: Backward goal (reverse flag)
    BWD_GOAL, BWD_PAUSE, BWD_RECORD,
    // Test 3: Turn CW 90
    CW90_GOAL, CW90_PAUSE, CW90_RECORD,
    // Test 4: Turn CCW 90
    CCW90_GOAL, CCW90_PAUSE, CCW90_RECORD,
    // Test 5: Turn 180
    T180_GOAL, T180_PAUSE, T180_RECORD,
    // Test 6: Turn back to 0
    T0_GOAL, T0_PAUSE, T0_RECORD,
    // Test 7: Diagonal
    DIAG_GOAL, DIAG_PAUSE, DIAG_RECORD,
    // Test 8: Return to origin from diagonal
    DIAG_RTN_GOAL, DIAG_RTN_PAUSE, DIAG_RTN_RECORD,
    // Test 9-12: Square path
    SQ1_GOAL, SQ1_PAUSE,
    SQ2_GOAL, SQ2_PAUSE,
    SQ3_GOAL, SQ3_PAUSE,
    SQ4_GOAL, SQ4_PAUSE, SQ_RECORD,
    // Done
    SUMMARY, DONE
  };

  Phase phase_ = Phase::WAIT_STATUS;
  double distance_;

  // Current goal
  double gx_, gy_, gtheta_;
  bool greverse_ = false;

  void enterPhase(Phase p) {
    phase_ = p;
    resetPhaseTimer();
  }

  void setGoal(double x, double y, double theta, bool reverse = false) {
    gx_ = x; gy_ = y; gtheta_ = theta; greverse_ = reverse;
  }

  bool reachedGoal() {
    return distanceTo(gx_, gy_) < goal_tolerance_;
  }

  bool reachedHeading() {
    return headingError(gtheta_) < heading_tolerance_;
  }

  bool timedOut() {
    return elapsed() > goal_timeout_;
  }

  void tick() {
    switch (phase_) {
      case Phase::WAIT_STATUS:
        if (!has_status_) return;
        captureOrigin();
        RCLCPP_INFO(this->get_logger(), "Status received, starting tests...");
        logPose("ORIGIN");
        // Test 1: Forward goal
        RCLCPP_INFO(this->get_logger(), "\n--- Test 1: Forward goal (%.2fm) ---", distance_);
        setGoal(origin_x_ + distance_, origin_y_, origin_theta_);
        enterPhase(Phase::FWD_GOAL);
        break;

      // ═══════ Test 1: Forward goal ═══════
      case Phase::FWD_GOAL:
        sendGoal(gx_, gy_, gtheta_);
        if (reachedGoal() || timedOut()) {
          stopRobot();
          enterPhase(Phase::FWD_PAUSE);
        }
        break;
      case Phase::FWD_PAUSE:
        stopRobot();
        if (elapsed() >= pause_time_) enterPhase(Phase::FWD_RECORD);
        break;
      case Phase::FWD_RECORD: {
        double err = distanceTo(gx_, gy_);
        bool passed = err < goal_tolerance_ * 3;
        recordResult("Forward goal", passed, err,
                     headingError(gtheta_), elapsed(),
                     timedOut() ? "TIMED OUT" : "");
        logPose("FWD_END");
        // Test 2: Backward (reverse) back to origin
        RCLCPP_INFO(this->get_logger(), "\n--- Test 2: Backward goal (reverse) ---");
        setGoal(origin_x_, origin_y_, origin_theta_, true);
        enterPhase(Phase::BWD_GOAL);
        break;
      }

      // ═══════ Test 2: Backward goal ═══════
      case Phase::BWD_GOAL:
        sendGoal(gx_, gy_, gtheta_, greverse_);
        if (reachedGoal() || timedOut()) {
          stopRobot();
          enterPhase(Phase::BWD_PAUSE);
        }
        break;
      case Phase::BWD_PAUSE:
        stopRobot();
        if (elapsed() >= pause_time_) enterPhase(Phase::BWD_RECORD);
        break;
      case Phase::BWD_RECORD: {
        double err = distanceTo(gx_, gy_);
        bool passed = err < goal_tolerance_ * 3;
        recordResult("Backward goal (reverse)", passed, err,
                     headingError(gtheta_), elapsed(),
                     timedOut() ? "TIMED OUT" : "");
        logPose("BWD_END");
        // Test 3: Turn CW 90
        RCLCPP_INFO(this->get_logger(), "\n--- Test 3: Turn CW 90deg ---");
        setGoal(robot_x_, robot_y_, origin_theta_ - M_PI / 2.0);
        enterPhase(Phase::CW90_GOAL);
        break;
      }

      // ═══════ Test 3: Turn CW 90 ═══════
      case Phase::CW90_GOAL:
        sendGoal(gx_, gy_, gtheta_);
        if (reachedHeading() || timedOut()) {
          stopRobot();
          enterPhase(Phase::CW90_PAUSE);
        }
        break;
      case Phase::CW90_PAUSE:
        stopRobot();
        if (elapsed() >= pause_time_) enterPhase(Phase::CW90_RECORD);
        break;
      case Phase::CW90_RECORD: {
        double herr = headingError(gtheta_);
        double pos_drift = distanceTo(gx_, gy_);
        bool passed = herr < heading_tolerance_ * 2 && pos_drift < 0.1;
        recordResult("Turn CW 90deg", passed, pos_drift, herr, elapsed(),
                     timedOut() ? "TIMED OUT" : "");
        logPose("CW90_END");
        // Test 4: Turn CCW 90 (back to original heading)
        RCLCPP_INFO(this->get_logger(), "\n--- Test 4: Turn CCW 90deg ---");
        setGoal(robot_x_, robot_y_, origin_theta_);
        enterPhase(Phase::CCW90_GOAL);
        break;
      }

      // ═══════ Test 4: Turn CCW 90 ═══════
      case Phase::CCW90_GOAL:
        sendGoal(gx_, gy_, gtheta_);
        if (reachedHeading() || timedOut()) {
          stopRobot();
          enterPhase(Phase::CCW90_PAUSE);
        }
        break;
      case Phase::CCW90_PAUSE:
        stopRobot();
        if (elapsed() >= pause_time_) enterPhase(Phase::CCW90_RECORD);
        break;
      case Phase::CCW90_RECORD: {
        double herr = headingError(gtheta_);
        double pos_drift = distanceTo(gx_, gy_);
        bool passed = herr < heading_tolerance_ * 2 && pos_drift < 0.1;
        recordResult("Turn CCW 90deg", passed, pos_drift, herr, elapsed(),
                     timedOut() ? "TIMED OUT" : "");
        logPose("CCW90_END");
        // Test 5: Turn 180
        RCLCPP_INFO(this->get_logger(), "\n--- Test 5: Turn 180deg ---");
        setGoal(robot_x_, robot_y_,
                normalizeAngle(origin_theta_ + M_PI));
        enterPhase(Phase::T180_GOAL);
        break;
      }

      // ═══════ Test 5: Turn 180 ═══════
      case Phase::T180_GOAL:
        sendGoal(gx_, gy_, gtheta_);
        if (reachedHeading() || timedOut()) {
          stopRobot();
          enterPhase(Phase::T180_PAUSE);
        }
        break;
      case Phase::T180_PAUSE:
        stopRobot();
        if (elapsed() >= pause_time_) enterPhase(Phase::T180_RECORD);
        break;
      case Phase::T180_RECORD: {
        double herr = headingError(gtheta_);
        bool passed = herr < heading_tolerance_ * 3;
        recordResult("Turn 180deg", passed, distanceTo(gx_, gy_), herr,
                     elapsed(), timedOut() ? "TIMED OUT" : "");
        logPose("T180_END");
        // Test 6: Turn back to 0
        RCLCPP_INFO(this->get_logger(), "\n--- Test 6: Turn back to 0deg ---");
        setGoal(robot_x_, robot_y_, origin_theta_);
        enterPhase(Phase::T0_GOAL);
        break;
      }

      // ═══════ Test 6: Turn back to 0 ═══════
      case Phase::T0_GOAL:
        sendGoal(gx_, gy_, gtheta_);
        if (reachedHeading() || timedOut()) {
          stopRobot();
          enterPhase(Phase::T0_PAUSE);
        }
        break;
      case Phase::T0_PAUSE:
        stopRobot();
        if (elapsed() >= pause_time_) enterPhase(Phase::T0_RECORD);
        break;
      case Phase::T0_RECORD: {
        double herr = headingError(gtheta_);
        bool passed = herr < heading_tolerance_ * 2;
        recordResult("Turn back to 0deg", passed, distanceTo(gx_, gy_),
                     herr, elapsed(), timedOut() ? "TIMED OUT" : "");
        logPose("T0_END");
        // Test 7: Diagonal goal
        RCLCPP_INFO(this->get_logger(), "\n--- Test 7: Diagonal goal ---");
        double diag = distance_ * 0.707;
        setGoal(origin_x_ + diag, origin_y_ + diag, origin_theta_);
        enterPhase(Phase::DIAG_GOAL);
        break;
      }

      // ═══════ Test 7: Diagonal ═══════
      case Phase::DIAG_GOAL:
        sendGoal(gx_, gy_, gtheta_);
        if (reachedGoal() || timedOut()) {
          stopRobot();
          enterPhase(Phase::DIAG_PAUSE);
        }
        break;
      case Phase::DIAG_PAUSE:
        stopRobot();
        if (elapsed() >= pause_time_) enterPhase(Phase::DIAG_RECORD);
        break;
      case Phase::DIAG_RECORD: {
        double err = distanceTo(gx_, gy_);
        bool passed = err < goal_tolerance_ * 5;
        recordResult("Diagonal goal", passed, err,
                     headingError(gtheta_), elapsed(),
                     timedOut() ? "TIMED OUT" : "");
        logPose("DIAG_END");
        // Test 8: Return from diagonal
        RCLCPP_INFO(this->get_logger(), "\n--- Test 8: Return to origin from diagonal ---");
        setGoal(origin_x_, origin_y_, origin_theta_);
        enterPhase(Phase::DIAG_RTN_GOAL);
        break;
      }

      // ═══════ Test 8: Return from diagonal ═══════
      case Phase::DIAG_RTN_GOAL:
        sendGoal(gx_, gy_, gtheta_);
        if (reachedGoal() || timedOut()) {
          stopRobot();
          enterPhase(Phase::DIAG_RTN_PAUSE);
        }
        break;
      case Phase::DIAG_RTN_PAUSE:
        stopRobot();
        if (elapsed() >= pause_time_) enterPhase(Phase::DIAG_RTN_RECORD);
        break;
      case Phase::DIAG_RTN_RECORD: {
        double err = distanceTo(gx_, gy_);
        bool passed = err < goal_tolerance_ * 5;
        recordResult("Return from diagonal", passed, err,
                     headingError(gtheta_), elapsed(),
                     timedOut() ? "TIMED OUT" : "");
        logPose("DIAG_RTN_END");
        // Test 9-12: Square path
        RCLCPP_INFO(this->get_logger(), "\n--- Tests 9-12: Square path (%.2fm sides) ---", distance_);
        double d = distance_;
        setGoal(origin_x_ + d, origin_y_, origin_theta_);
        enterPhase(Phase::SQ1_GOAL);
        break;
      }

      // ═══════ Square path ═══════
      case Phase::SQ1_GOAL:
        sendGoal(gx_, gy_, gtheta_);
        if (reachedGoal() || timedOut()) {
          stopRobot();
          logPose("SQ1");
          enterPhase(Phase::SQ1_PAUSE);
        }
        break;
      case Phase::SQ1_PAUSE:
        stopRobot();
        if (elapsed() >= pause_time_) {
          double d = distance_;
          setGoal(origin_x_ + d, origin_y_ + d, origin_theta_);
          enterPhase(Phase::SQ2_GOAL);
        }
        break;
      case Phase::SQ2_GOAL:
        sendGoal(gx_, gy_, gtheta_);
        if (reachedGoal() || timedOut()) {
          stopRobot();
          logPose("SQ2");
          enterPhase(Phase::SQ2_PAUSE);
        }
        break;
      case Phase::SQ2_PAUSE:
        stopRobot();
        if (elapsed() >= pause_time_) {
          double d = distance_;
          setGoal(origin_x_, origin_y_ + d, origin_theta_);
          enterPhase(Phase::SQ3_GOAL);
        }
        break;
      case Phase::SQ3_GOAL:
        sendGoal(gx_, gy_, gtheta_);
        if (reachedGoal() || timedOut()) {
          stopRobot();
          logPose("SQ3");
          enterPhase(Phase::SQ3_PAUSE);
        }
        break;
      case Phase::SQ3_PAUSE:
        stopRobot();
        if (elapsed() >= pause_time_) {
          setGoal(origin_x_, origin_y_, origin_theta_);
          enterPhase(Phase::SQ4_GOAL);
        }
        break;
      case Phase::SQ4_GOAL:
        sendGoal(gx_, gy_, gtheta_);
        if (reachedGoal() || timedOut()) {
          stopRobot();
          logPose("SQ4");
          enterPhase(Phase::SQ4_PAUSE);
        }
        break;
      case Phase::SQ4_PAUSE:
        stopRobot();
        if (elapsed() >= pause_time_) enterPhase(Phase::SQ_RECORD);
        break;
      case Phase::SQ_RECORD: {
        double drift = driftFromOrigin();
        double herr = headingError(origin_theta_);
        bool passed = drift < goal_tolerance_ * 10;
        recordResult("Square path (cumulative drift)", passed, drift, herr,
                     0.0, "drift=" + std::to_string(drift) + "m");
        logPose("SQUARE_END");
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
  rclcpp::spin(std::make_shared<secbot_drive_tests::GoalTest>());
  rclcpp::shutdown();
  return 0;
}
