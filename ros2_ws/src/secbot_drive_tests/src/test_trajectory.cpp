/**
 * @file test_trajectory.cpp
 * @brief DRIVE_TRAJ mode tests — waypoint following with pure pursuit
 *
 * Tests:
 *   1. Straight line — 5 waypoints in a line, verify arrival
 *   2. L-shaped path — right angle path
 *   3. Square waypoints — 4-corner square path
 *   4. Zigzag path — alternating lateral offsets
 *   5. Dense curve — circular arc via 16+ dense waypoints
 *   6. Return path — reverse path back to origin
 *   7. Long straight — extended distance with many waypoints
 *   8. Single waypoint — degenerate case with just 1 point
 *
 * Usage:
 *   ros2 run secbot_drive_tests test_trajectory
 *   ros2 run secbot_drive_tests test_trajectory --ros-args -p scale:=0.5
 */

#include "secbot_drive_tests/test_harness.hpp"

#include <array>
#include <cmath>

using namespace std::chrono_literals;

namespace secbot_drive_tests {

class TrajectoryTest : public DriveTestHarness {
 public:
  TrajectoryTest() : DriveTestHarness("test_trajectory") {
    this->declare_parameter("scale", 1.0);
    this->declare_parameter("traj_timeout", 15.0);
    scale_ = this->get_parameter("scale").as_double();
    traj_timeout_ = this->get_parameter("traj_timeout").as_double();

    tick_timer_ = this->create_wall_timer(100ms, std::bind(&TrajectoryTest::tick, this));

    RCLCPP_INFO(this->get_logger(), "===================================");
    RCLCPP_INFO(this->get_logger(), "  TRAJECTORY MODE TEST SUITE");
    RCLCPP_INFO(this->get_logger(), "  scale=%.2f  traj_timeout=%.1fs", scale_, traj_timeout_);
    RCLCPP_INFO(this->get_logger(), "===================================");
  }

 private:
  enum class Phase {
    WAIT_STATUS,
    // Test 1: Straight line
    STRAIGHT_TRAJ, STRAIGHT_PAUSE, STRAIGHT_RECORD,
    // Test 2: L-shaped
    L_TRAJ, L_PAUSE, L_RECORD,
    // Return to origin between tests
    RTN1_GOAL, RTN1_PAUSE,
    // Test 3: Square waypoints
    SQ_TRAJ, SQ_PAUSE, SQ_RECORD,
    // Test 4: Zigzag
    ZIG_TRAJ, ZIG_PAUSE, ZIG_RECORD,
    // Return to origin
    RTN2_GOAL, RTN2_PAUSE,
    // Test 5: Dense curve (arc)
    ARC_TRAJ, ARC_PAUSE, ARC_RECORD,
    // Return to origin
    RTN3_GOAL, RTN3_PAUSE,
    // Test 6: Return path
    RET_TRAJ, RET_PAUSE, RET_RECORD,
    // Test 7: Long straight
    LONG_TRAJ, LONG_PAUSE, LONG_RECORD,
    // Return to origin
    RTN4_GOAL, RTN4_PAUSE,
    // Test 8: Single waypoint
    SINGLE_TRAJ, SINGLE_PAUSE, SINGLE_RECORD,
    // Done
    SUMMARY, DONE
  };

  Phase phase_ = Phase::WAIT_STATUS;
  double scale_;
  double traj_timeout_;

  // Current trajectory endpoint for arrival check
  double target_x_, target_y_;
  std::vector<std::array<double, 2>> current_waypoints_;

  void enterPhase(Phase p) {
    phase_ = p;
    resetPhaseTimer();
  }

  void sendCurrentTrajectory() {
    sendTrajectory(current_waypoints_);
    if (!current_waypoints_.empty()) {
      target_x_ = current_waypoints_.back()[0];
      target_y_ = current_waypoints_.back()[1];
    }
  }

  bool reachedEnd() {
    // Use a slightly larger tolerance for trajectory following
    return distanceTo(target_x_, target_y_) < goal_tolerance_ * 5;
  }

  bool timedOut() {
    return elapsed() > traj_timeout_;
  }

  void returnToOrigin(Phase next_phase) {
    sendGoal(origin_x_, origin_y_, origin_theta_);
    // We'll check in the RTN phases
  }

  // ── Path generators ──
  std::vector<std::array<double, 2>> makeStraightLine(
      double x0, double y0, double x1, double y1, int n) {
    std::vector<std::array<double, 2>> pts;
    for (int i = 0; i <= n; i++) {
      double t = static_cast<double>(i) / n;
      pts.push_back({x0 + t * (x1 - x0), y0 + t * (y1 - y0)});
    }
    return pts;
  }

  std::vector<std::array<double, 2>> makeLShape(double x0, double y0,
                                                 double leg1, double leg2) {
    std::vector<std::array<double, 2>> pts;
    int n = 5;
    // Leg 1: forward in X
    for (int i = 0; i <= n; i++) {
      double t = static_cast<double>(i) / n;
      pts.push_back({x0 + t * leg1, y0});
    }
    // Leg 2: left in Y
    for (int i = 1; i <= n; i++) {
      double t = static_cast<double>(i) / n;
      pts.push_back({x0 + leg1, y0 + t * leg2});
    }
    return pts;
  }

  std::vector<std::array<double, 2>> makeSquare(double x0, double y0, double side) {
    return {
      {x0, y0},
      {x0 + side, y0},
      {x0 + side, y0 + side},
      {x0, y0 + side},
      {x0, y0}
    };
  }

  std::vector<std::array<double, 2>> makeZigzag(double x0, double y0,
                                                  double step_x, double amp_y,
                                                  int n) {
    std::vector<std::array<double, 2>> pts;
    for (int i = 0; i <= n; i++) {
      double x = x0 + i * step_x;
      double y = y0 + ((i % 2 == 0) ? 0.0 : amp_y);
      pts.push_back({x, y});
    }
    return pts;
  }

  std::vector<std::array<double, 2>> makeArc(double cx, double cy,
                                               double radius, double start_angle,
                                               double sweep, int n) {
    std::vector<std::array<double, 2>> pts;
    for (int i = 0; i <= n; i++) {
      double t = static_cast<double>(i) / n;
      double angle = start_angle + t * sweep;
      pts.push_back({cx + radius * std::cos(angle),
                      cy + radius * std::sin(angle)});
    }
    return pts;
  }

  void tick() {
    switch (phase_) {
      case Phase::WAIT_STATUS:
        if (!has_status_) return;
        captureOrigin();
        RCLCPP_INFO(this->get_logger(), "Status received, starting trajectory tests...");
        logPose("ORIGIN");

        // Test 1: Straight line
        RCLCPP_INFO(this->get_logger(), "\n--- Test 1: Straight line (5 waypoints) ---");
        current_waypoints_ = makeStraightLine(
            origin_x_, origin_y_,
            origin_x_ + 0.5 * scale_, origin_y_, 5);
        sendCurrentTrajectory();
        enterPhase(Phase::STRAIGHT_TRAJ);
        break;

      // ═══════ Test 1: Straight line ═══════
      case Phase::STRAIGHT_TRAJ:
        sendCurrentTrajectory();
        if (reachedEnd() || timedOut()) {
          stopRobot();
          enterPhase(Phase::STRAIGHT_PAUSE);
        }
        break;
      case Phase::STRAIGHT_PAUSE:
        stopRobot();
        if (elapsed() >= pause_time_) enterPhase(Phase::STRAIGHT_RECORD);
        break;
      case Phase::STRAIGHT_RECORD: {
        double err = distanceTo(target_x_, target_y_);
        double lateral = std::abs(robot_y_ - origin_y_);
        bool passed = err < 0.15;
        recordResult("Straight line traj", passed, err,
                     headingError(origin_theta_), elapsed(),
                     "lateral_drift=" + std::to_string(lateral));
        logPose("STRAIGHT_END");
        // Test 2: L-shaped
        RCLCPP_INFO(this->get_logger(), "\n--- Test 2: L-shaped path ---");
        current_waypoints_ = makeLShape(
            robot_x_, robot_y_, 0.3 * scale_, 0.3 * scale_);
        sendCurrentTrajectory();
        enterPhase(Phase::L_TRAJ);
        break;
      }

      // ═══════ Test 2: L-shaped ═══════
      case Phase::L_TRAJ:
        sendCurrentTrajectory();
        if (reachedEnd() || timedOut()) {
          stopRobot();
          enterPhase(Phase::L_PAUSE);
        }
        break;
      case Phase::L_PAUSE:
        stopRobot();
        if (elapsed() >= pause_time_) enterPhase(Phase::L_RECORD);
        break;
      case Phase::L_RECORD: {
        double err = distanceTo(target_x_, target_y_);
        bool passed = err < 0.15;
        recordResult("L-shaped path", passed, err, 0.0, elapsed(),
                     timedOut() ? "TIMED OUT" : "");
        logPose("L_END");
        // Return to origin
        enterPhase(Phase::RTN1_GOAL);
        break;
      }

      // ── Return 1 ──
      case Phase::RTN1_GOAL:
        sendGoal(origin_x_, origin_y_, origin_theta_);
        if (distanceTo(origin_x_, origin_y_) < goal_tolerance_ * 3 || elapsed() > goal_timeout_) {
          stopRobot();
          enterPhase(Phase::RTN1_PAUSE);
        }
        break;
      case Phase::RTN1_PAUSE:
        stopRobot();
        if (elapsed() >= pause_time_) {
          // Test 3: Square waypoints
          RCLCPP_INFO(this->get_logger(), "\n--- Test 3: Square waypoints ---");
          current_waypoints_ = makeSquare(
              origin_x_, origin_y_, 0.3 * scale_);
          sendCurrentTrajectory();
          enterPhase(Phase::SQ_TRAJ);
        }
        break;

      // ═══════ Test 3: Square ═══════
      case Phase::SQ_TRAJ:
        sendCurrentTrajectory();
        if (reachedEnd() || timedOut()) {
          stopRobot();
          enterPhase(Phase::SQ_PAUSE);
        }
        break;
      case Phase::SQ_PAUSE:
        stopRobot();
        if (elapsed() >= pause_time_) enterPhase(Phase::SQ_RECORD);
        break;
      case Phase::SQ_RECORD: {
        double drift = driftFromOrigin();
        bool passed = drift < 0.2;
        recordResult("Square waypoints", passed, drift,
                     headingError(origin_theta_), elapsed(),
                     "return_drift=" + std::to_string(drift));
        logPose("SQ_END");
        // Test 4: Zigzag
        RCLCPP_INFO(this->get_logger(), "\n--- Test 4: Zigzag path ---");
        current_waypoints_ = makeZigzag(
            origin_x_, origin_y_, 0.1 * scale_, 0.15 * scale_, 6);
        sendCurrentTrajectory();
        enterPhase(Phase::ZIG_TRAJ);
        break;
      }

      // ═══════ Test 4: Zigzag ═══════
      case Phase::ZIG_TRAJ:
        sendCurrentTrajectory();
        if (reachedEnd() || timedOut()) {
          stopRobot();
          enterPhase(Phase::ZIG_PAUSE);
        }
        break;
      case Phase::ZIG_PAUSE:
        stopRobot();
        if (elapsed() >= pause_time_) enterPhase(Phase::ZIG_RECORD);
        break;
      case Phase::ZIG_RECORD: {
        double err = distanceTo(target_x_, target_y_);
        bool passed = err < 0.2;
        recordResult("Zigzag path", passed, err, 0.0, elapsed(),
                     timedOut() ? "TIMED OUT" : "");
        logPose("ZIG_END");
        // Return to origin
        enterPhase(Phase::RTN2_GOAL);
        break;
      }

      // ── Return 2 ──
      case Phase::RTN2_GOAL:
        sendGoal(origin_x_, origin_y_, origin_theta_);
        if (distanceTo(origin_x_, origin_y_) < goal_tolerance_ * 3 || elapsed() > goal_timeout_) {
          stopRobot();
          enterPhase(Phase::RTN2_PAUSE);
        }
        break;
      case Phase::RTN2_PAUSE:
        stopRobot();
        if (elapsed() >= pause_time_) {
          // Test 5: Dense curve (arc)
          RCLCPP_INFO(this->get_logger(), "\n--- Test 5: Dense arc curve ---");
          double radius = 0.25 * scale_;
          current_waypoints_ = makeArc(
              origin_x_, origin_y_ + radius, radius, -M_PI / 2.0, M_PI, 16);
          sendCurrentTrajectory();
          enterPhase(Phase::ARC_TRAJ);
        }
        break;

      // ═══════ Test 5: Dense arc ═══════
      case Phase::ARC_TRAJ:
        sendCurrentTrajectory();
        if (reachedEnd() || timedOut()) {
          stopRobot();
          enterPhase(Phase::ARC_PAUSE);
        }
        break;
      case Phase::ARC_PAUSE:
        stopRobot();
        if (elapsed() >= pause_time_) enterPhase(Phase::ARC_RECORD);
        break;
      case Phase::ARC_RECORD: {
        double err = distanceTo(target_x_, target_y_);
        bool passed = err < 0.2;
        recordResult("Dense arc curve", passed, err, 0.0, elapsed(),
                     timedOut() ? "TIMED OUT" : "");
        logPose("ARC_END");
        // Return to origin
        enterPhase(Phase::RTN3_GOAL);
        break;
      }

      // ── Return 3 ──
      case Phase::RTN3_GOAL:
        sendGoal(origin_x_, origin_y_, origin_theta_);
        if (distanceTo(origin_x_, origin_y_) < goal_tolerance_ * 3 || elapsed() > goal_timeout_) {
          stopRobot();
          enterPhase(Phase::RTN3_PAUSE);
        }
        break;
      case Phase::RTN3_PAUSE:
        stopRobot();
        if (elapsed() >= pause_time_) {
          // Test 6: Return path (forward then back along same path)
          RCLCPP_INFO(this->get_logger(), "\n--- Test 6: Forward + reverse path ---");
          auto fwd = makeStraightLine(
              origin_x_, origin_y_,
              origin_x_ + 0.4 * scale_, origin_y_ + 0.2 * scale_, 4);
          // Add reverse waypoints
          for (int i = static_cast<int>(fwd.size()) - 2; i >= 0; i--) {
            fwd.push_back(fwd[i]);
          }
          current_waypoints_ = fwd;
          sendCurrentTrajectory();
          enterPhase(Phase::RET_TRAJ);
        }
        break;

      // ═══════ Test 6: Return path ═══════
      case Phase::RET_TRAJ:
        sendCurrentTrajectory();
        if (reachedEnd() || timedOut()) {
          stopRobot();
          enterPhase(Phase::RET_PAUSE);
        }
        break;
      case Phase::RET_PAUSE:
        stopRobot();
        if (elapsed() >= pause_time_) enterPhase(Phase::RET_RECORD);
        break;
      case Phase::RET_RECORD: {
        double drift = driftFromOrigin();
        bool passed = drift < 0.2;
        recordResult("Forward+reverse path", passed, drift,
                     headingError(origin_theta_), elapsed(),
                     "return_drift=" + std::to_string(drift));
        logPose("RET_END");
        // Test 7: Long straight
        RCLCPP_INFO(this->get_logger(), "\n--- Test 7: Long straight (many waypoints) ---");
        current_waypoints_ = makeStraightLine(
            robot_x_, robot_y_,
            robot_x_ + 1.0 * scale_, robot_y_, 20);
        sendCurrentTrajectory();
        enterPhase(Phase::LONG_TRAJ);
        break;
      }

      // ═══════ Test 7: Long straight ═══════
      case Phase::LONG_TRAJ:
        sendCurrentTrajectory();
        if (reachedEnd() || timedOut()) {
          stopRobot();
          enterPhase(Phase::LONG_PAUSE);
        }
        break;
      case Phase::LONG_PAUSE:
        stopRobot();
        if (elapsed() >= pause_time_) enterPhase(Phase::LONG_RECORD);
        break;
      case Phase::LONG_RECORD: {
        double err = distanceTo(target_x_, target_y_);
        bool passed = err < 0.15;
        recordResult("Long straight (20 pts)", passed, err,
                     headingError(origin_theta_), elapsed(),
                     timedOut() ? "TIMED OUT" : "");
        logPose("LONG_END");
        // Return to origin
        enterPhase(Phase::RTN4_GOAL);
        break;
      }

      // ── Return 4 ──
      case Phase::RTN4_GOAL:
        sendGoal(origin_x_, origin_y_, origin_theta_);
        if (distanceTo(origin_x_, origin_y_) < goal_tolerance_ * 3 || elapsed() > goal_timeout_) {
          stopRobot();
          enterPhase(Phase::RTN4_PAUSE);
        }
        break;
      case Phase::RTN4_PAUSE:
        stopRobot();
        if (elapsed() >= pause_time_) {
          // Test 8: Single waypoint
          RCLCPP_INFO(this->get_logger(), "\n--- Test 8: Single waypoint (degenerate) ---");
          current_waypoints_ = {{origin_x_ + 0.3 * scale_, origin_y_}};
          sendCurrentTrajectory();
          enterPhase(Phase::SINGLE_TRAJ);
        }
        break;

      // ═══════ Test 8: Single waypoint ═══════
      case Phase::SINGLE_TRAJ:
        sendCurrentTrajectory();
        if (reachedEnd() || timedOut()) {
          stopRobot();
          enterPhase(Phase::SINGLE_PAUSE);
        }
        break;
      case Phase::SINGLE_PAUSE:
        stopRobot();
        if (elapsed() >= pause_time_) enterPhase(Phase::SINGLE_RECORD);
        break;
      case Phase::SINGLE_RECORD: {
        double err = distanceTo(target_x_, target_y_);
        bool passed = err < 0.2;
        recordResult("Single waypoint", passed, err, 0.0, elapsed(),
                     timedOut() ? "TIMED OUT" : "");
        logPose("SINGLE_END");
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
  rclcpp::spin(std::make_shared<secbot_drive_tests::TrajectoryTest>());
  rclcpp::shutdown();
  return 0;
}
