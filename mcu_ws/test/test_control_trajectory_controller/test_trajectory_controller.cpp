/**
 * @file test_trajectory_controller.cpp
 * @brief Unit tests for TrajectoryController (waypoint follower)
 * @date 12/22/2025
 */

#define _USE_MATH_DEFINES
#include <unity.h>
#include <traj_controller.h>
#include <cmath>

bool floatEqual(float a, float b, float epsilon = 0.001f) {
  return fabs(a - b) < epsilon;
}

void setUp(void) {}
void tearDown(void) {}

// === Configuration Tests ===

void test_traj_default_constructor() {
  TrajectoryController traj;
  TEST_ASSERT_FALSE(traj.hasTrajectory());
  TEST_ASSERT_TRUE(traj.isFinished());
}

void test_traj_configure() {
  TrajectoryController::Config cfg;
  cfg.lookahead_dist = 0.5f;
  cfg.cruise_v = 1.0f;
  cfg.max_v = 2.0f;
  cfg.max_w = 4.0f;

  TrajectoryController traj(cfg);
  TEST_ASSERT_EQUAL_FLOAT(0.5f, traj.config().lookahead_dist);
  TEST_ASSERT_EQUAL_FLOAT(1.0f, traj.config().cruise_v);
}

// === Trajectory Setting Tests ===

void test_traj_set_trajectory() {
  TrajectoryController traj;

  TrajectoryController::Waypoint wps[3] = {
    {0.0f, 0.0f},
    {1.0f, 0.0f},
    {2.0f, 0.0f}
  };

  traj.setTrajectory(wps, 3);
  TEST_ASSERT_TRUE(traj.hasTrajectory());
  TEST_ASSERT_FALSE(traj.isFinished());
}

void test_traj_clear_trajectory() {
  TrajectoryController traj;

  TrajectoryController::Waypoint wps[2] = {
    {0.0f, 0.0f},
    {1.0f, 0.0f}
  };

  traj.setTrajectory(wps, 2);
  TEST_ASSERT_TRUE(traj.hasTrajectory());

  traj.clearTrajectory();
  TEST_ASSERT_FALSE(traj.hasTrajectory());
}

void test_traj_empty_trajectory() {
  TrajectoryController traj;
  traj.setTrajectory(nullptr, 0);
  TEST_ASSERT_FALSE(traj.hasTrajectory());
}

void test_traj_single_waypoint() {
  TrajectoryController::Config cfg;
  cfg.pos_tol = 0.1f;

  TrajectoryController traj(cfg);

  TrajectoryController::Waypoint wp[1] = {{1.0f, 1.0f}};

  traj.setTrajectory(wp, 1);
  TEST_ASSERT_TRUE(traj.hasTrajectory());
}

// === Waypoint Following Tests ===

void test_traj_straight_line() {
  TrajectoryController::Config cfg;
  cfg.lookahead_dist = 0.3f;
  cfg.cruise_v = 0.5f;
  cfg.pos_tol = 0.1f;

  TrajectoryController traj(cfg);

  TrajectoryController::Waypoint wps[2] = {
    {0.0f, 0.0f},
    {5.0f, 0.0f}
  };

  traj.setTrajectory(wps, 2);

  TrajectoryController::Pose2D pose = {0.0f, 0.0f, 0.0f};

  TrajectoryController::Command cmd = traj.update(pose, 0.1f);

  // Should command forward velocity
  TEST_ASSERT_GREATER_THAN(0.0f, cmd.v);
  // Angular velocity should be near zero (straight line)
  TEST_ASSERT_LESS_THAN(0.5f, fabs(cmd.w));
}

void test_traj_curved_path() {
  TrajectoryController::Config cfg;
  cfg.lookahead_dist = 0.3f;
  cfg.cruise_v = 0.5f;

  TrajectoryController traj(cfg);

  TrajectoryController::Waypoint wps[3] = {
    {0.0f, 0.0f},
    {1.0f, 0.0f},
    {1.0f, 1.0f}
  };

  traj.setTrajectory(wps, 3);

  TrajectoryController::Pose2D pose = {0.9f, 0.0f, 0.0f};

  TrajectoryController::Command cmd = traj.update(pose, 0.1f);

  // Should have both linear and angular velocity
  TEST_ASSERT_GREATER_THAN(0.0f, cmd.v);
  TEST_ASSERT_NOT_EQUAL(0.0f, cmd.w);  // Turning
}

void test_traj_backward_motion() {
  TrajectoryController::Config cfg;
  cfg.lookahead_dist = 0.3f;
  cfg.cruise_v = 0.5f;

  TrajectoryController traj(cfg);

  TrajectoryController::Waypoint wps[2] = {
    {5.0f, 0.0f},
    {0.0f, 0.0f}
  };

  traj.setTrajectory(wps, 2);

  TrajectoryController::Pose2D pose = {5.0f, 0.0f, M_PI};  // Facing backward

  TrajectoryController::Command cmd = traj.update(pose, 0.1f);

  // Should command some velocity
  TEST_ASSERT_NOT_EQUAL(0.0f, cmd.v);
}

// === Lookahead Tests ===

void test_traj_lookahead_distance() {
  TrajectoryController::Config cfg;
  cfg.lookahead_dist = 1.0f;
  cfg.cruise_v = 0.5f;

  TrajectoryController traj(cfg);

  TrajectoryController::Waypoint wps[2] = {
    {0.0f, 0.0f},
    {10.0f, 0.0f}
  };

  traj.setTrajectory(wps, 2);

  TrajectoryController::Pose2D pose = {0.0f, 0.0f, 0.0f};

  TrajectoryController::Command cmd = traj.update(pose, 0.1f);

  // Lookahead point should be ahead
  TEST_ASSERT_GREATER_THAN(0.0f, cmd.lookahead_x);
  TEST_ASSERT_LESS_THAN(2.0f, cmd.lookahead_x);  // Not too far
}

// === Goal Detection Tests ===

void test_traj_reaches_goal() {
  TrajectoryController::Config cfg;
  cfg.lookahead_dist = 0.3f;
  cfg.cruise_v = 0.5f;
  cfg.pos_tol = 0.1f;

  TrajectoryController traj(cfg);

  TrajectoryController::Waypoint wps[2] = {
    {0.0f, 0.0f},
    {2.0f, 0.0f}
  };

  traj.setTrajectory(wps, 2);

  TrajectoryController::Pose2D pose;

  // Simulate motion toward goal
  for(int i = 0; i < 50; i++) {
    pose.x += 0.05f;  // Move forward
    pose.y = 0.0f;
    pose.theta = 0.0f;

    TrajectoryController::Command cmd = traj.update(pose, 0.1f);

    if(cmd.finished) {
      TEST_ASSERT_TRUE(traj.isFinished());
      return;
    }
  }

  // Should have finished or be very close
  TEST_ASSERT_TRUE(traj.isFinished() || floatEqual(2.0f, pose.x, 0.2f));
}

void test_traj_goal_tolerance() {
  TrajectoryController::Config cfg;
  cfg.pos_tol = 0.5f;  // Large tolerance

  TrajectoryController traj(cfg);

  TrajectoryController::Waypoint wps[2] = {
    {0.0f, 0.0f},
    {5.0f, 0.0f}
  };

  traj.setTrajectory(wps, 2);

  // Position close to goal (within tolerance)
  TrajectoryController::Pose2D pose = {4.6f, 0.0f, 0.0f};

  TrajectoryController::Command cmd = traj.update(pose, 0.1f);

  // Should be finished or finishing soon
  TEST_ASSERT_TRUE(cmd.finished || cmd.v < 0.2f);
}

// === Segment Transition Tests ===

void test_traj_multi_waypoint_progress() {
  TrajectoryController::Config cfg;
  cfg.lookahead_dist = 0.3f;
  cfg.cruise_v = 0.5f;
  cfg.advance_tol = 0.2f;

  TrajectoryController traj(cfg);

  TrajectoryController::Waypoint wps[4] = {
    {0.0f, 0.0f},
    {1.0f, 0.0f},
    {2.0f, 0.0f},
    {3.0f, 0.0f}
  };

  traj.setTrajectory(wps, 4);

  TrajectoryController::Pose2D pose = {0.0f, 0.0f, 0.0f};

  // Should make progress through waypoints
  for(int i = 0; i < 100 && !traj.isFinished(); i++) {
    TrajectoryController::Command cmd = traj.update(pose, 0.1f);

    // Simulate motion
    pose.x += cmd.v * 0.1f * cos(pose.theta);
    pose.y += cmd.v * 0.1f * sin(pose.theta);
    pose.theta += cmd.w * 0.1f;
  }

  // Should reach or approach final waypoint
  TEST_ASSERT_GREATER_THAN(2.0f, pose.x);
}

// === chassisToWheelSpeeds Tests ===

void test_traj_chassis_to_wheel_straight() {
  float left, right;
  TrajectoryController::chassisToWheelSpeeds(1.0f, 0.0f, 0.5f, &left, &right);

  // Straight motion: both wheels equal
  TEST_ASSERT_EQUAL_FLOAT(1.0f, left);
  TEST_ASSERT_EQUAL_FLOAT(1.0f, right);
}

void test_traj_chassis_to_wheel_turn_left() {
  float left, right;
  // v=1.0, w=2.0 (turning left), track_width=0.5
  TrajectoryController::chassisToWheelSpeeds(1.0f, 2.0f, 0.5f, &left, &right);

  // left wheel slower, right wheel faster
  TEST_ASSERT_LESS_THAN(right, left);
  TEST_ASSERT_EQUAL_FLOAT(0.5f, left);  // 1.0 - 2.0*(0.5/2) = 0.5
  TEST_ASSERT_EQUAL_FLOAT(1.5f, right); // 1.0 + 2.0*(0.5/2) = 1.5
}

void test_traj_chassis_to_wheel_turn_right() {
  float left, right;
  // v=1.0, w=-2.0 (turning right), track_width=0.5
  TrajectoryController::chassisToWheelSpeeds(1.0f, -2.0f, 0.5f, &left, &right);

  // left wheel faster, right wheel slower
  TEST_ASSERT_GREATER_THAN(right, left);
  TEST_ASSERT_EQUAL_FLOAT(1.5f, left);  // 1.0 - (-2.0)*(0.5/2) = 1.5
  TEST_ASSERT_EQUAL_FLOAT(0.5f, right); // 1.0 + (-2.0)*(0.5/2) = 0.5
}

void test_traj_chassis_to_wheel_spin_in_place() {
  float left, right;
  // v=0, w=2.0 (pure rotation), track_width=0.5
  TrajectoryController::chassisToWheelSpeeds(0.0f, 2.0f, 0.5f, &left, &right);

  // Equal and opposite
  TEST_ASSERT_EQUAL_FLOAT(-0.5f, left);
  TEST_ASSERT_EQUAL_FLOAT(0.5f, right);
  TEST_ASSERT_TRUE(floatEqual(-left, right));
}

// === Edge Cases ===

void test_traj_duplicate_waypoints() {
  TrajectoryController traj;

  TrajectoryController::Waypoint wps[3] = {
    {1.0f, 1.0f},
    {1.0f, 1.0f},  // Duplicate
    {2.0f, 2.0f}
  };

  traj.setTrajectory(wps, 3);

  TrajectoryController::Pose2D pose = {0.0f, 0.0f, 0.0f};

  // Should handle without crashing
  TrajectoryController::Command cmd = traj.update(pose, 0.1f);

  TEST_ASSERT_TRUE(isfinite(cmd.v));
  TEST_ASSERT_TRUE(isfinite(cmd.w));
}

void test_traj_reset() {
  TrajectoryController traj;

  TrajectoryController::Waypoint wps[2] = {
    {0.0f, 0.0f},
    {5.0f, 0.0f}
  };

  traj.setTrajectory(wps, 2);

  TrajectoryController::Pose2D pose = {2.0f, 0.0f, 0.0f};
  traj.update(pose, 0.1f);

  traj.reset();
  TEST_ASSERT_FALSE(traj.isFinished());
}

void test_traj_final_heading() {
  TrajectoryController::Config cfg;
  cfg.use_final_heading = true;
  cfg.heading_tol = 0.1f;

  TrajectoryController traj(cfg);

  TrajectoryController::Waypoint wps[2] = {
    {0.0f, 0.0f, 0.0f, 0, 0.0f, 0},
    {2.0f, 0.0f, 0.0f, 0, M_PI/2, 1}  // 90 degree final heading
  };

  traj.setTrajectory(wps, 2);

  // At goal position but wrong heading
  TrajectoryController::Pose2D pose = {2.0f, 0.0f, 0.0f};

  TrajectoryController::Command cmd = traj.update(pose, 0.1f);

  // Should command rotation
  TEST_ASSERT_NOT_EQUAL(0.0f, cmd.w);
}

int main(int argc, char **argv) {
  UNITY_BEGIN();

  // Configuration tests
  RUN_TEST(test_traj_default_constructor);
  RUN_TEST(test_traj_configure);

  // Trajectory setting tests
  RUN_TEST(test_traj_set_trajectory);
  RUN_TEST(test_traj_clear_trajectory);
  RUN_TEST(test_traj_empty_trajectory);
  RUN_TEST(test_traj_single_waypoint);

  // Waypoint following tests
  RUN_TEST(test_traj_straight_line);
  RUN_TEST(test_traj_curved_path);
  RUN_TEST(test_traj_backward_motion);

  // Lookahead tests
  RUN_TEST(test_traj_lookahead_distance);

  // Goal detection tests
  RUN_TEST(test_traj_reaches_goal);
  RUN_TEST(test_traj_goal_tolerance);

  // Segment transition tests
  RUN_TEST(test_traj_multi_waypoint_progress);

  // chassisToWheelSpeeds tests
  RUN_TEST(test_traj_chassis_to_wheel_straight);
  RUN_TEST(test_traj_chassis_to_wheel_turn_left);
  RUN_TEST(test_traj_chassis_to_wheel_turn_right);
  RUN_TEST(test_traj_chassis_to_wheel_spin_in_place);

  // Edge case tests
  RUN_TEST(test_traj_duplicate_waypoints);
  RUN_TEST(test_traj_reset);
  RUN_TEST(test_traj_final_heading);

  return UNITY_END();
}
