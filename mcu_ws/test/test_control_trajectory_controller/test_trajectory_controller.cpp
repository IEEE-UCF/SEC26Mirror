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
  return fabs(a - b) <= epsilon;
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
  TEST_ASSERT_TRUE(fabs(cmd.w) < 0.5f);
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

// === Per-Waypoint Velocity Tests ===

void test_traj_per_waypoint_velocity() {
  TrajectoryController::Config cfg;
  cfg.cruise_v = 0.5f;  // Default cruise speed
  cfg.max_v = 2.0f;
  cfg.lookahead_dist = 0.2f;

  TrajectoryController traj(cfg);

  // Create trajectory with different speeds per waypoint
  TrajectoryController::Waypoint waypoints[3];

  // First waypoint: slow speed
  waypoints[0].x = 1.0f;
  waypoints[0].y = 0.0f;
  waypoints[0].v = 0.3f;
  waypoints[0].has_vel = 1;  // Use custom velocity

  // Second waypoint: fast speed
  waypoints[1].x = 2.0f;
  waypoints[1].y = 0.0f;
  waypoints[1].v = 1.5f;
  waypoints[1].has_vel = 1;  // Use custom velocity

  // Third waypoint: use default cruise speed
  waypoints[2].x = 3.0f;
  waypoints[2].y = 0.0f;
  waypoints[2].has_vel = 0;  // Use cfg.cruise_v

  traj.setTrajectory(waypoints, 3);

  TrajectoryController::Pose2D pose;
  pose.x = 0.0f;
  pose.y = 0.0f;
  pose.theta = 0.0f;

  // Update near first waypoint
  TrajectoryController::Command cmd1 = traj.update(pose, 0.1f);

  // Should be using slow speed (0.3 m/s) or close to it
  TEST_ASSERT_LESS_THAN(0.6f, cmd1.v);  // Not using full cruise_v

  // Advance to second waypoint area
  pose.x = 1.5f;
  TrajectoryController::Command cmd2 = traj.update(pose, 0.1f);

  // Should be using faster speed (1.5 m/s) or ramping to it
  TEST_ASSERT_GREATER_THAN(0.5f, cmd2.v);  // Higher than first waypoint

  // Advance to third waypoint area
  pose.x = 2.5f;
  TrajectoryController::Command cmd3 = traj.update(pose, 0.1f);

  // Should be using default cruise_v (0.5 m/s) since has_vel = 0
  TEST_ASSERT_TRUE(floatEqual(0.5f, cmd3.v, 0.3f));
}

void test_traj_waypoint_velocity_override() {
  TrajectoryController::Config cfg;
  cfg.cruise_v = 1.0f;
  cfg.max_v = 2.0f;

  TrajectoryController traj(cfg);

  // Waypoint with explicit zero velocity (stop point)
  TrajectoryController::Waypoint waypoints[2];
  waypoints[0].x = 1.0f;
  waypoints[0].y = 0.0f;
  waypoints[0].v = 0.0f;  // Stop at this point
  waypoints[0].has_vel = 1;

  waypoints[1].x = 2.0f;
  waypoints[1].y = 0.0f;
  waypoints[1].has_vel = 0;  // Resume cruise speed

  traj.setTrajectory(waypoints, 2);

  TrajectoryController::Pose2D pose;
  pose.x = 0.5f;
  pose.y = 0.0f;
  pose.theta = 0.0f;

  TrajectoryController::Command cmd = traj.update(pose, 0.1f);

  // Should be slowing down toward the stop point
  TEST_ASSERT_LESS_THAN(cfg.cruise_v + 0.1f, cmd.v);
}

// === Curvature Saturation Tests ===

void test_traj_preserve_curvature_on_w_saturation() {
  TrajectoryController::Config cfg;
  cfg.cruise_v = 1.0f;
  cfg.max_v = 2.0f;
  cfg.max_w = 1.0f;  // Low max angular velocity
  cfg.lookahead_dist = 0.3f;
  cfg.preserve_curvature_on_w_saturation = true;  // Enable curvature preservation

  TrajectoryController traj(cfg);

  // Create sharp turn that would saturate angular velocity
  TrajectoryController::Waypoint waypoints[3];
  waypoints[0].x = 1.0f;
  waypoints[0].y = 0.0f;

  waypoints[1].x = 1.0f;
  waypoints[1].y = 1.0f;  // 90 degree turn

  waypoints[2].x = 2.0f;
  waypoints[2].y = 1.0f;

  traj.setTrajectory(waypoints, 3);

  // Start just before the turn
  TrajectoryController::Pose2D pose;
  pose.x = 0.9f;
  pose.y = 0.5f;
  pose.theta = 1.57f;  // Facing up, need to turn right

  TrajectoryController::Command cmd = traj.update(pose, 0.1f);

  // When w saturates and curvature preservation is enabled, v should scale down
  if (fabs(cmd.w) >= cfg.max_w - 0.01f) {
    // Angular velocity is saturated
    TEST_ASSERT_LESS_THAN(cfg.cruise_v, cmd.v);  // Linear velocity should be reduced

    // The ratio should preserve curvature: v/r = w, or v = w*r
    // If w is saturated, v must be reduced proportionally
    TEST_ASSERT_GREATER_THAN(0.0f, cmd.v);  // Still moving forward
  }
}

void test_traj_no_curvature_preservation() {
  TrajectoryController::Config cfg;
  cfg.cruise_v = 1.0f;
  cfg.max_v = 2.0f;
  cfg.max_w = 1.0f;
  cfg.lookahead_dist = 0.3f;
  cfg.preserve_curvature_on_w_saturation = false;  // Disable curvature preservation

  TrajectoryController traj(cfg);

  // Sharp turn
  TrajectoryController::Waypoint waypoints[2];
  waypoints[0].x = 1.0f;
  waypoints[0].y = 0.0f;

  waypoints[1].x = 1.0f;
  waypoints[1].y = 1.0f;

  traj.setTrajectory(waypoints, 2);

  TrajectoryController::Pose2D pose;
  pose.x = 0.9f;
  pose.y = 0.5f;
  pose.theta = 1.57f;

  TrajectoryController::Command cmd = traj.update(pose, 0.1f);

  // Without curvature preservation, v may not be scaled down
  // even if w saturates (may cause understeer)
  // Just verify command is generated
  TEST_ASSERT_GREATER_THAN(0.0f, cmd.v);
  TEST_ASSERT_NOT_EQUAL(0.0f, cmd.w);
}

void test_traj_curvature_saturation_scaling() {
  TrajectoryController::Config cfg;
  cfg.cruise_v = 2.0f;  // High cruise speed
  cfg.max_v = 3.0f;
  cfg.max_w = 2.0f;  // Angular velocity limit
  cfg.lookahead_dist = 0.5f;
  cfg.preserve_curvature_on_w_saturation = true;

  TrajectoryController traj(cfg);

  // Very sharp turn requiring high angular velocity
  TrajectoryController::Waypoint waypoints[3];
  waypoints[0].x = 0.0f;
  waypoints[0].y = 0.0f;

  waypoints[1].x = 0.3f;
  waypoints[1].y = 0.0f;

  waypoints[2].x = 0.3f;
  waypoints[2].y = 0.3f;  // Sharp right turn

  traj.setTrajectory(waypoints, 3);

  TrajectoryController::Pose2D pose;
  pose.x = 0.25f;
  pose.y = 0.0f;
  pose.theta = 0.0f;  // Facing right, need to turn up

  TrajectoryController::Command cmd = traj.update(pose, 0.1f);

  // If w is at or near saturation, v should be proportionally reduced
  if (fabs(cmd.w) >= cfg.max_w * 0.95f) {
    // Check that velocity was scaled
    // The desired curvature = w_desired / v_desired
    // After saturation: w_actual = max_w, v_actual = max_w / curvature
    TEST_ASSERT_LESS_THAN(cfg.cruise_v, cmd.v);

    // Velocity should still be positive
    TEST_ASSERT_GREATER_THAN(0.0f, cmd.v);

    // Angular velocity should be at limit
    TEST_ASSERT_TRUE(floatEqual(cfg.max_w, fabs(cmd.w), 0.1f));
  }
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

  // Per-waypoint velocity tests
  RUN_TEST(test_traj_per_waypoint_velocity);
  RUN_TEST(test_traj_waypoint_velocity_override);

  // Curvature saturation tests
  RUN_TEST(test_traj_preserve_curvature_on_w_saturation);
  RUN_TEST(test_traj_no_curvature_preservation);
  RUN_TEST(test_traj_curvature_saturation_scaling);

  return UNITY_END();
}
