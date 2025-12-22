/**
 * @file test_trapezoidal_motion_profile.cpp
 * @brief Unit tests for TrapezoidalMotionProfile
 * @date 12/22/2025
 */

#define _USE_MATH_DEFINES
#include <unity.h>
#include <motion_profile.h>
#include <cmath>

// Helper function to compare floats with tolerance
bool floatEqual(float a, float b, float epsilon = 0.001f) {
  return fabs(a - b) < epsilon;
}

void setUp(void) {}
void tearDown(void) {}

// === Configuration Tests ===

void test_trapezoid_default_constructor() {
  TrapezoidalMotionProfile profile;
  TEST_ASSERT_TRUE(profile.isFinished());
  TEST_ASSERT_EQUAL_FLOAT(0.0f, profile.state().pos);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, profile.state().vel);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, profile.state().acc);
}

void test_trapezoid_configure_valid() {
  TrapezoidalMotionProfile::Config cfg;
  cfg.limits.v_max = 2.0f;
  cfg.limits.a_max = 1.0f;
  cfg.limits.d_max = 1.0f;

  TrapezoidalMotionProfile profile(cfg);
  TEST_ASSERT_TRUE(profile.isFinished());
}

void test_trapezoid_configure_zero_velocity() {
  TrapezoidalMotionProfile::Config cfg;
  cfg.limits.v_max = 0.0f;  // Invalid
  cfg.limits.a_max = 1.0f;
  cfg.limits.d_max = 1.0f;

  TrapezoidalMotionProfile profile(cfg);
  // Should handle gracefully (sanitize or clamp)
  TEST_ASSERT_TRUE(profile.isFinished());
}

void test_trapezoid_configure_negative_limits() {
  TrapezoidalMotionProfile::Config cfg;
  cfg.limits.v_max = -2.0f;  // Should be sanitized
  cfg.limits.a_max = -1.0f;
  cfg.limits.d_max = -1.0f;

  TrapezoidalMotionProfile profile(cfg);
  // Should still work, values sanitized internally
  TEST_ASSERT_TRUE(profile.isFinished());
}

// === Goal Setting Tests ===

void test_trapezoid_set_goal_forward() {
  TrapezoidalMotionProfile::Config cfg;
  cfg.limits.v_max = 2.0f;
  cfg.limits.a_max = 1.0f;
  cfg.limits.d_max = 1.0f;

  TrapezoidalMotionProfile profile(cfg);

  MotionGoal goal;
  goal.pos = 10.0f;
  goal.vel = 0.0f;

  profile.setGoal(goal);
  TEST_ASSERT_FALSE(profile.isFinished());
  TEST_ASSERT_EQUAL_FLOAT(10.0f, profile.goal().pos);
}

void test_trapezoid_set_goal_backward() {
  TrapezoidalMotionProfile::Config cfg;
  cfg.limits.v_max = 2.0f;
  cfg.limits.a_max = 1.0f;
  cfg.limits.d_max = 1.0f;

  TrapezoidalMotionProfile profile(cfg);
  profile.reset({10.0f, 0.0f, 0.0f});  // Start at pos=10

  MotionGoal goal;
  goal.pos = 0.0f;  // Go backward
  goal.vel = 0.0f;

  profile.setGoal(goal);
  TEST_ASSERT_FALSE(profile.isFinished());
}

void test_trapezoid_set_goal_zero_distance() {
  TrapezoidalMotionProfile::Config cfg;
  cfg.limits.v_max = 2.0f;
  cfg.limits.a_max = 1.0f;
  cfg.limits.d_max = 1.0f;

  TrapezoidalMotionProfile profile(cfg);

  MotionGoal goal;
  goal.pos = 0.0f;  // Already at 0
  goal.vel = 0.0f;

  profile.setGoal(goal);
  TEST_ASSERT_TRUE(profile.isFinished());  // Should be instantly finished
}

// === Reset Tests ===

void test_trapezoid_reset_to_zero() {
  TrapezoidalMotionProfile profile;
  profile.reset();

  TEST_ASSERT_EQUAL_FLOAT(0.0f, profile.state().pos);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, profile.state().vel);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, profile.state().acc);
  TEST_ASSERT_TRUE(profile.isFinished());
}

void test_trapezoid_reset_to_custom_state() {
  TrapezoidalMotionProfile profile;

  MotionState initial;
  initial.pos = 5.0f;
  initial.vel = 1.0f;
  initial.acc = 0.5f;

  profile.reset(initial);

  TEST_ASSERT_EQUAL_FLOAT(5.0f, profile.state().pos);
  TEST_ASSERT_EQUAL_FLOAT(1.0f, profile.state().vel);
  TEST_ASSERT_EQUAL_FLOAT(0.5f, profile.state().acc);
}

// === Profile Generation Tests ===

void test_trapezoid_acceleration_phase() {
  TrapezoidalMotionProfile::Config cfg;
  cfg.limits.v_max = 2.0f;
  cfg.limits.a_max = 1.0f;
  cfg.limits.d_max = 1.0f;

  TrapezoidalMotionProfile profile(cfg);

  MotionGoal goal;
  goal.pos = 100.0f;  // Far away
  goal.vel = 0.0f;

  profile.setGoal(goal);

  // Update for 1 second - should accelerate
  MotionState st = profile.update(1.0f);

  // After 1s at a_max=1: v = 1 m/s, pos = 0.5 m
  TEST_ASSERT_TRUE(floatEqual(1.0f, st.vel, 0.1f));
  TEST_ASSERT_GREATER_THAN(0.0f, st.pos);
  TEST_ASSERT_FALSE(profile.isFinished());
}

void test_trapezoid_cruise_phase() {
  TrapezoidalMotionProfile::Config cfg;
  cfg.limits.v_max = 2.0f;
  cfg.limits.a_max = 1.0f;
  cfg.limits.d_max = 1.0f;

  TrapezoidalMotionProfile profile(cfg);

  MotionGoal goal;
  goal.pos = 100.0f;
  goal.vel = 0.0f;

  profile.setGoal(goal);

  // Accelerate to max velocity (2 seconds)
  for(int i = 0; i < 20; i++) {
    profile.update(0.1f);
  }

  MotionState st = profile.state();
  // Should be at or near max velocity
  TEST_ASSERT_TRUE(floatEqual(2.0f, st.vel, 0.2f));
}

void test_trapezoid_deceleration_phase() {
  TrapezoidalMotionProfile::Config cfg;
  cfg.limits.v_max = 2.0f;
  cfg.limits.a_max = 1.0f;
  cfg.limits.d_max = 1.0f;
  cfg.pos_tol = 0.1f;
  cfg.vel_tol = 0.1f;

  TrapezoidalMotionProfile profile(cfg);

  MotionGoal goal;
  goal.pos = 10.0f;
  goal.vel = 0.0f;

  profile.setGoal(goal);

  // Run profile to completion
  for(int i = 0; i < 200 && !profile.isFinished(); i++) {
    profile.update(0.1f);
  }

  // Should finish near the goal
  TEST_ASSERT_TRUE(profile.isFinished());
  TEST_ASSERT_TRUE(floatEqual(10.0f, profile.state().pos, 0.2f));
  TEST_ASSERT_TRUE(floatEqual(0.0f, profile.state().vel, 0.2f));
}

// === Edge Cases ===

void test_trapezoid_instant_arrival() {
  TrapezoidalMotionProfile::Config cfg;
  cfg.limits.v_max = 100.0f;  // Very high
  cfg.limits.a_max = 1000.0f;
  cfg.limits.d_max = 1000.0f;
  cfg.pos_tol = 0.5f;

  TrapezoidalMotionProfile profile(cfg);

  MotionGoal goal;
  goal.pos = 0.1f;  // Very close
  goal.vel = 0.0f;

  profile.setGoal(goal);
  profile.update(0.1f);

  // Should finish quickly or instantly
  TEST_ASSERT_TRUE(profile.isFinished() || floatEqual(0.1f, profile.state().pos, 0.2f));
}

void test_trapezoid_dt_clamping_min() {
  TrapezoidalMotionProfile::Config cfg;
  cfg.limits.v_max = 2.0f;
  cfg.limits.a_max = 1.0f;
  cfg.limits.d_max = 1.0f;
  cfg.min_dt = 0.001f;

  TrapezoidalMotionProfile profile(cfg);

  MotionGoal goal;
  goal.pos = 10.0f;
  goal.vel = 0.0f;

  profile.setGoal(goal);

  // Update with very small dt (should be clamped to min_dt)
  MotionState st = profile.update(0.0000001f);

  // Should still work without exploding
  TEST_ASSERT_TRUE(isfinite(st.pos));
  TEST_ASSERT_TRUE(isfinite(st.vel));
}

void test_trapezoid_dt_clamping_max() {
  TrapezoidalMotionProfile::Config cfg;
  cfg.limits.v_max = 2.0f;
  cfg.limits.a_max = 1.0f;
  cfg.limits.d_max = 1.0f;
  cfg.max_dt = 0.1f;

  TrapezoidalMotionProfile profile(cfg);

  MotionGoal goal;
  goal.pos = 10.0f;
  goal.vel = 0.0f;

  profile.setGoal(goal);

  // Update with very large dt (should be clamped to max_dt)
  MotionState st = profile.update(10.0f);

  // Should not jump too far
  TEST_ASSERT_LESS_THAN(1.0f, st.pos);  // Shouldn't travel too far in one step
}

void test_trapezoid_negative_goal() {
  TrapezoidalMotionProfile::Config cfg;
  cfg.limits.v_max = 2.0f;
  cfg.limits.a_max = 1.0f;
  cfg.limits.d_max = 1.0f;

  TrapezoidalMotionProfile profile(cfg);

  MotionGoal goal;
  goal.pos = -10.0f;  // Negative position
  goal.vel = 0.0f;

  profile.setGoal(goal);

  // Should move backward
  profile.update(0.1f);
  MotionState st = profile.state();

  TEST_ASSERT_LESS_THAN(0.0f, st.vel);  // Negative velocity
  TEST_ASSERT_FALSE(profile.isFinished());
}

void test_trapezoid_very_small_distance() {
  TrapezoidalMotionProfile::Config cfg;
  cfg.limits.v_max = 2.0f;
  cfg.limits.a_max = 1.0f;
  cfg.limits.d_max = 1.0f;
  cfg.pos_tol = 0.01f;

  TrapezoidalMotionProfile profile(cfg);

  MotionGoal goal;
  goal.pos = 0.001f;  // Very tiny distance
  goal.vel = 0.0f;

  profile.setGoal(goal);

  // Should finish within tolerance
  for(int i = 0; i < 10; i++) {
    profile.update(0.01f);
    if(profile.isFinished()) break;
  }

  TEST_ASSERT_TRUE(profile.isFinished());
}

// === Finished Detection Tests ===

void test_trapezoid_finished_at_goal() {
  TrapezoidalMotionProfile::Config cfg;
  cfg.limits.v_max = 2.0f;
  cfg.limits.a_max = 1.0f;
  cfg.limits.d_max = 1.0f;
  cfg.pos_tol = 0.1f;
  cfg.vel_tol = 0.1f;

  TrapezoidalMotionProfile profile(cfg);

  MotionGoal goal;
  goal.pos = 5.0f;
  goal.vel = 0.0f;

  profile.setGoal(goal);

  // Run to completion
  for(int i = 0; i < 200; i++) {
    profile.update(0.05f);
    if(profile.isFinished()) break;
  }

  TEST_ASSERT_TRUE(profile.isFinished());
  TEST_ASSERT_TRUE(floatEqual(5.0f, profile.state().pos, 0.15f));
}

void test_trapezoid_not_finished_during_motion() {
  TrapezoidalMotionProfile::Config cfg;
  cfg.limits.v_max = 2.0f;
  cfg.limits.a_max = 1.0f;
  cfg.limits.d_max = 1.0f;

  TrapezoidalMotionProfile profile(cfg);

  MotionGoal goal;
  goal.pos = 100.0f;
  goal.vel = 0.0f;

  profile.setGoal(goal);

  // Should not be finished after just one update
  profile.update(0.1f);
  TEST_ASSERT_FALSE(profile.isFinished());
}

// === Multiple Updates Test ===

void test_trapezoid_consistent_updates() {
  TrapezoidalMotionProfile::Config cfg;
  cfg.limits.v_max = 1.0f;
  cfg.limits.a_max = 0.5f;
  cfg.limits.d_max = 0.5f;

  TrapezoidalMotionProfile profile(cfg);

  MotionGoal goal;
  goal.pos = 10.0f;
  goal.vel = 0.0f;

  profile.setGoal(goal);

  float last_pos = 0.0f;

  // Position should monotonically increase
  for(int i = 0; i < 50 && !profile.isFinished(); i++) {
    MotionState st = profile.update(0.1f);
    TEST_ASSERT_GREATER_OR_EQUAL(last_pos - 0.001f, st.pos);  // Allow tiny numerical error
    last_pos = st.pos;
  }
}

int main(int argc, char **argv) {
  UNITY_BEGIN();

  // Configuration tests
  RUN_TEST(test_trapezoid_default_constructor);
  RUN_TEST(test_trapezoid_configure_valid);
  RUN_TEST(test_trapezoid_configure_zero_velocity);
  RUN_TEST(test_trapezoid_configure_negative_limits);

  // Goal setting tests
  RUN_TEST(test_trapezoid_set_goal_forward);
  RUN_TEST(test_trapezoid_set_goal_backward);
  RUN_TEST(test_trapezoid_set_goal_zero_distance);

  // Reset tests
  RUN_TEST(test_trapezoid_reset_to_zero);
  RUN_TEST(test_trapezoid_reset_to_custom_state);

  // Profile generation tests
  RUN_TEST(test_trapezoid_acceleration_phase);
  RUN_TEST(test_trapezoid_cruise_phase);
  RUN_TEST(test_trapezoid_deceleration_phase);

  // Edge case tests
  RUN_TEST(test_trapezoid_instant_arrival);
  RUN_TEST(test_trapezoid_dt_clamping_min);
  RUN_TEST(test_trapezoid_dt_clamping_max);
  RUN_TEST(test_trapezoid_negative_goal);
  RUN_TEST(test_trapezoid_very_small_distance);

  // Finished detection tests
  RUN_TEST(test_trapezoid_finished_at_goal);
  RUN_TEST(test_trapezoid_not_finished_during_motion);

  // Consistency test
  RUN_TEST(test_trapezoid_consistent_updates);

  return UNITY_END();
}
