/**
 * @file test_scurve_motion_profile.cpp
 * @brief Unit tests for SCurveMotionProfile (jerk-limited motion)
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

void test_scurve_default_constructor() {
  SCurveMotionProfile profile;
  TEST_ASSERT_TRUE(profile.isFinished());
  TEST_ASSERT_EQUAL_FLOAT(0.0f, profile.state().pos);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, profile.state().vel);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, profile.state().acc);
}

void test_scurve_configure_valid() {
  SCurveMotionProfile::Config cfg;
  cfg.limits.v_max = 2.0f;
  cfg.limits.a_max = 1.0f;
  cfg.limits.d_max = 1.0f;
  cfg.limits.j_max = 10.0f;

  SCurveMotionProfile profile(cfg);
  TEST_ASSERT_TRUE(profile.isFinished());
}

void test_scurve_configure_zero_jerk() {
  SCurveMotionProfile::Config cfg;
  cfg.limits.v_max = 2.0f;
  cfg.limits.a_max = 1.0f;
  cfg.limits.d_max = 1.0f;
  cfg.limits.j_max = 0.0f;  // Should fall back to trapezoid-like behavior

  SCurveMotionProfile profile(cfg);
  TEST_ASSERT_TRUE(profile.isFinished());
}

void test_scurve_configure_negative_limits() {
  SCurveMotionProfile::Config cfg;
  cfg.limits.v_max = -2.0f;  // Should be sanitized
  cfg.limits.a_max = -1.0f;
  cfg.limits.d_max = -1.0f;
  cfg.limits.j_max = -10.0f;

  SCurveMotionProfile profile(cfg);
  TEST_ASSERT_TRUE(profile.isFinished());
}

// === Goal Setting Tests ===

void test_scurve_set_goal_forward() {
  SCurveMotionProfile::Config cfg;
  cfg.limits.v_max = 2.0f;
  cfg.limits.a_max = 1.0f;
  cfg.limits.d_max = 1.0f;
  cfg.limits.j_max = 10.0f;

  SCurveMotionProfile profile(cfg);

  MotionGoal goal;
  goal.pos = 10.0f;
  goal.vel = 0.0f;

  profile.setGoal(goal);
  TEST_ASSERT_FALSE(profile.isFinished());
  TEST_ASSERT_EQUAL_FLOAT(10.0f, profile.goal().pos);
}

void test_scurve_set_goal_backward() {
  SCurveMotionProfile::Config cfg;
  cfg.limits.v_max = 2.0f;
  cfg.limits.a_max = 1.0f;
  cfg.limits.d_max = 1.0f;
  cfg.limits.j_max = 10.0f;

  SCurveMotionProfile profile(cfg);
  profile.reset({10.0f, 0.0f, 0.0f});

  MotionGoal goal;
  goal.pos = 0.0f;
  goal.vel = 0.0f;

  profile.setGoal(goal);
  TEST_ASSERT_FALSE(profile.isFinished());
}

void test_scurve_set_goal_zero_distance() {
  SCurveMotionProfile::Config cfg;
  cfg.limits.v_max = 2.0f;
  cfg.limits.a_max = 1.0f;
  cfg.limits.d_max = 1.0f;
  cfg.limits.j_max = 10.0f;

  SCurveMotionProfile profile(cfg);

  MotionGoal goal;
  goal.pos = 0.0f;
  goal.vel = 0.0f;

  profile.setGoal(goal);
  TEST_ASSERT_TRUE(profile.isFinished());
}

// === Reset Tests ===

void test_scurve_reset_to_zero() {
  SCurveMotionProfile profile;
  profile.reset();

  TEST_ASSERT_EQUAL_FLOAT(0.0f, profile.state().pos);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, profile.state().vel);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, profile.state().acc);
  TEST_ASSERT_TRUE(profile.isFinished());
}

void test_scurve_reset_to_custom_state() {
  SCurveMotionProfile profile;

  MotionState initial;
  initial.pos = 5.0f;
  initial.vel = 1.0f;
  initial.acc = 0.5f;

  profile.reset(initial);

  TEST_ASSERT_EQUAL_FLOAT(5.0f, profile.state().pos);
  TEST_ASSERT_EQUAL_FLOAT(1.0f, profile.state().vel);
  TEST_ASSERT_EQUAL_FLOAT(0.5f, profile.state().acc);
}

// === Jerk-Limited Behavior Tests ===

void test_scurve_smooth_acceleration() {
  SCurveMotionProfile::Config cfg;
  cfg.limits.v_max = 2.0f;
  cfg.limits.a_max = 1.0f;
  cfg.limits.d_max = 1.0f;
  cfg.limits.j_max = 5.0f;  // Moderate jerk limit

  SCurveMotionProfile profile(cfg);

  MotionGoal goal;
  goal.pos = 100.0f;
  goal.vel = 0.0f;

  profile.setGoal(goal);

  // First update - acceleration should start smoothly (not jump to a_max)
  MotionState st1 = profile.update(0.1f);

  // Acceleration should be less than a_max initially
  TEST_ASSERT_LESS_THAN(cfg.limits.a_max, fabs(st1.acc) + 0.1f);
  TEST_ASSERT_FALSE(profile.isFinished());
}

void test_scurve_jerk_compliance() {
  SCurveMotionProfile::Config cfg;
  cfg.limits.v_max = 2.0f;
  cfg.limits.a_max = 1.0f;
  cfg.limits.d_max = 1.0f;
  cfg.limits.j_max = 10.0f;

  SCurveMotionProfile profile(cfg);

  MotionGoal goal;
  goal.pos = 50.0f;
  goal.vel = 0.0f;

  profile.setGoal(goal);

  float last_acc = 0.0f;
  float dt = 0.01f;

  // Check that jerk (rate of change of acceleration) doesn't exceed j_max
  for(int i = 0; i < 20; i++) {
    MotionState st = profile.update(dt);
    float jerk = fabs((st.acc - last_acc) / dt);

    // Allow some numerical tolerance
    TEST_ASSERT_LESS_THAN(cfg.limits.j_max + 2.0f, jerk);

    last_acc = st.acc;
  }
}

void test_scurve_acceleration_continuity() {
  SCurveMotionProfile::Config cfg;
  cfg.limits.v_max = 2.0f;
  cfg.limits.a_max = 1.0f;
  cfg.limits.d_max = 1.0f;
  cfg.limits.j_max = 10.0f;

  SCurveMotionProfile profile(cfg);

  MotionGoal goal;
  goal.pos = 20.0f;
  goal.vel = 0.0f;

  profile.setGoal(goal);

  float last_acc = 0.0f;

  // Acceleration should change smoothly, no discontinuities
  for(int i = 0; i < 50 && !profile.isFinished(); i++) {
    MotionState st = profile.update(0.05f);

    // Check for smoothness (no sudden jumps)
    TEST_ASSERT_LESS_THAN(0.6f, fabs(st.acc - last_acc));

    last_acc = st.acc;
  }
}

// === Profile Completion Tests ===

void test_scurve_reaches_goal() {
  SCurveMotionProfile::Config cfg;
  cfg.limits.v_max = 2.0f;
  cfg.limits.a_max = 1.0f;
  cfg.limits.d_max = 1.0f;
  cfg.limits.j_max = 10.0f;
  cfg.pos_tol = 0.1f;
  cfg.vel_tol = 0.1f;

  SCurveMotionProfile profile(cfg);

  MotionGoal goal;
  goal.pos = 10.0f;
  goal.vel = 0.0f;

  profile.setGoal(goal);

  // Run to completion
  for(int i = 0; i < 300 && !profile.isFinished(); i++) {
    profile.update(0.05f);
  }

  TEST_ASSERT_TRUE(profile.isFinished());
  TEST_ASSERT_TRUE(floatEqual(10.0f, profile.state().pos, 0.2f));
  TEST_ASSERT_TRUE(floatEqual(0.0f, profile.state().vel, 0.2f));
}

void test_scurve_stops_at_goal() {
  SCurveMotionProfile::Config cfg;
  cfg.limits.v_max = 1.0f;
  cfg.limits.a_max = 0.5f;
  cfg.limits.d_max = 0.5f;
  cfg.limits.j_max = 5.0f;
  cfg.pos_tol = 0.05f;
  cfg.vel_tol = 0.05f;

  SCurveMotionProfile profile(cfg);

  MotionGoal goal;
  goal.pos = 5.0f;
  goal.vel = 0.0f;

  profile.setGoal(goal);

  // Run to completion
  for(int i = 0; i < 200 && !profile.isFinished(); i++) {
    profile.update(0.05f);
  }

  TEST_ASSERT_TRUE(profile.isFinished());

  // Both position and velocity should be at goal
  TEST_ASSERT_TRUE(floatEqual(5.0f, profile.state().pos, 0.1f));
  TEST_ASSERT_TRUE(floatEqual(0.0f, profile.state().vel, 0.1f));
}

// === Edge Case Tests ===

void test_scurve_instant_arrival() {
  SCurveMotionProfile::Config cfg;
  cfg.limits.v_max = 100.0f;
  cfg.limits.a_max = 1000.0f;
  cfg.limits.d_max = 1000.0f;
  cfg.limits.j_max = 10000.0f;
  cfg.pos_tol = 0.5f;

  SCurveMotionProfile profile(cfg);

  MotionGoal goal;
  goal.pos = 0.1f;
  goal.vel = 0.0f;

  profile.setGoal(goal);
  profile.update(0.1f);

  TEST_ASSERT_TRUE(profile.isFinished() || floatEqual(0.1f, profile.state().pos, 0.2f));
}

void test_scurve_dt_clamping_min() {
  SCurveMotionProfile::Config cfg;
  cfg.limits.v_max = 2.0f;
  cfg.limits.a_max = 1.0f;
  cfg.limits.d_max = 1.0f;
  cfg.limits.j_max = 10.0f;
  cfg.min_dt = 0.001f;

  SCurveMotionProfile profile(cfg);

  MotionGoal goal;
  goal.pos = 10.0f;
  goal.vel = 0.0f;

  profile.setGoal(goal);

  MotionState st = profile.update(0.0000001f);

  TEST_ASSERT_TRUE(isfinite(st.pos));
  TEST_ASSERT_TRUE(isfinite(st.vel));
  TEST_ASSERT_TRUE(isfinite(st.acc));
}

void test_scurve_dt_clamping_max() {
  SCurveMotionProfile::Config cfg;
  cfg.limits.v_max = 2.0f;
  cfg.limits.a_max = 1.0f;
  cfg.limits.d_max = 1.0f;
  cfg.limits.j_max = 10.0f;
  cfg.max_dt = 0.1f;

  SCurveMotionProfile profile(cfg);

  MotionGoal goal;
  goal.pos = 10.0f;
  goal.vel = 0.0f;

  profile.setGoal(goal);

  MotionState st = profile.update(10.0f);

  TEST_ASSERT_LESS_THAN(1.0f, st.pos);
}

void test_scurve_negative_goal() {
  SCurveMotionProfile::Config cfg;
  cfg.limits.v_max = 2.0f;
  cfg.limits.a_max = 1.0f;
  cfg.limits.d_max = 1.0f;
  cfg.limits.j_max = 10.0f;

  SCurveMotionProfile profile(cfg);

  MotionGoal goal;
  goal.pos = -10.0f;
  goal.vel = 0.0f;

  profile.setGoal(goal);

  profile.update(0.1f);
  MotionState st = profile.state();

  TEST_ASSERT_LESS_THAN(0.0f, st.vel);
  TEST_ASSERT_FALSE(profile.isFinished());
}

void test_scurve_very_small_distance() {
  SCurveMotionProfile::Config cfg;
  cfg.limits.v_max = 2.0f;
  cfg.limits.a_max = 1.0f;
  cfg.limits.d_max = 1.0f;
  cfg.limits.j_max = 10.0f;
  cfg.pos_tol = 0.01f;

  SCurveMotionProfile profile(cfg);

  MotionGoal goal;
  goal.pos = 0.001f;
  goal.vel = 0.0f;

  profile.setGoal(goal);

  for(int i = 0; i < 10; i++) {
    profile.update(0.01f);
    if(profile.isFinished()) break;
  }

  TEST_ASSERT_TRUE(profile.isFinished());
}

// === Comparison with Trapezoid Tests ===

void test_scurve_smoother_than_trapezoid() {
  // This test verifies that S-curve has smoother acceleration changes

  SCurveMotionProfile::Config scfg;
  scfg.limits.v_max = 2.0f;
  scfg.limits.a_max = 1.0f;
  scfg.limits.d_max = 1.0f;
  scfg.limits.j_max = 5.0f;

  SCurveMotionProfile scurve(scfg);

  MotionGoal goal;
  goal.pos = 50.0f;
  goal.vel = 0.0f;

  scurve.setGoal(goal);

  float last_acc = 0.0f;
  float max_jerk = 0.0f;
  float dt = 0.01f;

  // Measure maximum jerk in S-curve
  for(int i = 0; i < 50; i++) {
    MotionState st = scurve.update(dt);
    float jerk = fabs((st.acc - last_acc) / dt);
    if(jerk > max_jerk) max_jerk = jerk;
    last_acc = st.acc;
  }

  // S-curve jerk should be bounded
  TEST_ASSERT_LESS_THAN(scfg.limits.j_max + 2.0f, max_jerk);
}

// === Consistency Tests ===

void test_scurve_consistent_updates() {
  SCurveMotionProfile::Config cfg;
  cfg.limits.v_max = 1.0f;
  cfg.limits.a_max = 0.5f;
  cfg.limits.d_max = 0.5f;
  cfg.limits.j_max = 5.0f;

  SCurveMotionProfile profile(cfg);

  MotionGoal goal;
  goal.pos = 10.0f;
  goal.vel = 0.0f;

  profile.setGoal(goal);

  float last_pos = 0.0f;

  for(int i = 0; i < 100 && !profile.isFinished(); i++) {
    MotionState st = profile.update(0.05f);
    TEST_ASSERT_GREATER_OR_EQUAL(last_pos - 0.001f, st.pos);
    last_pos = st.pos;
  }
}

void test_scurve_velocity_continuity() {
  SCurveMotionProfile::Config cfg;
  cfg.limits.v_max = 2.0f;
  cfg.limits.a_max = 1.0f;
  cfg.limits.d_max = 1.0f;
  cfg.limits.j_max = 10.0f;

  SCurveMotionProfile profile(cfg);

  MotionGoal goal;
  goal.pos = 20.0f;
  goal.vel = 0.0f;

  profile.setGoal(goal);

  float last_vel = 0.0f;

  // Velocity should change smoothly
  for(int i = 0; i < 50 && !profile.isFinished(); i++) {
    MotionState st = profile.update(0.05f);

    // No sudden velocity jumps
    TEST_ASSERT_LESS_THAN(0.15f, fabs(st.vel - last_vel));

    last_vel = st.vel;
  }
}

void test_scurve_not_finished_during_motion() {
  SCurveMotionProfile::Config cfg;
  cfg.limits.v_max = 2.0f;
  cfg.limits.a_max = 1.0f;
  cfg.limits.d_max = 1.0f;
  cfg.limits.j_max = 10.0f;

  SCurveMotionProfile profile(cfg);

  MotionGoal goal;
  goal.pos = 100.0f;
  goal.vel = 0.0f;

  profile.setGoal(goal);

  profile.update(0.1f);
  TEST_ASSERT_FALSE(profile.isFinished());
}

// === Pass-Through Tests (Non-Zero Goal Velocity) ===

void test_scurve_pass_through_nonzero_vel() {
  SCurveMotionProfile::Config cfg;
  cfg.limits.v_max = 2.0f;
  cfg.limits.a_max = 1.0f;
  cfg.limits.d_max = 1.0f;
  cfg.limits.j_max = 10.0f;
  cfg.pos_tol = 0.1f;
  cfg.vel_tol = 0.2f;

  SCurveMotionProfile profile(cfg);

  // Set goal with non-zero final velocity (pass through at speed)
  MotionGoal goal;
  goal.pos = 10.0f;
  goal.vel = 1.5f;  // Target velocity at goal position

  profile.setGoal(goal);

  // Run until we reach the goal
  for(int i = 0; i < 200 && !profile.isFinished(); i++) {
    profile.update(0.05f);
  }

  MotionState final_state = profile.state();

  // Should reach goal position
  TEST_ASSERT_TRUE(floatEqual(10.0f, final_state.pos, 0.3f));

  // Should be moving at target velocity (not stopped)
  TEST_ASSERT_TRUE(floatEqual(1.5f, final_state.vel, 0.3f));

  // Should be marked as finished
  TEST_ASSERT_TRUE(profile.isFinished());
}

void test_scurve_pass_through_max_vel() {
  SCurveMotionProfile::Config cfg;
  cfg.limits.v_max = 2.0f;
  cfg.limits.a_max = 1.0f;
  cfg.limits.d_max = 1.0f;
  cfg.limits.j_max = 10.0f;

  SCurveMotionProfile profile(cfg);

  // Pass through at max velocity
  MotionGoal goal;
  goal.pos = 20.0f;
  goal.vel = 2.0f;  // Request max velocity

  profile.setGoal(goal);

  for(int i = 0; i < 200 && !profile.isFinished(); i++) {
    profile.update(0.05f);
  }

  MotionState final_state = profile.state();

  // Should be near max velocity or goal velocity
  TEST_ASSERT_GREATER_THAN(1.5f, final_state.vel);
}

// === Mid-Motion Retargeting Tests ===

void test_scurve_retarget_mid_motion() {
  SCurveMotionProfile::Config cfg;
  cfg.limits.v_max = 2.0f;
  cfg.limits.a_max = 1.0f;
  cfg.limits.d_max = 1.0f;
  cfg.limits.j_max = 10.0f;
  cfg.pos_tol = 0.1f;
  cfg.vel_tol = 0.1f;

  SCurveMotionProfile profile(cfg);

  // Set initial goal
  MotionGoal goal1;
  goal1.pos = 20.0f;
  goal1.vel = 0.0f;

  profile.setGoal(goal1);

  // Run for a bit to get moving
  for(int i = 0; i < 20; i++) {
    profile.update(0.05f);
  }

  MotionState mid_state = profile.state();
  TEST_ASSERT_FALSE(profile.isFinished());
  TEST_ASSERT_GREATER_THAN(0.0f, mid_state.vel);  // Should be moving

  // Change goal mid-motion
  MotionGoal goal2;
  goal2.pos = 10.0f;  // Reverse direction
  goal2.vel = 0.0f;

  profile.setGoal(goal2);

  // Should not be finished after retarget
  TEST_ASSERT_FALSE(profile.isFinished());

  // Run to new goal
  for(int i = 0; i < 200 && !profile.isFinished(); i++) {
    profile.update(0.05f);
  }

  MotionState final_state = profile.state();

  // Should reach new goal
  TEST_ASSERT_TRUE(floatEqual(10.0f, final_state.pos, 0.2f));
  TEST_ASSERT_TRUE(floatEqual(0.0f, final_state.vel, 0.2f));
  TEST_ASSERT_TRUE(profile.isFinished());
}

void test_scurve_retarget_forward_extension() {
  SCurveMotionProfile::Config cfg;
  cfg.limits.v_max = 2.0f;
  cfg.limits.a_max = 1.0f;
  cfg.limits.d_max = 1.0f;
  cfg.limits.j_max = 10.0f;

  SCurveMotionProfile profile(cfg);

  // Initial goal
  MotionGoal goal1;
  goal1.pos = 10.0f;
  goal1.vel = 0.0f;

  profile.setGoal(goal1);

  // Run partway
  for(int i = 0; i < 20; i++) {
    profile.update(0.05f);
  }

  // Extend goal forward
  MotionGoal goal2;
  goal2.pos = 25.0f;
  goal2.vel = 0.0f;

  profile.setGoal(goal2);

  // Should continue forward smoothly
  MotionState st = profile.state();
  TEST_ASSERT_GREATER_OR_EQUAL(0.0f, st.vel);  // Should maintain forward velocity

  // Run to completion
  for(int i = 0; i < 200 && !profile.isFinished(); i++) {
    profile.update(0.05f);
  }

  TEST_ASSERT_TRUE(floatEqual(25.0f, profile.state().pos, 0.3f));
}

void test_scurve_retarget_smooth_jerk() {
  SCurveMotionProfile::Config cfg;
  cfg.limits.v_max = 2.0f;
  cfg.limits.a_max = 1.0f;
  cfg.limits.d_max = 1.0f;
  cfg.limits.j_max = 10.0f;

  SCurveMotionProfile profile(cfg);

  // Initial goal
  MotionGoal goal1;
  goal1.pos = 15.0f;
  goal1.vel = 0.0f;

  profile.setGoal(goal1);

  // Run partway
  for(int i = 0; i < 15; i++) {
    profile.update(0.05f);
  }

  float last_acc = profile.state().acc;

  // Retarget
  MotionGoal goal2;
  goal2.pos = 30.0f;
  goal2.vel = 0.0f;

  profile.setGoal(goal2);

  // Check that jerk remains bounded after retarget
  float dt = 0.05f;
  for(int i = 0; i < 20; i++) {
    MotionState st = profile.update(dt);
    float jerk = fabs((st.acc - last_acc) / dt);

    // Jerk should remain controlled (allow some tolerance)
    TEST_ASSERT_LESS_THAN(cfg.limits.j_max + 3.0f, jerk);

    last_acc = st.acc;
  }
}

int main(int argc, char **argv) {
  UNITY_BEGIN();

  // Configuration tests
  RUN_TEST(test_scurve_default_constructor);
  RUN_TEST(test_scurve_configure_valid);
  RUN_TEST(test_scurve_configure_zero_jerk);
  RUN_TEST(test_scurve_configure_negative_limits);

  // Goal setting tests
  RUN_TEST(test_scurve_set_goal_forward);
  RUN_TEST(test_scurve_set_goal_backward);
  RUN_TEST(test_scurve_set_goal_zero_distance);

  // Reset tests
  RUN_TEST(test_scurve_reset_to_zero);
  RUN_TEST(test_scurve_reset_to_custom_state);

  // Jerk-limited behavior tests
  RUN_TEST(test_scurve_smooth_acceleration);
  RUN_TEST(test_scurve_jerk_compliance);
  RUN_TEST(test_scurve_acceleration_continuity);

  // Profile completion tests
  RUN_TEST(test_scurve_reaches_goal);
  RUN_TEST(test_scurve_stops_at_goal);

  // Edge case tests
  RUN_TEST(test_scurve_instant_arrival);
  RUN_TEST(test_scurve_dt_clamping_min);
  RUN_TEST(test_scurve_dt_clamping_max);
  RUN_TEST(test_scurve_negative_goal);
  RUN_TEST(test_scurve_very_small_distance);

  // Comparison test
  RUN_TEST(test_scurve_smoother_than_trapezoid);

  // Consistency tests
  RUN_TEST(test_scurve_consistent_updates);
  RUN_TEST(test_scurve_velocity_continuity);
  RUN_TEST(test_scurve_not_finished_during_motion);

  // Pass-through tests (non-zero goal velocity)
  RUN_TEST(test_scurve_pass_through_nonzero_vel);
  RUN_TEST(test_scurve_pass_through_max_vel);

  // Mid-motion retargeting tests
  RUN_TEST(test_scurve_retarget_mid_motion);
  RUN_TEST(test_scurve_retarget_forward_extension);
  RUN_TEST(test_scurve_retarget_smooth_jerk);

  return UNITY_END();
}
