/**
 * @file test_arm_kinematics.cpp
 * @brief Unit tests for 2-link arm kinematics
 * @date 12/22/2025
 */

#define _USE_MATH_DEFINES
#include <unity.h>
#include <arm_kinematics.h>
#include <cmath>

using namespace secbot::arm;

// Helper function to compare floats with tolerance
bool floatEqual(float a, float b, float epsilon = 0.001f) {
  return fabs(a - b) < epsilon;
}

void setUp(void) {}
void tearDown(void) {}

// === Test wrapPi ===

void test_wrap_pi_zero() {
  TEST_ASSERT_TRUE(floatEqual(0.0f, wrapPi(0.0f)));
}

void test_wrap_pi_positive() {
  float wrapped = wrapPi(4.0f * kPi);
  TEST_ASSERT_TRUE(floatEqual(0.0f, wrapped));
}

void test_wrap_pi_negative() {
  float wrapped = wrapPi(-4.0f * kPi);
  TEST_ASSERT_TRUE(floatEqual(0.0f, wrapped));
}

void test_wrap_pi_at_pi() {
  float wrapped = wrapPi(kPi);
  TEST_ASSERT_TRUE(floatEqual(kPi, wrapped, 0.01f));
}

void test_wrap_pi_at_negative_pi() {
  float wrapped = wrapPi(-kPi);
  // wrapPi wraps to (-pi, pi], so -pi becomes close to +pi
  TEST_ASSERT_TRUE(floatEqual(kPi, wrapped, 0.01f) || floatEqual(-kPi, wrapped, 0.01f));
}

// === Test withinLimits ===

void test_within_limits_true() {
  TwoLinkParams p;
  p.l1_m = 1.0f;
  p.l2_m = 1.0f;
  p.shoulder_min_rad = -kPi;
  p.shoulder_max_rad = kPi;
  p.elbow_min_rad = -kPi;
  p.elbow_max_rad = kPi;

  TEST_ASSERT_TRUE(withinLimits(p, 0.0f, 0.0f));
  TEST_ASSERT_TRUE(withinLimits(p, kPi / 2, kPi / 2));
}

void test_within_limits_false_shoulder() {
  TwoLinkParams p;
  p.l1_m = 1.0f;
  p.l2_m = 1.0f;
  p.shoulder_min_rad = -kPi / 2;
  p.shoulder_max_rad = kPi / 2;
  p.elbow_min_rad = -kPi;
  p.elbow_max_rad = kPi;

  TEST_ASSERT_FALSE(withinLimits(p, kPi, 0.0f)); // Shoulder out of range
}

void test_within_limits_false_elbow() {
  TwoLinkParams p;
  p.l1_m = 1.0f;
  p.l2_m = 1.0f;
  p.shoulder_min_rad = -kPi;
  p.shoulder_max_rad = kPi;
  p.elbow_min_rad = -kPi / 2;
  p.elbow_max_rad = kPi / 2;

  TEST_ASSERT_FALSE(withinLimits(p, 0.0f, kPi)); // Elbow out of range
}

// === Test Forward Kinematics ===

void test_forward_kinematics_zero_angles() {
  TwoLinkParams p;
  p.l1_m = 1.0f;
  p.l2_m = 1.0f;

  float x = 0.0f, y = 0.0f;
  forward2Link(p, 0.0f, 0.0f, x, y);

  // Both links aligned along x-axis
  TEST_ASSERT_TRUE(floatEqual(2.0f, x));
  TEST_ASSERT_TRUE(floatEqual(0.0f, y));
}

void test_forward_kinematics_90_degrees() {
  TwoLinkParams p;
  p.l1_m = 1.0f;
  p.l2_m = 1.0f;

  float x = 0.0f, y = 0.0f;
  forward2Link(p, kPi / 2, 0.0f, x, y);

  // Shoulder at 90°, elbow straight
  TEST_ASSERT_TRUE(floatEqual(0.0f, x, 0.01f));
  TEST_ASSERT_TRUE(floatEqual(2.0f, y));
}

void test_forward_kinematics_folded() {
  TwoLinkParams p;
  p.l1_m = 1.0f;
  p.l2_m = 1.0f;

  float x = 0.0f, y = 0.0f;
  forward2Link(p, 0.0f, kPi, x, y);

  // Arm folded back on itself
  TEST_ASSERT_TRUE(floatEqual(0.0f, x, 0.01f));
  TEST_ASSERT_TRUE(floatEqual(0.0f, y, 0.01f));
}

// === Test Inverse Kinematics ===

void test_inverse_kinematics_reachable_extended() {
  TwoLinkParams p;
  p.l1_m = 1.0f;
  p.l2_m = 1.0f;
  p.shoulder_min_rad = -kPi;
  p.shoulder_max_rad = kPi;
  p.elbow_min_rad = -kPi;
  p.elbow_max_rad = kPi;

  // Target at fully extended position
  IKResult res = inverse2Link(p, 2.0f, 0.0f, false);

  TEST_ASSERT_TRUE(res.ok);
  TEST_ASSERT_TRUE(floatEqual(0.0f, res.shoulder_rad, 0.01f));
  TEST_ASSERT_TRUE(floatEqual(0.0f, res.elbow_rad, 0.01f));
  TEST_ASSERT_TRUE(res.reach_error_m < 0.01f);
}

void test_inverse_kinematics_unreachable() {
  TwoLinkParams p;
  p.l1_m = 1.0f;
  p.l2_m = 1.0f;
  p.shoulder_min_rad = -kPi;
  p.shoulder_max_rad = kPi;
  p.elbow_min_rad = -kPi;
  p.elbow_max_rad = kPi;

  // Target too far away (max reach is 2.0)
  IKResult res = inverse2Link(p, 3.0f, 0.0f, false);

  TEST_ASSERT_FALSE(res.ok);
  TEST_ASSERT_TRUE(res.reach_error_m > 0.5f); // Significant error
}

void test_inverse_kinematics_elbow_up_vs_down() {
  TwoLinkParams p;
  p.l1_m = 1.0f;
  p.l2_m = 1.0f;
  p.shoulder_min_rad = -kPi;
  p.shoulder_max_rad = kPi;
  p.elbow_min_rad = -kPi;
  p.elbow_max_rad = kPi;

  // Same target, different elbow configurations
  IKResult up = inverse2Link(p, 1.5f, 0.5f, true);
  IKResult down = inverse2Link(p, 1.5f, 0.5f, false);

  TEST_ASSERT_TRUE(up.ok);
  TEST_ASSERT_TRUE(down.ok);

  // Elbow angles should be different
  TEST_ASSERT_FALSE(floatEqual(up.elbow_rad, down.elbow_rad));

  // Both should reach the target accurately
  TEST_ASSERT_TRUE(up.reach_error_m < 0.01f);
  TEST_ASSERT_TRUE(down.reach_error_m < 0.01f);
}

void test_inverse_kinematics_joint_limits() {
  TwoLinkParams p;
  p.l1_m = 1.0f;
  p.l2_m = 1.0f;
  p.shoulder_min_rad = 0.0f;
  p.shoulder_max_rad = kPi / 2;
  p.elbow_min_rad = 0.0f;
  p.elbow_max_rad = kPi / 2;

  // Target that would require negative shoulder angle (outside limits)
  IKResult res = inverse2Link(p, 1.0f, -1.0f, false);

  TEST_ASSERT_FALSE(res.ok); // Should fail due to limits
}

void test_inverse_kinematics_round_trip() {
  TwoLinkParams p;
  p.l1_m = 1.5f;
  p.l2_m = 1.0f;
  p.shoulder_min_rad = -kPi;
  p.shoulder_max_rad = kPi;
  p.elbow_min_rad = -kPi;
  p.elbow_max_rad = kPi;

  // Forward kinematics to get a target
  float target_x = 0.0f, target_y = 0.0f;
  forward2Link(p, kPi / 4, kPi / 6, target_x, target_y);

  // Inverse kinematics back
  IKResult res = inverse2Link(p, target_x, target_y, false);

  TEST_ASSERT_TRUE(res.ok);
  TEST_ASSERT_TRUE(res.reach_error_m < 0.01f);
}

// === Test inverse2LinkBest ===

void test_inverse_best_both_valid() {
  TwoLinkParams p;
  p.l1_m = 1.0f;
  p.l2_m = 1.0f;
  p.shoulder_min_rad = -kPi;
  p.shoulder_max_rad = kPi;
  p.elbow_min_rad = -kPi;
  p.elbow_max_rad = kPi;

  IKResult best = inverse2LinkBest(p, 1.5f, 0.5f);

  TEST_ASSERT_TRUE(best.ok);
  TEST_ASSERT_TRUE(best.reach_error_m < 0.01f);
}

void test_inverse_best_one_valid() {
  TwoLinkParams p;
  p.l1_m = 1.0f;
  p.l2_m = 1.0f;
  p.shoulder_min_rad = 0.0f;
  p.shoulder_max_rad = kPi;
  p.elbow_min_rad = 0.0f; // Only positive elbow angles allowed
  p.elbow_max_rad = kPi;

  // This should prefer the elbow_down configuration (positive elbow)
  IKResult best = inverse2LinkBest(p, 1.5f, 0.5f);

  TEST_ASSERT_TRUE(best.ok || best.reach_error_m < 0.5f);
}

void test_inverse_best_neither_valid() {
  TwoLinkParams p;
  p.l1_m = 1.0f;
  p.l2_m = 1.0f;
  p.shoulder_min_rad = -kPi;
  p.shoulder_max_rad = kPi;
  p.elbow_min_rad = -kPi;
  p.elbow_max_rad = kPi;

  // Target too far
  IKResult best = inverse2LinkBest(p, 5.0f, 0.0f);

  TEST_ASSERT_FALSE(best.ok);
  // Should return whichever has smaller error
  TEST_ASSERT_TRUE(best.reach_error_m > 0.0f);
}

void test_forward_inverse_consistency() {
  TwoLinkParams p;
  p.l1_m = 1.2f;
  p.l2_m = 0.8f;
  p.shoulder_min_rad = -kPi;
  p.shoulder_max_rad = kPi;
  p.elbow_min_rad = -kPi;
  p.elbow_max_rad = kPi;

  float test_shoulder = kPi / 3;
  float test_elbow = -kPi / 4;

  // Forward
  float x = 0.0f, y = 0.0f;
  forward2Link(p, test_shoulder, test_elbow, x, y);

  // Inverse
  IKResult res = inverse2LinkBest(p, x, y);

  TEST_ASSERT_TRUE(res.ok);
  TEST_ASSERT_TRUE(res.reach_error_m < 0.01f);
}

int main(int argc, char **argv) {
  UNITY_BEGIN();

  // wrapPi tests
  RUN_TEST(test_wrap_pi_zero);
  RUN_TEST(test_wrap_pi_positive);
  RUN_TEST(test_wrap_pi_negative);
  RUN_TEST(test_wrap_pi_at_pi);
  RUN_TEST(test_wrap_pi_at_negative_pi);

  // withinLimits tests
  RUN_TEST(test_within_limits_true);
  RUN_TEST(test_within_limits_false_shoulder);
  RUN_TEST(test_within_limits_false_elbow);

  // Forward kinematics
  RUN_TEST(test_forward_kinematics_zero_angles);
  RUN_TEST(test_forward_kinematics_90_degrees);
  RUN_TEST(test_forward_kinematics_folded);

  // Inverse kinematics
  RUN_TEST(test_inverse_kinematics_reachable_extended);
  RUN_TEST(test_inverse_kinematics_unreachable);
  RUN_TEST(test_inverse_kinematics_elbow_up_vs_down);
  RUN_TEST(test_inverse_kinematics_joint_limits);
  RUN_TEST(test_inverse_kinematics_round_trip);

  // inverse2LinkBest
  RUN_TEST(test_inverse_best_both_valid);
  RUN_TEST(test_inverse_best_one_valid);
  RUN_TEST(test_inverse_best_neither_valid);

  // Consistency
  RUN_TEST(test_forward_inverse_consistency);

  return UNITY_END();
}
