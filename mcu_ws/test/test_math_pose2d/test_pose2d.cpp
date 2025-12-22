/**
 * @file test_pose2d.cpp
 * @brief Unit tests for Pose2D class
 * @date 12/21/2025
 */

#include <unity.h>
#include <Pose2D.h>
#include <cmath>

// Helper function to compare floats with tolerance
bool floatEqual(float a, float b, float epsilon = 0.0001f) {
  return fabs(a - b) < epsilon;
}

void setUp(void) {
  // Set up code here, runs before each test
}

void tearDown(void) {
  // Clean up code here, runs after each test
}

// Test: Default constructor
void test_pose2d_default_constructor() {
  Pose2D pose;
  TEST_ASSERT_EQUAL_FLOAT(0.0f, pose.x);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, pose.y);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, pose.theta);
}

// Test: Parameterized constructor
void test_pose2d_parameterized_constructor() {
  Pose2D pose(1.5f, 2.5f, PI / 4.0f);
  TEST_ASSERT_EQUAL_FLOAT(1.5f, pose.x);
  TEST_ASSERT_EQUAL_FLOAT(2.5f, pose.y);
  TEST_ASSERT_EQUAL_FLOAT(PI / 4.0f, pose.theta);
}

// Test: Getters
void test_pose2d_getters() {
  Pose2D pose(3.0f, 4.0f, PI / 2.0f);
  TEST_ASSERT_EQUAL_FLOAT(3.0f, pose.getX());
  TEST_ASSERT_EQUAL_FLOAT(4.0f, pose.getY());
  TEST_ASSERT_EQUAL_FLOAT(PI / 2.0f, pose.getTheta());
}

// Test: normalizeTheta basic
void test_pose2d_normalize_theta_basic() {
  Pose2D pose(0.0f, 0.0f, 3.0f * PI);
  pose.normalizeTheta();
  TEST_ASSERT_TRUE(floatEqual(pose.theta, -PI));
}

// Test: normalizeTheta with positive overflow
void test_pose2d_normalize_theta_positive_overflow() {
  Pose2D pose(0.0f, 0.0f, 2.5f * PI);
  pose.normalizeTheta();
  TEST_ASSERT_TRUE(floatEqual(pose.theta, 0.5f * PI));
}

// Test: normalizeTheta with negative overflow
void test_pose2d_normalize_theta_negative_overflow() {
  Pose2D pose(0.0f, 0.0f, -2.5f * PI);
  pose.normalizeTheta();
  TEST_ASSERT_TRUE(floatEqual(pose.theta, -0.5f * PI));
}

// Test: normalizeTheta at exactly PI
void test_pose2d_normalize_theta_at_pi() {
  Pose2D pose(0.0f, 0.0f, PI);
  pose.normalizeTheta();
  TEST_ASSERT_TRUE(floatEqual(pose.theta, -PI) || floatEqual(pose.theta, PI));
}

// Test: normalizeTheta at exactly -PI
void test_pose2d_normalize_theta_at_negative_pi() {
  Pose2D pose(0.0f, 0.0f, -PI);
  pose.normalizeTheta();
  TEST_ASSERT_TRUE(floatEqual(pose.theta, -PI) || floatEqual(pose.theta, PI));
}

// Test: normalizeTheta with zero
void test_pose2d_normalize_theta_zero() {
  Pose2D pose(0.0f, 0.0f, 0.0f);
  pose.normalizeTheta();
  TEST_ASSERT_EQUAL_FLOAT(0.0f, pose.theta);
}

// Test: add method basic
void test_pose2d_add_basic() {
  Pose2D pose1(1.0f, 2.0f, 0.5f);
  Pose2D pose2(3.0f, 4.0f, 0.3f);
  pose1.add(pose2);
  TEST_ASSERT_EQUAL_FLOAT(4.0f, pose1.x);
  TEST_ASSERT_EQUAL_FLOAT(6.0f, pose1.y);
  TEST_ASSERT_EQUAL_FLOAT(0.8f, pose1.theta);
}

// Test: add method with negative values
void test_pose2d_add_negative() {
  Pose2D pose1(5.0f, 10.0f, 1.0f);
  Pose2D pose2(-3.0f, -7.0f, -0.5f);
  pose1.add(pose2);
  TEST_ASSERT_EQUAL_FLOAT(2.0f, pose1.x);
  TEST_ASSERT_EQUAL_FLOAT(3.0f, pose1.y);
  TEST_ASSERT_EQUAL_FLOAT(0.5f, pose1.theta);
}

// Test: add method with zeros
void test_pose2d_add_zeros() {
  Pose2D pose1(1.0f, 2.0f, 3.0f);
  Pose2D pose2(0.0f, 0.0f, 0.0f);
  pose1.add(pose2);
  TEST_ASSERT_EQUAL_FLOAT(1.0f, pose1.x);
  TEST_ASSERT_EQUAL_FLOAT(2.0f, pose1.y);
  TEST_ASSERT_EQUAL_FLOAT(3.0f, pose1.theta);
}

// Test: rotate method basic
void test_pose2d_rotate_basic() {
  Pose2D pose(0.0f, 0.0f, 0.0f);
  pose.rotate(PI / 2.0f);
  TEST_ASSERT_TRUE(floatEqual(pose.theta, PI / 2.0f));
}

// Test: rotate method with existing angle
void test_pose2d_rotate_with_existing_angle() {
  Pose2D pose(0.0f, 0.0f, PI / 4.0f);
  pose.rotate(PI / 4.0f);
  TEST_ASSERT_TRUE(floatEqual(pose.theta, PI / 2.0f));
}

// Test: rotate method full circle
void test_pose2d_rotate_full_circle() {
  Pose2D pose(0.0f, 0.0f, 0.0f);
  pose.rotate(2.0f * PI);
  pose.normalizeTheta();
  TEST_ASSERT_TRUE(floatEqual(pose.theta, 0.0f, 0.001f));
}

// Test: rotate method negative angle
void test_pose2d_rotate_negative() {
  Pose2D pose(0.0f, 0.0f, PI / 2.0f);
  pose.rotate(-PI / 4.0f);
  TEST_ASSERT_TRUE(floatEqual(pose.theta, PI / 4.0f));
}

// Test: Large angle normalization (multiple rotations)
void test_pose2d_large_angle_normalization() {
  Pose2D pose(0.0f, 0.0f, 10.0f * PI);
  pose.normalizeTheta();
  TEST_ASSERT_TRUE(floatEqual(pose.theta, 0.0f, 0.001f));
}

// Test: Very small angles (precision test)
void test_pose2d_small_angles() {
  Pose2D pose(0.0f, 0.0f, 0.0001f);
  pose.normalizeTheta();
  TEST_ASSERT_TRUE(floatEqual(pose.theta, 0.0001f));
}

// Test: Combined operations
void test_pose2d_combined_operations() {
  Pose2D pose(1.0f, 2.0f, 0.0f);
  Pose2D delta(0.5f, 0.5f, PI / 6.0f);

  pose.add(delta);
  pose.rotate(PI / 6.0f);
  pose.normalizeTheta();

  TEST_ASSERT_EQUAL_FLOAT(1.5f, pose.x);
  TEST_ASSERT_EQUAL_FLOAT(2.5f, pose.y);
  TEST_ASSERT_TRUE(floatEqual(pose.theta, PI / 3.0f));
}

int main(int argc, char **argv) {
  UNITY_BEGIN();

  RUN_TEST(test_pose2d_default_constructor);
  RUN_TEST(test_pose2d_parameterized_constructor);
  RUN_TEST(test_pose2d_getters);
  RUN_TEST(test_pose2d_normalize_theta_basic);
  RUN_TEST(test_pose2d_normalize_theta_positive_overflow);
  RUN_TEST(test_pose2d_normalize_theta_negative_overflow);
  RUN_TEST(test_pose2d_normalize_theta_at_pi);
  RUN_TEST(test_pose2d_normalize_theta_at_negative_pi);
  RUN_TEST(test_pose2d_normalize_theta_zero);
  RUN_TEST(test_pose2d_add_basic);
  RUN_TEST(test_pose2d_add_negative);
  RUN_TEST(test_pose2d_add_zeros);
  RUN_TEST(test_pose2d_rotate_basic);
  RUN_TEST(test_pose2d_rotate_with_existing_angle);
  RUN_TEST(test_pose2d_rotate_full_circle);
  RUN_TEST(test_pose2d_rotate_negative);
  RUN_TEST(test_pose2d_large_angle_normalization);
  RUN_TEST(test_pose2d_small_angles);
  RUN_TEST(test_pose2d_combined_operations);

  return UNITY_END();
}
