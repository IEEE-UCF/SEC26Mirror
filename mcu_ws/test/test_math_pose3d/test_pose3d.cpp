/**
 * @file test_pose3d.cpp
 * @brief Unit tests for Pose3D class
 * @date 12/21/2025
 */

#include <unity.h>
#include <Pose3D.h>
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
void test_pose3d_default_constructor() {
  Pose3D pose;
  TEST_ASSERT_EQUAL_FLOAT(0.0f, pose.x);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, pose.y);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, pose.z);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, pose.qx);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, pose.qy);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, pose.qz);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, pose.qw);
}

// Test: Parameterized constructor with position only
void test_pose3d_parameterized_constructor_position() {
  Pose3D pose(1.0f, 2.0f, 3.0f);
  TEST_ASSERT_EQUAL_FLOAT(1.0f, pose.x);
  TEST_ASSERT_EQUAL_FLOAT(2.0f, pose.y);
  TEST_ASSERT_EQUAL_FLOAT(3.0f, pose.z);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, pose.qx);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, pose.qy);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, pose.qz);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, pose.qw);
}

// Test: Parameterized constructor with position and orientation
void test_pose3d_parameterized_constructor_full() {
  Pose3D pose(1.0f, 2.0f, 3.0f, 0.1f, 0.2f, 0.3f, 0.9f);
  TEST_ASSERT_EQUAL_FLOAT(1.0f, pose.x);
  TEST_ASSERT_EQUAL_FLOAT(2.0f, pose.y);
  TEST_ASSERT_EQUAL_FLOAT(3.0f, pose.z);
  TEST_ASSERT_EQUAL_FLOAT(0.1f, pose.qx);
  TEST_ASSERT_EQUAL_FLOAT(0.2f, pose.qy);
  TEST_ASSERT_EQUAL_FLOAT(0.3f, pose.qz);
  TEST_ASSERT_EQUAL_FLOAT(0.9f, pose.qw);
}

// Test: Position getters
void test_pose3d_position_getters() {
  Pose3D pose(5.5f, 6.6f, 7.7f);
  TEST_ASSERT_EQUAL_FLOAT(5.5f, pose.getX());
  TEST_ASSERT_EQUAL_FLOAT(6.6f, pose.getY());
  TEST_ASSERT_EQUAL_FLOAT(7.7f, pose.getZ());
}

// Test: Orientation getters
void test_pose3d_orientation_getters() {
  Pose3D pose(0.0f, 0.0f, 0.0f, 0.1f, 0.2f, 0.3f, 0.9f);
  TEST_ASSERT_EQUAL_FLOAT(0.1f, pose.getQx());
  TEST_ASSERT_EQUAL_FLOAT(0.2f, pose.getQy());
  TEST_ASSERT_EQUAL_FLOAT(0.3f, pose.getQz());
  TEST_ASSERT_EQUAL_FLOAT(0.9f, pose.getQw());
}

// Test: Negative position values
void test_pose3d_negative_position() {
  Pose3D pose(-1.0f, -2.0f, -3.0f);
  TEST_ASSERT_EQUAL_FLOAT(-1.0f, pose.x);
  TEST_ASSERT_EQUAL_FLOAT(-2.0f, pose.y);
  TEST_ASSERT_EQUAL_FLOAT(-3.0f, pose.z);
}

// Test: Negative quaternion values
void test_pose3d_negative_quaternion() {
  Pose3D pose(0.0f, 0.0f, 0.0f, -0.1f, -0.2f, -0.3f, -0.9f);
  TEST_ASSERT_EQUAL_FLOAT(-0.1f, pose.qx);
  TEST_ASSERT_EQUAL_FLOAT(-0.2f, pose.qy);
  TEST_ASSERT_EQUAL_FLOAT(-0.3f, pose.qz);
  TEST_ASSERT_EQUAL_FLOAT(-0.9f, pose.qw);
}

// Test: Identity quaternion (no rotation)
void test_pose3d_identity_quaternion() {
  Pose3D pose(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, pose.qx);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, pose.qy);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, pose.qz);
  TEST_ASSERT_EQUAL_FLOAT(1.0f, pose.qw);
}

// Test: 90 degree rotation around Z axis (approximate quaternion)
void test_pose3d_rotation_z_90() {
  // 90 degrees around Z = qz = sin(45°) ≈ 0.707, qw = cos(45°) ≈ 0.707
  float halfAngle = (M_PI / 4.0f);  // 45 degrees in radians
  float s = sinf(halfAngle);
  float c = cosf(halfAngle);
  Pose3D pose(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, s, c);

  TEST_ASSERT_TRUE(floatEqual(pose.qz, s, 0.001f));
  TEST_ASSERT_TRUE(floatEqual(pose.qw, c, 0.001f));
}

// Test: 90 degree rotation around X axis
void test_pose3d_rotation_x_90() {
  float halfAngle = (M_PI / 4.0f);
  float s = sinf(halfAngle);
  float c = cosf(halfAngle);
  Pose3D pose(0.0f, 0.0f, 0.0f, s, 0.0f, 0.0f, c);

  TEST_ASSERT_TRUE(floatEqual(pose.qx, s, 0.001f));
  TEST_ASSERT_TRUE(floatEqual(pose.qw, c, 0.001f));
}

// Test: 90 degree rotation around Y axis
void test_pose3d_rotation_y_90() {
  float halfAngle = (M_PI / 4.0f);
  float s = sinf(halfAngle);
  float c = cosf(halfAngle);
  Pose3D pose(0.0f, 0.0f, 0.0f, 0.0f, s, 0.0f, c);

  TEST_ASSERT_TRUE(floatEqual(pose.qy, s, 0.001f));
  TEST_ASSERT_TRUE(floatEqual(pose.qw, c, 0.001f));
}

// Test: Very large position values
void test_pose3d_large_position_values() {
  Pose3D pose(1000.0f, 2000.0f, 3000.0f);
  TEST_ASSERT_EQUAL_FLOAT(1000.0f, pose.x);
  TEST_ASSERT_EQUAL_FLOAT(2000.0f, pose.y);
  TEST_ASSERT_EQUAL_FLOAT(3000.0f, pose.z);
}

// Test: Very small position values (precision test)
void test_pose3d_small_position_values() {
  Pose3D pose(0.0001f, 0.0002f, 0.0003f);
  TEST_ASSERT_TRUE(floatEqual(pose.x, 0.0001f));
  TEST_ASSERT_TRUE(floatEqual(pose.y, 0.0002f));
  TEST_ASSERT_TRUE(floatEqual(pose.z, 0.0003f));
}

// Test: Very small quaternion values (precision test)
void test_pose3d_small_quaternion_values() {
  Pose3D pose(0.0f, 0.0f, 0.0f, 0.0001f, 0.0002f, 0.0003f, 0.9999f);
  TEST_ASSERT_TRUE(floatEqual(pose.qx, 0.0001f));
  TEST_ASSERT_TRUE(floatEqual(pose.qy, 0.0002f));
  TEST_ASSERT_TRUE(floatEqual(pose.qz, 0.0003f));
  TEST_ASSERT_TRUE(floatEqual(pose.qw, 0.9999f));
}

// Test: Zero quaternion (edge case - not normalized)
void test_pose3d_zero_quaternion() {
  Pose3D pose(1.0f, 2.0f, 3.0f, 0.0f, 0.0f, 0.0f, 0.0f);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, pose.qx);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, pose.qy);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, pose.qz);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, pose.qw);
}

// Test: Direct field assignment
void test_pose3d_direct_assignment() {
  Pose3D pose;
  pose.x = 10.0f;
  pose.y = 20.0f;
  pose.z = 30.0f;
  pose.qx = 0.5f;
  pose.qy = 0.5f;
  pose.qz = 0.5f;
  pose.qw = 0.5f;

  TEST_ASSERT_EQUAL_FLOAT(10.0f, pose.x);
  TEST_ASSERT_EQUAL_FLOAT(20.0f, pose.y);
  TEST_ASSERT_EQUAL_FLOAT(30.0f, pose.z);
  TEST_ASSERT_EQUAL_FLOAT(0.5f, pose.qx);
  TEST_ASSERT_EQUAL_FLOAT(0.5f, pose.qy);
  TEST_ASSERT_EQUAL_FLOAT(0.5f, pose.qz);
  TEST_ASSERT_EQUAL_FLOAT(0.5f, pose.qw);
}

// Test: Copy constructor behavior (if exists)
void test_pose3d_copy() {
  Pose3D pose1(1.0f, 2.0f, 3.0f, 0.1f, 0.2f, 0.3f, 0.9f);
  Pose3D pose2 = pose1;

  TEST_ASSERT_EQUAL_FLOAT(pose1.x, pose2.x);
  TEST_ASSERT_EQUAL_FLOAT(pose1.y, pose2.y);
  TEST_ASSERT_EQUAL_FLOAT(pose1.z, pose2.z);
  TEST_ASSERT_EQUAL_FLOAT(pose1.qx, pose2.qx);
  TEST_ASSERT_EQUAL_FLOAT(pose1.qy, pose2.qy);
  TEST_ASSERT_EQUAL_FLOAT(pose1.qz, pose2.qz);
  TEST_ASSERT_EQUAL_FLOAT(pose1.qw, pose2.qw);
}

// Test: Mixed positive and negative values
void test_pose3d_mixed_values() {
  Pose3D pose(-1.5f, 2.5f, -3.5f, 0.1f, -0.2f, 0.3f, -0.9f);
  TEST_ASSERT_EQUAL_FLOAT(-1.5f, pose.x);
  TEST_ASSERT_EQUAL_FLOAT(2.5f, pose.y);
  TEST_ASSERT_EQUAL_FLOAT(-3.5f, pose.z);
  TEST_ASSERT_EQUAL_FLOAT(0.1f, pose.qx);
  TEST_ASSERT_EQUAL_FLOAT(-0.2f, pose.qy);
  TEST_ASSERT_EQUAL_FLOAT(0.3f, pose.qz);
  TEST_ASSERT_EQUAL_FLOAT(-0.9f, pose.qw);
}

int main(int argc, char **argv) {
  UNITY_BEGIN();

  RUN_TEST(test_pose3d_default_constructor);
  RUN_TEST(test_pose3d_parameterized_constructor_position);
  RUN_TEST(test_pose3d_parameterized_constructor_full);
  RUN_TEST(test_pose3d_position_getters);
  RUN_TEST(test_pose3d_orientation_getters);
  RUN_TEST(test_pose3d_negative_position);
  RUN_TEST(test_pose3d_negative_quaternion);
  RUN_TEST(test_pose3d_identity_quaternion);
  RUN_TEST(test_pose3d_rotation_z_90);
  RUN_TEST(test_pose3d_rotation_x_90);
  RUN_TEST(test_pose3d_rotation_y_90);
  RUN_TEST(test_pose3d_large_position_values);
  RUN_TEST(test_pose3d_small_position_values);
  RUN_TEST(test_pose3d_small_quaternion_values);
  RUN_TEST(test_pose3d_zero_quaternion);
  RUN_TEST(test_pose3d_direct_assignment);
  RUN_TEST(test_pose3d_copy);
  RUN_TEST(test_pose3d_mixed_values);

  return UNITY_END();
}
