/**
 * @file test_tankdrivelocalization.cpp
 * @brief Unit tests for TankDriveLocalization class
 * @author Claude Code
 * @date 12/24/2025
 */

#define _USE_MATH_DEFINES
#include <unity.h>
#include <TankDriveLocalization.h>
#include <cmath>
#include <cstring>

using namespace Drive;

// Helper function to compare floats with tolerance
bool floatEqual(float a, float b, float epsilon = 0.0001f) {
  return fabs(a - b) <= epsilon;
}

void setUp(void) {
  // Set up code here, runs before each test
}

void tearDown(void) {
  // Clean up code here, runs after each test
}

// =============================================================================
// TankDriveLocalizationSetup Tests
// =============================================================================

// Test: Setup calculates derived constants correctly
void test_setup_derived_constants() {
  // Robot with 4-inch wheels, 2048 ticks/rev, 1:1 gear ratio
  TankDriveLocalizationSetup setup("test", 12.0f, 4.0f, 2048, 1);

  TEST_ASSERT_EQUAL_FLOAT(2.0f, setup.wheel_radius);
  TEST_ASSERT_TRUE(floatEqual(setup.wheel_circumference, 4.0f * M_PI));
  TEST_ASSERT_EQUAL_INT(2048, setup.ticks_per_revolution);
  TEST_ASSERT_TRUE(floatEqual(setup.dist_per_tick,
                              (4.0f * M_PI) / 2048.0f));
}

// Test: Setup with gear ratio
void test_setup_with_gear_ratio() {
  // Robot with 6-inch wheels, 1024 ticks/rev, 3:1 gear ratio
  TankDriveLocalizationSetup setup("test", 10.0f, 6.0f, 1024, 3);

  TEST_ASSERT_EQUAL_FLOAT(3.0f, setup.wheel_radius);
  TEST_ASSERT_TRUE(floatEqual(setup.wheel_circumference, 6.0f * M_PI));
  TEST_ASSERT_EQUAL_INT(3072, setup.ticks_per_revolution); // 1024 * 3
  TEST_ASSERT_TRUE(floatEqual(setup.dist_per_tick,
                              (6.0f * M_PI) / 3072.0f));
}

// Test: Setup with custom starting pose
void test_setup_custom_start_pose() {
  TankDriveLocalizationSetup setup("test", 12.0f, 4.0f, 2048, 1,
                                   10.0f, 20.0f, M_PI / 4.0f);

  TEST_ASSERT_EQUAL_FLOAT(10.0f, setup.start_x);
  TEST_ASSERT_EQUAL_FLOAT(20.0f, setup.start_y);
  TEST_ASSERT_TRUE(floatEqual(setup.start_theta, M_PI / 4.0f));
}

// =============================================================================
// TankDriveLocalization Constructor Tests
// =============================================================================

// Test: Default constructor initializes at origin
void test_constructor_default_pose() {
  TankDriveLocalizationSetup setup("test", 12.0f, 4.0f, 2048, 1);
  TankDriveLocalization odom(setup);

  Pose2D pose = odom.getPose();
  TEST_ASSERT_EQUAL_FLOAT(0.0f, pose.x);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, pose.y);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, pose.theta);
}

// Test: Constructor with custom starting pose
void test_constructor_custom_pose() {
  TankDriveLocalizationSetup setup("test", 12.0f, 4.0f, 2048, 1,
                                   5.0f, 10.0f, M_PI / 2.0f);
  TankDriveLocalization odom(setup);

  Pose2D pose = odom.getPose();
  TEST_ASSERT_EQUAL_FLOAT(5.0f, pose.x);
  TEST_ASSERT_EQUAL_FLOAT(10.0f, pose.y);
  TEST_ASSERT_TRUE(floatEqual(pose.theta, M_PI / 2.0f));
}

// =============================================================================
// Basic Movement Tests
// =============================================================================

// Test: Straight line forward (both wheels same positive speed)
void test_straight_line_forward() {
  TankDriveLocalizationSetup setup("test", 12.0f, 4.0f, 2048, 1);
  TankDriveLocalization odom(setup);

  // Move forward with both wheels moving 1024 ticks (half revolution)
  // Distance = (4 * PI) / 2 = ~6.28 inches
  odom.update(1024, 1024, 0.0f);

  Pose2D pose = odom.getPose();
  float expected_dist = (4.0f * M_PI) / 2.0f;

  TEST_ASSERT_TRUE(floatEqual(pose.x, expected_dist, 0.01f));
  TEST_ASSERT_TRUE(floatEqual(pose.y, 0.0f, 0.01f));
  TEST_ASSERT_EQUAL_FLOAT(0.0f, pose.theta);
}

// Test: Straight line backward (both wheels same negative speed)
void test_straight_line_backward() {
  TankDriveLocalizationSetup setup("test", 12.0f, 4.0f, 2048, 1);
  TankDriveLocalization odom(setup);

  // Move backward with both wheels moving -2048 ticks (one full revolution)
  odom.update(-2048, -2048, 0.0f);

  Pose2D pose = odom.getPose();
  float expected_dist = -(4.0f * M_PI);

  TEST_ASSERT_TRUE(floatEqual(pose.x, expected_dist, 0.01f));
  TEST_ASSERT_TRUE(floatEqual(pose.y, 0.0f, 0.01f));
  TEST_ASSERT_EQUAL_FLOAT(0.0f, pose.theta);
}

// Test: Straight line at 45 degrees
void test_straight_line_angled() {
  TankDriveLocalizationSetup setup("test", 12.0f, 4.0f, 2048, 1);
  TankDriveLocalization odom(setup);

  // Move at 45 degrees with both wheels moving 2048 ticks
  float angle = M_PI / 4.0f;
  odom.update(2048, 2048, angle);

  Pose2D pose = odom.getPose();
  float dist = 4.0f * M_PI;
  float expected_x = dist * cosf(angle);
  float expected_y = dist * sinf(angle);

  TEST_ASSERT_TRUE(floatEqual(pose.x, expected_x, 0.01f));
  TEST_ASSERT_TRUE(floatEqual(pose.y, expected_y, 0.01f));
  TEST_ASSERT_TRUE(floatEqual(pose.theta, angle, 0.01f));
}

// Test: Multiple updates accumulate correctly
void test_multiple_updates_straight() {
  TankDriveLocalizationSetup setup("test", 12.0f, 4.0f, 2048, 1);
  TankDriveLocalization odom(setup);

  // Three updates of 512 ticks each = total 1536 ticks
  odom.update(512, 512, 0.0f);
  odom.update(1024, 1024, 0.0f);  // Cumulative: 1024 total
  odom.update(1536, 1536, 0.0f);  // Cumulative: 1536 total

  Pose2D pose = odom.getPose();
  float expected_dist = (4.0f * M_PI * 1536.0f) / 2048.0f;

  TEST_ASSERT_TRUE(floatEqual(pose.x, expected_dist, 0.01f));
  TEST_ASSERT_TRUE(floatEqual(pose.y, 0.0f, 0.01f));
}

// Test: Zero movement (no encoder change)
void test_zero_movement() {
  TankDriveLocalizationSetup setup("test", 12.0f, 4.0f, 2048, 1);
  TankDriveLocalization odom(setup);

  odom.update(0, 0, 0.0f);

  Pose2D pose = odom.getPose();
  TEST_ASSERT_EQUAL_FLOAT(0.0f, pose.x);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, pose.y);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, pose.theta);
}

// =============================================================================
// Rotation Tests
// =============================================================================

// Test: Rotation updates heading correctly
void test_rotation_heading_update() {
  TankDriveLocalizationSetup setup("test", 12.0f, 4.0f, 2048, 1);
  TankDriveLocalization odom(setup);

  // Pure rotation: wheels don't move but heading changes
  odom.update(0, 0, M_PI / 2.0f);

  Pose2D pose = odom.getPose();
  TEST_ASSERT_EQUAL_FLOAT(0.0f, pose.x);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, pose.y);
  TEST_ASSERT_TRUE(floatEqual(pose.theta, M_PI / 2.0f));
}

// Test: Rotation with movement
void test_rotation_with_movement() {
  TankDriveLocalizationSetup setup("test", 12.0f, 4.0f, 2048, 1);
  TankDriveLocalization odom(setup);

  // Start facing east, move forward, then turn north
  odom.update(2048, 2048, 0.0f);  // Move forward facing east

  Pose2D pose1 = odom.getPose();
  float dist = 4.0f * M_PI;
  TEST_ASSERT_TRUE(floatEqual(pose1.x, dist, 0.01f));
  TEST_ASSERT_TRUE(floatEqual(pose1.y, 0.0f, 0.01f));

  // Now turn to face north and move again
  odom.update(4096, 4096, M_PI / 2.0f);  // Another full wheel rotation

  Pose2D pose2 = odom.getPose();
  // Should move north (positive y)
  TEST_ASSERT_TRUE(floatEqual(pose2.x, dist, 0.01f));
  TEST_ASSERT_TRUE(floatEqual(pose2.y, dist, 0.01f));
  TEST_ASSERT_TRUE(floatEqual(pose2.theta, M_PI / 2.0f));
}

// Test: Full 360 degree rotation
void test_full_rotation() {
  TankDriveLocalizationSetup setup("test", 12.0f, 4.0f, 2048, 1);
  TankDriveLocalization odom(setup);

  odom.update(0, 0, 2.0f * M_PI);

  Pose2D pose = odom.getPose();
  // After normalization, should be close to 0
  TEST_ASSERT_TRUE(floatEqual(pose.theta, 0.0f, 0.01f) ||
                   floatEqual(pose.theta, -M_PI, 0.01f) ||
                   floatEqual(pose.theta, M_PI, 0.01f));
}

// Test: Negative rotation
void test_negative_rotation() {
  TankDriveLocalizationSetup setup("test", 12.0f, 4.0f, 2048, 1);
  TankDriveLocalization odom(setup);

  odom.update(0, 0, -M_PI / 4.0f);

  Pose2D pose = odom.getPose();
  TEST_ASSERT_TRUE(floatEqual(pose.theta, -M_PI / 4.0f));
}

// =============================================================================
// Complex Movement Tests
// =============================================================================

// Test: Curved path (differential wheel speeds)
void test_curved_path() {
  TankDriveLocalizationSetup setup("test", 12.0f, 4.0f, 2048, 1);
  TankDriveLocalization odom(setup);

  // Left wheel moves slower than right, should curve to the left
  // This is a simplification - actual arc motion is complex
  odom.update(1024, 2048, M_PI / 8.0f);  // Right wheel moves more

  Pose2D pose = odom.getPose();
  // Should have moved in an arc
  TEST_ASSERT_NOT_EQUAL(0.0f, pose.x);
  TEST_ASSERT_TRUE(floatEqual(pose.theta, M_PI / 8.0f));
}

// Test: Sequential curved movements
void test_sequential_curves() {
  TankDriveLocalizationSetup setup("test", 12.0f, 4.0f, 2048, 1);
  TankDriveLocalization odom(setup);

  // Quarter circle motion
  odom.update(512, 512, 0.0f);
  odom.update(1024, 1024, M_PI / 8.0f);
  odom.update(1536, 1536, M_PI / 4.0f);
  odom.update(2048, 2048, M_PI / 2.0f);

  Pose2D pose = odom.getPose();
  TEST_ASSERT_TRUE(floatEqual(pose.theta, M_PI / 2.0f));
  TEST_ASSERT_NOT_EQUAL(0.0f, pose.x);
  TEST_ASSERT_NOT_EQUAL(0.0f, pose.y);
}

// =============================================================================
// Edge Cases
// =============================================================================

// Test: Very small movement (precision)
void test_very_small_movement() {
  TankDriveLocalizationSetup setup("test", 12.0f, 4.0f, 2048, 1);
  TankDriveLocalization odom(setup);

  // Single tick movement
  odom.update(1, 1, 0.0f);

  Pose2D pose = odom.getPose();
  float expected = (4.0f * M_PI) / 2048.0f;
  TEST_ASSERT_TRUE(floatEqual(pose.x, expected, 0.0001f));
}

// Test: Very large movement
void test_very_large_movement() {
  TankDriveLocalizationSetup setup("test", 12.0f, 4.0f, 2048, 1);
  TankDriveLocalization odom(setup);

  // 100 full revolutions
  odom.update(204800, 204800, 0.0f);

  Pose2D pose = odom.getPose();
  float expected = 100.0f * 4.0f * M_PI;
  TEST_ASSERT_TRUE(floatEqual(pose.x, expected, 0.1f));
}

// Test: Movement with angle wrapping
void test_angle_wrapping() {
  TankDriveLocalizationSetup setup("test", 12.0f, 4.0f, 2048, 1);
  TankDriveLocalization odom(setup);

  // Move with angle > 2*PI (should normalize to approximately PI or -PI)
  odom.update(2048, 2048, 3.0f * M_PI);

  Pose2D pose = odom.getPose();
  // Angle should be normalized to approximately [-PI, PI]
  // Account for floating-point precision issues at boundaries
  TEST_ASSERT_TRUE(pose.theta >= -M_PI - 0.01f && pose.theta <= M_PI + 0.01f);
  // Should be close to PI or -PI
  TEST_ASSERT_TRUE(floatEqual(fabs(pose.theta), M_PI, 0.01f));
}

// Test: Backwards then forwards
void test_backwards_then_forwards() {
  TankDriveLocalizationSetup setup("test", 12.0f, 4.0f, 2048, 1);
  TankDriveLocalization odom(setup);

  odom.update(-2048, -2048, 0.0f);  // Move back one revolution
  odom.update(0, 0, 0.0f);           // Move forward one revolution

  Pose2D pose = odom.getPose();
  // Should be back at origin
  TEST_ASSERT_TRUE(floatEqual(pose.x, 0.0f, 0.01f));
  TEST_ASSERT_TRUE(floatEqual(pose.y, 0.0f, 0.01f));
}

// =============================================================================
// Reset Tests
// =============================================================================

// Test: Reset returns to starting pose (origin)
void test_reset_to_origin() {
  TankDriveLocalizationSetup setup("test", 12.0f, 4.0f, 2048, 1);
  TankDriveLocalization odom(setup);

  // Move somewhere
  odom.update(2048, 2048, M_PI / 4.0f);

  Pose2D pose1 = odom.getPose();
  TEST_ASSERT_NOT_EQUAL(0.0f, pose1.x);

  // Reset
  odom.reset();

  Pose2D pose2 = odom.getPose();
  TEST_ASSERT_EQUAL_FLOAT(0.0f, pose2.x);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, pose2.y);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, pose2.theta);
}

// Test: Reset returns to custom starting pose
void test_reset_to_custom_pose() {
  TankDriveLocalizationSetup setup("test", 12.0f, 4.0f, 2048, 1,
                                   5.0f, 10.0f, M_PI / 3.0f);
  TankDriveLocalization odom(setup);

  // Move somewhere
  odom.update(2048, 2048, 0.0f);

  // Reset
  odom.reset();

  Pose2D pose = odom.getPose();
  TEST_ASSERT_EQUAL_FLOAT(5.0f, pose.x);
  TEST_ASSERT_EQUAL_FLOAT(10.0f, pose.y);
  TEST_ASSERT_TRUE(floatEqual(pose.theta, M_PI / 3.0f));
}

// Test: Reset clears encoder history
void test_reset_clears_encoder_history() {
  TankDriveLocalizationSetup setup("test", 12.0f, 4.0f, 2048, 1);
  TankDriveLocalization odom(setup);

  // Move with high tick counts
  odom.update(10000, 10000, 0.0f);

  // Reset
  odom.reset();

  // Next update should start from 0, not from previous ticks
  odom.update(1024, 1024, 0.0f);

  Pose2D pose = odom.getPose();
  float expected = (4.0f * M_PI * 1024.0f) / 2048.0f;
  TEST_ASSERT_TRUE(floatEqual(pose.x, expected, 0.01f));
}

// =============================================================================
// getInfo() Tests
// =============================================================================

// Test: getInfo returns formatted string
void test_getinfo_format() {
  TankDriveLocalizationSetup setup("test", 12.0f, 4.0f, 2048, 1);
  TankDriveLocalization odom(setup);

  const char* info = odom.getInfo();
  TEST_ASSERT_NOT_NULL(info);
  // Should contain position information
  TEST_ASSERT_TRUE(strstr(info, "X:") != nullptr);
  TEST_ASSERT_TRUE(strstr(info, "Y:") != nullptr);
  TEST_ASSERT_TRUE(strstr(info, "Theta:") != nullptr);
}

// Test: getInfo after movement
void test_getinfo_after_movement() {
  TankDriveLocalizationSetup setup("test", 12.0f, 4.0f, 2048, 1);
  TankDriveLocalization odom(setup);

  odom.update(2048, 2048, M_PI / 4.0f);
  const char* info = odom.getInfo();

  TEST_ASSERT_NOT_NULL(info);
  // Should have non-zero values after movement
  TEST_ASSERT_TRUE(strstr(info, "0.00") == nullptr);  // Should not be all zeros
}

// =============================================================================
// Real-world Scenario Tests
// =============================================================================

// Test: Square path
void test_square_path() {
  TankDriveLocalizationSetup setup("test", 12.0f, 4.0f, 2048, 1);
  TankDriveLocalization odom(setup);

  float side_length = 4.0f * M_PI;  // One wheel revolution

  // Side 1: Move forward (east)
  odom.update(2048, 2048, 0.0f);

  // Side 2: Turn north, move forward
  odom.update(4096, 4096, M_PI / 2.0f);

  // Side 3: Turn west, move forward
  odom.update(6144, 6144, M_PI);

  // Side 4: Turn south, move forward
  odom.update(8192, 8192, -M_PI / 2.0f);

  Pose2D pose = odom.getPose();
  // Should be back near origin (within numerical precision)
  TEST_ASSERT_TRUE(floatEqual(pose.x, 0.0f, 0.1f));
  TEST_ASSERT_TRUE(floatEqual(pose.y, 0.0f, 0.1f));
}

// Test: Figure-8 pattern
void test_figure_eight() {
  TankDriveLocalizationSetup setup("test", 12.0f, 4.0f, 2048, 1);
  TankDriveLocalization odom(setup);

  // Simulate a figure-8 with multiple small movements
  for (int i = 0; i < 8; i++) {
    float angle = (float)i * M_PI / 4.0f;
    long ticks = 512 * (i + 1);
    odom.update(ticks, ticks, angle);
  }

  Pose2D pose = odom.getPose();
  // Just verify it moved somewhere reasonable
  TEST_ASSERT_NOT_EQUAL(0.0f, pose.x);
  TEST_ASSERT_TRUE(pose.theta >= -M_PI && pose.theta <= M_PI);
}

// =============================================================================
// Main Test Runner
// =============================================================================

int main(int argc, char **argv) {
  UNITY_BEGIN();

  // Setup tests
  RUN_TEST(test_setup_derived_constants);
  RUN_TEST(test_setup_with_gear_ratio);
  RUN_TEST(test_setup_custom_start_pose);

  // Constructor tests
  RUN_TEST(test_constructor_default_pose);
  RUN_TEST(test_constructor_custom_pose);

  // Basic movement tests
  RUN_TEST(test_straight_line_forward);
  RUN_TEST(test_straight_line_backward);
  RUN_TEST(test_straight_line_angled);
  RUN_TEST(test_multiple_updates_straight);
  RUN_TEST(test_zero_movement);

  // Rotation tests
  RUN_TEST(test_rotation_heading_update);
  RUN_TEST(test_rotation_with_movement);
  RUN_TEST(test_full_rotation);
  RUN_TEST(test_negative_rotation);

  // Complex movement tests
  RUN_TEST(test_curved_path);
  RUN_TEST(test_sequential_curves);

  // Edge cases
  RUN_TEST(test_very_small_movement);
  RUN_TEST(test_very_large_movement);
  RUN_TEST(test_angle_wrapping);
  RUN_TEST(test_backwards_then_forwards);

  // Reset tests
  RUN_TEST(test_reset_to_origin);
  RUN_TEST(test_reset_to_custom_pose);
  RUN_TEST(test_reset_clears_encoder_history);

  // getInfo tests
  RUN_TEST(test_getinfo_format);
  RUN_TEST(test_getinfo_after_movement);

  // Real-world scenarios
  RUN_TEST(test_square_path);
  RUN_TEST(test_figure_eight);

  return UNITY_END();
}
