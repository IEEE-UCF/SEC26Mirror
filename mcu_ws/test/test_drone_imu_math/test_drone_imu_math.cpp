/**
 * @file test_drone_imu_math.cpp
 * @brief Native unit tests for drone IMU math: quaternion-to-Euler and
 *        world-frame acceleration rotation.
 * @date 2026-03-07
 *
 * Tests the conversion formulas from GyroSubsystem:
 * - Quaternion (qr, qi, qj, qk) → Euler angles (roll, pitch, yaw) in degrees
 * - Body-frame accel (ax, ay) → world-frame via yaw rotation
 */

#define _USE_MATH_DEFINES
#include <cmath>
#include <unity.h>

static constexpr float RAD2DEG = 57.29577951f;
static constexpr float DEG2RAD = 0.01745329252f;

// ── Quaternion → Euler (from GyroSubsystem::update) ──

struct EulerAngles {
  float roll, pitch, yaw;  // degrees
};

static EulerAngles quatToEuler(float qr, float qi, float qj, float qk) {
  EulerAngles e;
  e.roll = atan2f(2.0f * (qr * qi + qj * qk),
                  1.0f - 2.0f * (qi * qi + qj * qj)) *
           RAD2DEG;

  float sin_pitch = 2.0f * (qr * qj - qk * qi);
  if (sin_pitch > 0.999999f) sin_pitch = 0.999999f;
  if (sin_pitch < -0.999999f) sin_pitch = -0.999999f;
  e.pitch = asinf(sin_pitch) * RAD2DEG;

  e.yaw = atan2f(2.0f * (qr * qk + qi * qj),
                 1.0f - 2.0f * (qj * qj + qk * qk)) *
          RAD2DEG;
  return e;
}

// ── World-frame rotation (from GyroSubsystem::getAccelWorld) ──

static void rotateToWorld(float accel_x, float accel_y, float yaw_rad,
                          float& ax_world, float& ay_world) {
  float cy = cosf(yaw_rad);
  float sy = sinf(yaw_rad);
  ax_world = accel_x * cy - accel_y * sy;
  ay_world = accel_x * sy + accel_y * cy;
}

void setUp(void) {}
void tearDown(void) {}

// ════════════════════════════════════════════════════════════════
//  QUATERNION → EULER TESTS
// ════════════════════════════════════════════════════════════════

void test_identity_quaternion_zero_euler() {
  // q = (1, 0, 0, 0) → no rotation
  auto e = quatToEuler(1.0f, 0.0f, 0.0f, 0.0f);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, e.roll);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, e.pitch);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, e.yaw);
}

void test_pure_roll_90() {
  // 90 deg roll about X: q = (cos45, sin45, 0, 0) = (0.7071, 0.7071, 0, 0)
  float c = cosf(M_PI / 4.0f), s = sinf(M_PI / 4.0f);
  auto e = quatToEuler(c, s, 0.0f, 0.0f);
  TEST_ASSERT_FLOAT_WITHIN(0.5f, 90.0f, e.roll);
  TEST_ASSERT_FLOAT_WITHIN(0.5f, 0.0f, e.pitch);
  TEST_ASSERT_FLOAT_WITHIN(0.5f, 0.0f, e.yaw);
}

void test_pure_pitch_45() {
  // 45 deg pitch about Y: q = (cos22.5, 0, sin22.5, 0)
  float angle = 45.0f * DEG2RAD;
  float c = cosf(angle / 2.0f), s = sinf(angle / 2.0f);
  auto e = quatToEuler(c, 0.0f, s, 0.0f);
  TEST_ASSERT_FLOAT_WITHIN(0.5f, 0.0f, e.roll);
  TEST_ASSERT_FLOAT_WITHIN(0.5f, 45.0f, e.pitch);
  TEST_ASSERT_FLOAT_WITHIN(0.5f, 0.0f, e.yaw);
}

void test_pure_yaw_90() {
  // 90 deg yaw about Z: q = (cos45, 0, 0, sin45)
  float c = cosf(M_PI / 4.0f), s = sinf(M_PI / 4.0f);
  auto e = quatToEuler(c, 0.0f, 0.0f, s);
  TEST_ASSERT_FLOAT_WITHIN(0.5f, 0.0f, e.roll);
  TEST_ASSERT_FLOAT_WITHIN(0.5f, 0.0f, e.pitch);
  TEST_ASSERT_FLOAT_WITHIN(0.5f, 90.0f, e.yaw);
}

void test_negative_roll() {
  float angle = -30.0f * DEG2RAD;
  float c = cosf(angle / 2.0f), s = sinf(angle / 2.0f);
  auto e = quatToEuler(c, s, 0.0f, 0.0f);
  TEST_ASSERT_FLOAT_WITHIN(0.5f, -30.0f, e.roll);
}

void test_negative_yaw() {
  float angle = -45.0f * DEG2RAD;
  float c = cosf(angle / 2.0f), s = sinf(angle / 2.0f);
  auto e = quatToEuler(c, 0.0f, 0.0f, s);
  TEST_ASSERT_FLOAT_WITHIN(0.5f, -45.0f, e.yaw);
}

void test_pitch_near_90_gimbal_lock() {
  // Near gimbal lock: pitch → 89 degrees
  float angle = 89.0f * DEG2RAD;
  float c = cosf(angle / 2.0f), s = sinf(angle / 2.0f);
  auto e = quatToEuler(c, 0.0f, s, 0.0f);
  TEST_ASSERT_FLOAT_WITHIN(1.0f, 89.0f, e.pitch);
}

void test_normalized_quaternion() {
  // Verify that the formulas work with a properly normalized quaternion
  float qr = 0.5f, qi = 0.5f, qj = 0.5f, qk = 0.5f;
  float norm = sqrtf(qr * qr + qi * qi + qj * qj + qk * qk);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 1.0f, norm);
  auto e = quatToEuler(qr, qi, qj, qk);
  // This quaternion = 120 deg rotation about (1,1,1) axis
  // Should produce valid Euler angles (no NaN)
  TEST_ASSERT_FALSE(std::isnan(e.roll));
  TEST_ASSERT_FALSE(std::isnan(e.pitch));
  TEST_ASSERT_FALSE(std::isnan(e.yaw));
}

// ════════════════════════════════════════════════════════════════
//  WORLD-FRAME ROTATION TESTS
// ════════════════════════════════════════════════════════════════

void test_zero_yaw_no_rotation() {
  float ax_w, ay_w;
  rotateToWorld(1.0f, 0.0f, 0.0f, ax_w, ay_w);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 1.0f, ax_w);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.0f, ay_w);
}

void test_90_yaw_rotates_x_to_y() {
  float ax_w, ay_w;
  // Yaw = 90 deg: body X becomes world Y
  rotateToWorld(1.0f, 0.0f, (float)(M_PI / 2.0), ax_w, ay_w);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, ax_w);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 1.0f, ay_w);
}

void test_180_yaw_negates() {
  float ax_w, ay_w;
  rotateToWorld(1.0f, 0.0f, (float)M_PI, ax_w, ay_w);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, -1.0f, ax_w);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, ay_w);
}

void test_45_yaw_equal_components() {
  float ax_w, ay_w;
  rotateToWorld(1.0f, 0.0f, (float)(M_PI / 4.0), ax_w, ay_w);
  float expected = 1.0f / sqrtf(2.0f);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, expected, ax_w);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, expected, ay_w);
}

void test_both_body_axes() {
  float ax_w, ay_w;
  // Body accel (1, 1) with yaw=0 → world (1, 1)
  rotateToWorld(1.0f, 1.0f, 0.0f, ax_w, ay_w);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 1.0f, ax_w);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 1.0f, ay_w);
}

void test_both_body_axes_90_yaw() {
  float ax_w, ay_w;
  // Body accel (1, 1) with yaw=90: ax_w = 1*cos90 - 1*sin90 = -1
  //                                 ay_w = 1*sin90 + 1*cos90 = 1
  rotateToWorld(1.0f, 1.0f, (float)(M_PI / 2.0), ax_w, ay_w);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, -1.0f, ax_w);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 1.0f, ay_w);
}

void test_rotation_preserves_magnitude() {
  float ax = 3.0f, ay = 4.0f;
  float yaw = 1.234f;
  float ax_w, ay_w;
  rotateToWorld(ax, ay, yaw, ax_w, ay_w);
  float mag_body = sqrtf(ax * ax + ay * ay);
  float mag_world = sqrtf(ax_w * ax_w + ay_w * ay_w);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, mag_body, mag_world);
}

void test_zero_accel_stays_zero() {
  float ax_w, ay_w;
  rotateToWorld(0.0f, 0.0f, 1.5f, ax_w, ay_w);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, ax_w);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, ay_w);
}

void test_full_rotation_identity() {
  float ax_w, ay_w;
  // 360 deg rotation → back to original
  rotateToWorld(2.5f, -1.3f, (float)(2.0 * M_PI), ax_w, ay_w);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 2.5f, ax_w);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, -1.3f, ay_w);
}

int main(int argc, char** argv) {
  UNITY_BEGIN();

  // Quaternion → Euler
  RUN_TEST(test_identity_quaternion_zero_euler);
  RUN_TEST(test_pure_roll_90);
  RUN_TEST(test_pure_pitch_45);
  RUN_TEST(test_pure_yaw_90);
  RUN_TEST(test_negative_roll);
  RUN_TEST(test_negative_yaw);
  RUN_TEST(test_pitch_near_90_gimbal_lock);
  RUN_TEST(test_normalized_quaternion);

  // World-frame rotation
  RUN_TEST(test_zero_yaw_no_rotation);
  RUN_TEST(test_90_yaw_rotates_x_to_y);
  RUN_TEST(test_180_yaw_negates);
  RUN_TEST(test_45_yaw_equal_components);
  RUN_TEST(test_both_body_axes);
  RUN_TEST(test_both_body_axes_90_yaw);
  RUN_TEST(test_rotation_preserves_magnitude);
  RUN_TEST(test_zero_accel_stays_zero);
  RUN_TEST(test_full_rotation_identity);

  return UNITY_END();
}
