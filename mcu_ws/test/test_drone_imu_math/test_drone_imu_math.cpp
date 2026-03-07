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

// Helper: create quaternion from axis-angle
static void axisAngleToQuat(float ax, float ay, float az, float angle_rad,
                            float& qr, float& qi, float& qj, float& qk) {
  float half = angle_rad / 2.0f;
  float s = sinf(half);
  qr = cosf(half);
  qi = ax * s;
  qj = ay * s;
  qk = az * s;
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
//  QUATERNION → EULER: IDENTITY & BASIC ROTATIONS
// ════════════════════════════════════════════════════════════════

void test_identity_quaternion_zero_euler() {
  auto e = quatToEuler(1.0f, 0.0f, 0.0f, 0.0f);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, e.roll);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, e.pitch);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, e.yaw);
}

void test_pure_roll_90() {
  float c = cosf(M_PI / 4.0f), s = sinf(M_PI / 4.0f);
  auto e = quatToEuler(c, s, 0.0f, 0.0f);
  TEST_ASSERT_FLOAT_WITHIN(0.5f, 90.0f, e.roll);
  TEST_ASSERT_FLOAT_WITHIN(0.5f, 0.0f, e.pitch);
  TEST_ASSERT_FLOAT_WITHIN(0.5f, 0.0f, e.yaw);
}

void test_pure_roll_180() {
  float c = cosf(M_PI / 2.0f), s = sinf(M_PI / 2.0f);
  auto e = quatToEuler(c, s, 0.0f, 0.0f);
  TEST_ASSERT_FLOAT_WITHIN(1.0f, 180.0f, fabsf(e.roll));
}

void test_pure_roll_45() {
  float angle = 45.0f * DEG2RAD;
  float c = cosf(angle / 2.0f), s = sinf(angle / 2.0f);
  auto e = quatToEuler(c, s, 0.0f, 0.0f);
  TEST_ASSERT_FLOAT_WITHIN(0.5f, 45.0f, e.roll);
}

void test_pure_pitch_45() {
  float angle = 45.0f * DEG2RAD;
  float c = cosf(angle / 2.0f), s = sinf(angle / 2.0f);
  auto e = quatToEuler(c, 0.0f, s, 0.0f);
  TEST_ASSERT_FLOAT_WITHIN(0.5f, 0.0f, e.roll);
  TEST_ASSERT_FLOAT_WITHIN(0.5f, 45.0f, e.pitch);
  TEST_ASSERT_FLOAT_WITHIN(0.5f, 0.0f, e.yaw);
}

void test_pure_pitch_30() {
  float angle = 30.0f * DEG2RAD;
  float c = cosf(angle / 2.0f), s = sinf(angle / 2.0f);
  auto e = quatToEuler(c, 0.0f, s, 0.0f);
  TEST_ASSERT_FLOAT_WITHIN(0.5f, 30.0f, e.pitch);
}

void test_pure_yaw_90() {
  float c = cosf(M_PI / 4.0f), s = sinf(M_PI / 4.0f);
  auto e = quatToEuler(c, 0.0f, 0.0f, s);
  TEST_ASSERT_FLOAT_WITHIN(0.5f, 0.0f, e.roll);
  TEST_ASSERT_FLOAT_WITHIN(0.5f, 0.0f, e.pitch);
  TEST_ASSERT_FLOAT_WITHIN(0.5f, 90.0f, e.yaw);
}

void test_pure_yaw_180() {
  float c = cosf(M_PI / 2.0f), s = sinf(M_PI / 2.0f);
  auto e = quatToEuler(c, 0.0f, 0.0f, s);
  TEST_ASSERT_FLOAT_WITHIN(1.0f, 180.0f, fabsf(e.yaw));
}

void test_pure_yaw_45() {
  float angle = 45.0f * DEG2RAD;
  float c = cosf(angle / 2.0f), s = sinf(angle / 2.0f);
  auto e = quatToEuler(c, 0.0f, 0.0f, s);
  TEST_ASSERT_FLOAT_WITHIN(0.5f, 45.0f, e.yaw);
}

void test_pure_yaw_270() {
  float angle = 270.0f * DEG2RAD;
  float c = cosf(angle / 2.0f), s = sinf(angle / 2.0f);
  auto e = quatToEuler(c, 0.0f, 0.0f, s);
  // 270 deg yaw = -90 deg yaw
  TEST_ASSERT_FLOAT_WITHIN(1.0f, -90.0f, e.yaw);
}

// ════════════════════════════════════════════════════════════════
//  QUATERNION → EULER: NEGATIVE ANGLES
// ════════════════════════════════════════════════════════════════

void test_negative_roll_30() {
  float angle = -30.0f * DEG2RAD;
  float c = cosf(angle / 2.0f), s = sinf(angle / 2.0f);
  auto e = quatToEuler(c, s, 0.0f, 0.0f);
  TEST_ASSERT_FLOAT_WITHIN(0.5f, -30.0f, e.roll);
}

void test_negative_roll_90() {
  float c = cosf(-M_PI / 4.0f), s = sinf(-M_PI / 4.0f);
  auto e = quatToEuler(c, s, 0.0f, 0.0f);
  TEST_ASSERT_FLOAT_WITHIN(0.5f, -90.0f, e.roll);
}

void test_negative_pitch_45() {
  float angle = -45.0f * DEG2RAD;
  float c = cosf(angle / 2.0f), s = sinf(angle / 2.0f);
  auto e = quatToEuler(c, 0.0f, s, 0.0f);
  TEST_ASSERT_FLOAT_WITHIN(0.5f, -45.0f, e.pitch);
}

void test_negative_yaw_45() {
  float angle = -45.0f * DEG2RAD;
  float c = cosf(angle / 2.0f), s = sinf(angle / 2.0f);
  auto e = quatToEuler(c, 0.0f, 0.0f, s);
  TEST_ASSERT_FLOAT_WITHIN(0.5f, -45.0f, e.yaw);
}

void test_negative_yaw_90() {
  float c = cosf(-M_PI / 4.0f), s = sinf(-M_PI / 4.0f);
  auto e = quatToEuler(c, 0.0f, 0.0f, s);
  TEST_ASSERT_FLOAT_WITHIN(0.5f, -90.0f, e.yaw);
}

// ════════════════════════════════════════════════════════════════
//  QUATERNION → EULER: SMALL ANGLES
// ════════════════════════════════════════════════════════════════

void test_small_roll_1_deg() {
  float angle = 1.0f * DEG2RAD;
  float c = cosf(angle / 2.0f), s = sinf(angle / 2.0f);
  auto e = quatToEuler(c, s, 0.0f, 0.0f);
  TEST_ASSERT_FLOAT_WITHIN(0.1f, 1.0f, e.roll);
}

void test_small_pitch_1_deg() {
  float angle = 1.0f * DEG2RAD;
  float c = cosf(angle / 2.0f), s = sinf(angle / 2.0f);
  auto e = quatToEuler(c, 0.0f, s, 0.0f);
  TEST_ASSERT_FLOAT_WITHIN(0.1f, 1.0f, e.pitch);
}

void test_small_yaw_1_deg() {
  float angle = 1.0f * DEG2RAD;
  float c = cosf(angle / 2.0f), s = sinf(angle / 2.0f);
  auto e = quatToEuler(c, 0.0f, 0.0f, s);
  TEST_ASSERT_FLOAT_WITHIN(0.1f, 1.0f, e.yaw);
}

void test_tiny_roll_0_1_deg() {
  float angle = 0.1f * DEG2RAD;
  float c = cosf(angle / 2.0f), s = sinf(angle / 2.0f);
  auto e = quatToEuler(c, s, 0.0f, 0.0f);
  TEST_ASSERT_FLOAT_WITHIN(0.05f, 0.1f, e.roll);
}

// ════════════════════════════════════════════════════════════════
//  QUATERNION → EULER: GIMBAL LOCK & EDGE CASES
// ════════════════════════════════════════════════════════════════

void test_pitch_near_90_gimbal_lock() {
  float angle = 89.0f * DEG2RAD;
  float c = cosf(angle / 2.0f), s = sinf(angle / 2.0f);
  auto e = quatToEuler(c, 0.0f, s, 0.0f);
  TEST_ASSERT_FLOAT_WITHIN(1.0f, 89.0f, e.pitch);
}

void test_pitch_near_neg90_gimbal_lock() {
  float angle = -89.0f * DEG2RAD;
  float c = cosf(angle / 2.0f), s = sinf(angle / 2.0f);
  auto e = quatToEuler(c, 0.0f, s, 0.0f);
  TEST_ASSERT_FLOAT_WITHIN(1.0f, -89.0f, e.pitch);
}

void test_pitch_exactly_90_clamped() {
  // sin_pitch exactly 1.0 → clamped to 0.999999 → ~89.9 deg
  float angle = 90.0f * DEG2RAD;
  float c = cosf(angle / 2.0f), s = sinf(angle / 2.0f);
  auto e = quatToEuler(c, 0.0f, s, 0.0f);
  TEST_ASSERT_TRUE(e.pitch > 88.0f && e.pitch <= 90.0f);
  TEST_ASSERT_FALSE(std::isnan(e.pitch));
}

void test_normalized_quaternion_120_about_111() {
  float qr = 0.5f, qi = 0.5f, qj = 0.5f, qk = 0.5f;
  float norm = sqrtf(qr * qr + qi * qi + qj * qj + qk * qk);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 1.0f, norm);
  auto e = quatToEuler(qr, qi, qj, qk);
  TEST_ASSERT_FALSE(std::isnan(e.roll));
  TEST_ASSERT_FALSE(std::isnan(e.pitch));
  TEST_ASSERT_FALSE(std::isnan(e.yaw));
}

void test_negative_quat_same_rotation() {
  // q and -q represent the same rotation
  float angle = 60.0f * DEG2RAD;
  float c = cosf(angle / 2.0f), s = sinf(angle / 2.0f);
  auto e1 = quatToEuler(c, s, 0.0f, 0.0f);
  auto e2 = quatToEuler(-c, -s, 0.0f, 0.0f);
  TEST_ASSERT_FLOAT_WITHIN(0.5f, e1.roll, e2.roll);
  TEST_ASSERT_FLOAT_WITHIN(0.5f, e1.pitch, e2.pitch);
  TEST_ASSERT_FLOAT_WITHIN(0.5f, e1.yaw, e2.yaw);
}

void test_no_nan_for_zero_quaternion_components() {
  // All-zero quaternion is degenerate but should not produce NaN
  auto e = quatToEuler(0.0f, 0.0f, 0.0f, 0.0f);
  // atan2(0,1)=0, asin(0)=0 → all zeros expected
  TEST_ASSERT_FALSE(std::isnan(e.roll));
  TEST_ASSERT_FALSE(std::isnan(e.pitch));
  TEST_ASSERT_FALSE(std::isnan(e.yaw));
}

// ════════════════════════════════════════════════════════════════
//  QUATERNION → EULER: COMBINED ROTATIONS
// ════════════════════════════════════════════════════════════════

void test_roll_and_yaw_combined() {
  // 45 deg roll then 45 deg yaw (sequential, not commutative)
  // q_roll = (cos22.5, sin22.5, 0, 0)
  // q_yaw  = (cos22.5, 0, 0, sin22.5)
  // q_combined = q_yaw * q_roll
  float a = 45.0f * DEG2RAD / 2.0f;
  float cr = cosf(a), sr = sinf(a);
  // q_roll
  float rw = cr, rx = sr, ry = 0, rz = 0;
  // q_yaw
  float yw = cr, yx = 0, yy = 0, yz = sr;
  // Multiply q_yaw * q_roll
  float qr = yw * rw - yx * rx - yy * ry - yz * rz;
  float qi = yw * rx + yx * rw + yy * rz - yz * ry;
  float qj = yw * ry - yx * rz + yy * rw + yz * rx;
  float qk = yw * rz + yx * ry - yy * rx + yz * rw;
  auto e = quatToEuler(qr, qi, qj, qk);
  // Should have both roll and yaw components
  TEST_ASSERT_TRUE(fabsf(e.roll) > 20.0f);
  TEST_ASSERT_TRUE(fabsf(e.yaw) > 20.0f);
  TEST_ASSERT_FALSE(std::isnan(e.roll));
  TEST_ASSERT_FALSE(std::isnan(e.yaw));
}

void test_axis_angle_arbitrary() {
  // 60 deg about (0, 0, 1) = pure yaw 60
  float qr, qi, qj, qk;
  axisAngleToQuat(0, 0, 1, 60.0f * DEG2RAD, qr, qi, qj, qk);
  auto e = quatToEuler(qr, qi, qj, qk);
  TEST_ASSERT_FLOAT_WITHIN(1.0f, 60.0f, e.yaw);
  TEST_ASSERT_FLOAT_WITHIN(1.0f, 0.0f, e.roll);
  TEST_ASSERT_FLOAT_WITHIN(1.0f, 0.0f, e.pitch);
}

// ════════════════════════════════════════════════════════════════
//  QUATERNION → EULER: CONSISTENCY / ROUNDTRIP
// ════════════════════════════════════════════════════════════════

void test_roll_increases_monotonically() {
  float prev = -1000.0f;
  for (int deg = -80; deg <= 80; deg += 10) {
    float angle = deg * DEG2RAD;
    float c = cosf(angle / 2.0f), s = sinf(angle / 2.0f);
    auto e = quatToEuler(c, s, 0.0f, 0.0f);
    TEST_ASSERT_TRUE(e.roll > prev);
    prev = e.roll;
  }
}

void test_yaw_increases_monotonically() {
  float prev = -1000.0f;
  for (int deg = -170; deg <= 170; deg += 20) {
    float angle = deg * DEG2RAD;
    float c = cosf(angle / 2.0f), s = sinf(angle / 2.0f);
    auto e = quatToEuler(c, 0.0f, 0.0f, s);
    TEST_ASSERT_TRUE(e.yaw > prev);
    prev = e.yaw;
  }
}

void test_pitch_increases_monotonically() {
  float prev = -1000.0f;
  for (int deg = -80; deg <= 80; deg += 10) {
    float angle = deg * DEG2RAD;
    float c = cosf(angle / 2.0f), s = sinf(angle / 2.0f);
    auto e = quatToEuler(c, 0.0f, s, 0.0f);
    TEST_ASSERT_TRUE(e.pitch > prev);
    prev = e.pitch;
  }
}

// ════════════════════════════════════════════════════════════════
//  WORLD-FRAME ROTATION: BASIC
// ════════════════════════════════════════════════════════════════

void test_zero_yaw_no_rotation() {
  float ax_w, ay_w;
  rotateToWorld(1.0f, 0.0f, 0.0f, ax_w, ay_w);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 1.0f, ax_w);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.0f, ay_w);
}

void test_90_yaw_rotates_x_to_y() {
  float ax_w, ay_w;
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

void test_270_yaw_rotates_x_to_neg_y() {
  float ax_w, ay_w;
  rotateToWorld(1.0f, 0.0f, (float)(3.0 * M_PI / 2.0), ax_w, ay_w);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, ax_w);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, -1.0f, ay_w);
}

void test_45_yaw_equal_components() {
  float ax_w, ay_w;
  rotateToWorld(1.0f, 0.0f, (float)(M_PI / 4.0), ax_w, ay_w);
  float expected = 1.0f / sqrtf(2.0f);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, expected, ax_w);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, expected, ay_w);
}

void test_neg_90_yaw_rotates_x_to_neg_y() {
  float ax_w, ay_w;
  rotateToWorld(1.0f, 0.0f, (float)(-M_PI / 2.0), ax_w, ay_w);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, ax_w);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, -1.0f, ay_w);
}

// ════════════════════════════════════════════════════════════════
//  WORLD-FRAME ROTATION: BODY Y-AXIS
// ════════════════════════════════════════════════════════════════

void test_90_yaw_body_y_to_neg_world_x() {
  float ax_w, ay_w;
  // Body (0, 1) with yaw=90: ax_w = -sin90*1 = -1, ay_w = cos90*1 = 0
  // Actually: ax_w = 0*cos90 - 1*sin90 = -1, ay_w = 0*sin90 + 1*cos90 = 0
  rotateToWorld(0.0f, 1.0f, (float)(M_PI / 2.0), ax_w, ay_w);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, -1.0f, ax_w);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, ay_w);
}

void test_zero_yaw_body_y_stays() {
  float ax_w, ay_w;
  rotateToWorld(0.0f, 1.0f, 0.0f, ax_w, ay_w);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.0f, ax_w);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 1.0f, ay_w);
}

// ════════════════════════════════════════════════════════════════
//  WORLD-FRAME ROTATION: BOTH BODY AXES
// ════════════════════════════════════════════════════════════════

void test_both_body_axes_zero_yaw() {
  float ax_w, ay_w;
  rotateToWorld(1.0f, 1.0f, 0.0f, ax_w, ay_w);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 1.0f, ax_w);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 1.0f, ay_w);
}

void test_both_body_axes_90_yaw() {
  float ax_w, ay_w;
  rotateToWorld(1.0f, 1.0f, (float)(M_PI / 2.0), ax_w, ay_w);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, -1.0f, ax_w);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 1.0f, ay_w);
}

void test_both_body_axes_45_yaw() {
  float ax_w, ay_w;
  // Body (1, 1) with yaw=45:
  // ax_w = cos45 - sin45 = 0
  // ay_w = sin45 + cos45 = sqrt(2)
  rotateToWorld(1.0f, 1.0f, (float)(M_PI / 4.0), ax_w, ay_w);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, ax_w);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, sqrtf(2.0f), ay_w);
}

// ════════════════════════════════════════════════════════════════
//  WORLD-FRAME ROTATION: MAGNITUDE & PROPERTIES
// ════════════════════════════════════════════════════════════════

void test_rotation_preserves_magnitude() {
  float ax = 3.0f, ay = 4.0f;
  float yaw = 1.234f;
  float ax_w, ay_w;
  rotateToWorld(ax, ay, yaw, ax_w, ay_w);
  float mag_body = sqrtf(ax * ax + ay * ay);
  float mag_world = sqrtf(ax_w * ax_w + ay_w * ay_w);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, mag_body, mag_world);
}

void test_rotation_preserves_magnitude_negative_accel() {
  float ax = -2.5f, ay = -3.7f;
  float yaw = 2.5f;
  float ax_w, ay_w;
  rotateToWorld(ax, ay, yaw, ax_w, ay_w);
  float mag_body = sqrtf(ax * ax + ay * ay);
  float mag_world = sqrtf(ax_w * ax_w + ay_w * ay_w);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, mag_body, mag_world);
}

void test_zero_accel_stays_zero() {
  float ax_w, ay_w;
  rotateToWorld(0.0f, 0.0f, 1.5f, ax_w, ay_w);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, ax_w);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, ay_w);
}

void test_full_rotation_identity() {
  float ax_w, ay_w;
  rotateToWorld(2.5f, -1.3f, (float)(2.0 * M_PI), ax_w, ay_w);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 2.5f, ax_w);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, -1.3f, ay_w);
}

void test_double_rotation_composes() {
  // Rotate by 30, then by 30 again = same as rotating by 60
  float ax = 1.0f, ay = 0.0f;
  float ax1, ay1, ax2, ay2;
  rotateToWorld(ax, ay, 30.0f * DEG2RAD, ax1, ay1);
  rotateToWorld(ax1, ay1, 30.0f * DEG2RAD, ax2, ay2);
  float ax_direct, ay_direct;
  rotateToWorld(ax, ay, 60.0f * DEG2RAD, ax_direct, ay_direct);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, ax_direct, ax2);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, ay_direct, ay2);
}

void test_rotate_and_inverse_returns_to_original() {
  float ax = 3.0f, ay = -2.0f;
  float ax1, ay1, ax2, ay2;
  float yaw = 1.2f;
  rotateToWorld(ax, ay, yaw, ax1, ay1);
  rotateToWorld(ax1, ay1, -yaw, ax2, ay2);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, ax, ax2);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, ay, ay2);
}

// ════════════════════════════════════════════════════════════════
//  WORLD-FRAME ROTATION: LARGE/NEGATIVE VALUES
// ════════════════════════════════════════════════════════════════

void test_large_accel_values() {
  float ax_w, ay_w;
  rotateToWorld(100.0f, 200.0f, (float)(M_PI / 6.0), ax_w, ay_w);
  float mag_body = sqrtf(100.0f * 100.0f + 200.0f * 200.0f);
  float mag_world = sqrtf(ax_w * ax_w + ay_w * ay_w);
  TEST_ASSERT_FLOAT_WITHIN(0.1f, mag_body, mag_world);
}

void test_negative_accel_x_only() {
  float ax_w, ay_w;
  rotateToWorld(-1.0f, 0.0f, 0.0f, ax_w, ay_w);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, -1.0f, ax_w);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.0f, ay_w);
}

void test_negative_accel_y_only() {
  float ax_w, ay_w;
  rotateToWorld(0.0f, -1.0f, 0.0f, ax_w, ay_w);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.0f, ax_w);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, -1.0f, ay_w);
}

void test_negative_accel_with_yaw() {
  float ax_w, ay_w;
  // (-1, 0) rotated by 90 deg: ax_w = -1*cos90 = 0, ay_w = -1*sin90 = -1
  rotateToWorld(-1.0f, 0.0f, (float)(M_PI / 2.0), ax_w, ay_w);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, ax_w);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, -1.0f, ay_w);
}

// ════════════════════════════════════════════════════════════════
//  WORLD-FRAME ROTATION: SPECIAL YAW VALUES
// ════════════════════════════════════════════════════════════════

void test_very_small_yaw() {
  float ax_w, ay_w;
  float yaw = 0.001f;
  rotateToWorld(1.0f, 0.0f, yaw, ax_w, ay_w);
  // cos(0.001)≈1, sin(0.001)≈0.001
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 1.0f, ax_w);
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.001f, ay_w);
}

void test_negative_yaw_rotation() {
  float ax_w, ay_w;
  // Negative 90 deg yaw: body X → world -Y
  rotateToWorld(1.0f, 0.0f, (float)(-M_PI / 2.0), ax_w, ay_w);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, ax_w);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, -1.0f, ay_w);
}

int main(int argc, char** argv) {
  UNITY_BEGIN();

  // Quaternion → Euler: identity & basic
  RUN_TEST(test_identity_quaternion_zero_euler);
  RUN_TEST(test_pure_roll_90);
  RUN_TEST(test_pure_roll_180);
  RUN_TEST(test_pure_roll_45);
  RUN_TEST(test_pure_pitch_45);
  RUN_TEST(test_pure_pitch_30);
  RUN_TEST(test_pure_yaw_90);
  RUN_TEST(test_pure_yaw_180);
  RUN_TEST(test_pure_yaw_45);
  RUN_TEST(test_pure_yaw_270);

  // Negative angles
  RUN_TEST(test_negative_roll_30);
  RUN_TEST(test_negative_roll_90);
  RUN_TEST(test_negative_pitch_45);
  RUN_TEST(test_negative_yaw_45);
  RUN_TEST(test_negative_yaw_90);

  // Small angles
  RUN_TEST(test_small_roll_1_deg);
  RUN_TEST(test_small_pitch_1_deg);
  RUN_TEST(test_small_yaw_1_deg);
  RUN_TEST(test_tiny_roll_0_1_deg);

  // Gimbal lock & edge cases
  RUN_TEST(test_pitch_near_90_gimbal_lock);
  RUN_TEST(test_pitch_near_neg90_gimbal_lock);
  RUN_TEST(test_pitch_exactly_90_clamped);
  RUN_TEST(test_normalized_quaternion_120_about_111);
  RUN_TEST(test_negative_quat_same_rotation);
  RUN_TEST(test_no_nan_for_zero_quaternion_components);

  // Combined rotations
  RUN_TEST(test_roll_and_yaw_combined);
  RUN_TEST(test_axis_angle_arbitrary);

  // Monotonicity / consistency
  RUN_TEST(test_roll_increases_monotonically);
  RUN_TEST(test_yaw_increases_monotonically);
  RUN_TEST(test_pitch_increases_monotonically);

  // World-frame rotation: basic
  RUN_TEST(test_zero_yaw_no_rotation);
  RUN_TEST(test_90_yaw_rotates_x_to_y);
  RUN_TEST(test_180_yaw_negates);
  RUN_TEST(test_270_yaw_rotates_x_to_neg_y);
  RUN_TEST(test_45_yaw_equal_components);
  RUN_TEST(test_neg_90_yaw_rotates_x_to_neg_y);

  // Body Y-axis
  RUN_TEST(test_90_yaw_body_y_to_neg_world_x);
  RUN_TEST(test_zero_yaw_body_y_stays);

  // Both body axes
  RUN_TEST(test_both_body_axes_zero_yaw);
  RUN_TEST(test_both_body_axes_90_yaw);
  RUN_TEST(test_both_body_axes_45_yaw);

  // Magnitude & properties
  RUN_TEST(test_rotation_preserves_magnitude);
  RUN_TEST(test_rotation_preserves_magnitude_negative_accel);
  RUN_TEST(test_zero_accel_stays_zero);
  RUN_TEST(test_full_rotation_identity);
  RUN_TEST(test_double_rotation_composes);
  RUN_TEST(test_rotate_and_inverse_returns_to_original);

  // Large/negative values
  RUN_TEST(test_large_accel_values);
  RUN_TEST(test_negative_accel_x_only);
  RUN_TEST(test_negative_accel_y_only);
  RUN_TEST(test_negative_accel_with_yaw);

  // Special yaw values
  RUN_TEST(test_very_small_yaw);
  RUN_TEST(test_negative_yaw_rotation);

  return UNITY_END();
}
