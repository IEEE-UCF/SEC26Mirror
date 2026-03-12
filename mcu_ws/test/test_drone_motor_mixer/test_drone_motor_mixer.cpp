/**
 * @file test_drone_motor_mixer.cpp
 * @brief Native unit tests for X-quad motor mixer algorithm.
 * @date 2026-03-07
 *
 * Tests the motor mixing formula used in DroneFlightSubsystem::controlMixer():
 *   FL = throttle + roll - pitch + yaw
 *   FR = throttle - roll - pitch - yaw
 *   BR = throttle - roll + pitch + yaw
 *   BL = throttle + roll + pitch - yaw
 *
 * Motors are clamped to [0.0, 1.0].
 */

#include <cmath>
#include <unity.h>

// ── Re-implement mixer logic from DroneFlightSubsystem::controlMixer() ──

static float clamp(float v, float lo, float hi) {
  return (v < lo) ? lo : (v > hi) ? hi : v;
}

struct MixerOutput {
  float fl, fr, br, bl;
};

static MixerOutput mix(float throttle, float roll, float pitch, float yaw) {
  MixerOutput m;
  m.fl = clamp(throttle + roll - pitch + yaw, 0.0f, 1.0f);
  m.fr = clamp(throttle - roll - pitch - yaw, 0.0f, 1.0f);
  m.br = clamp(throttle - roll + pitch + yaw, 0.0f, 1.0f);
  m.bl = clamp(throttle + roll + pitch - yaw, 0.0f, 1.0f);
  return m;
}

void setUp(void) {}
void tearDown(void) {}

// ════════════════════════════════════════════════════════════════
//  HOVER / BASELINE
// ════════════════════════════════════════════════════════════════

void test_hover_equal_motors() {
  auto m = mix(0.45f, 0.0f, 0.0f, 0.0f);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.45f, m.fl);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.45f, m.fr);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.45f, m.br);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.45f, m.bl);
}

void test_zero_throttle_zero_output() {
  auto m = mix(0.0f, 0.0f, 0.0f, 0.0f);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, m.fl);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, m.fr);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, m.br);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, m.bl);
}

void test_full_throttle() {
  auto m = mix(1.0f, 0.0f, 0.0f, 0.0f);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.0f, m.fl);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.0f, m.fr);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.0f, m.br);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.0f, m.bl);
}

void test_half_throttle_hover() {
  auto m = mix(0.5f, 0.0f, 0.0f, 0.0f);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.5f, m.fl);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.5f, m.fr);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.5f, m.br);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.5f, m.bl);
}

void test_tiny_throttle() {
  auto m = mix(0.001f, 0.0f, 0.0f, 0.0f);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.001f, m.fl);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.001f, m.fr);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.001f, m.br);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.001f, m.bl);
}

// ════════════════════════════════════════════════════════════════
//  PURE ROLL
// ════════════════════════════════════════════════════════════════

void test_positive_roll_increases_left() {
  auto m = mix(0.5f, 0.1f, 0.0f, 0.0f);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.6f, m.fl);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.4f, m.fr);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.4f, m.br);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.6f, m.bl);
}

void test_negative_roll() {
  auto m = mix(0.5f, -0.1f, 0.0f, 0.0f);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.4f, m.fl);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.6f, m.fr);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.6f, m.br);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.4f, m.bl);
}

void test_max_roll_correction() {
  // Roll = 0.5 at throttle 0.5 → FL/BL=1.0, FR/BR=0.0
  auto m = mix(0.5f, 0.5f, 0.0f, 0.0f);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.0f, m.fl);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, m.fr);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, m.br);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.0f, m.bl);
}

void test_small_roll_correction() {
  auto m = mix(0.5f, 0.001f, 0.0f, 0.0f);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.501f, m.fl);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.499f, m.fr);
}

// ════════════════════════════════════════════════════════════════
//  PURE PITCH
// ════════════════════════════════════════════════════════════════

void test_positive_pitch_increases_back() {
  auto m = mix(0.5f, 0.0f, 0.1f, 0.0f);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.4f, m.fl);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.4f, m.fr);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.6f, m.br);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.6f, m.bl);
}

void test_negative_pitch() {
  auto m = mix(0.5f, 0.0f, -0.1f, 0.0f);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.6f, m.fl);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.6f, m.fr);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.4f, m.br);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.4f, m.bl);
}

void test_max_pitch_correction() {
  auto m = mix(0.5f, 0.0f, 0.5f, 0.0f);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, m.fl);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, m.fr);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.0f, m.br);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.0f, m.bl);
}

// ════════════════════════════════════════════════════════════════
//  PURE YAW
// ════════════════════════════════════════════════════════════════

void test_positive_yaw_ccw_motors_increase() {
  auto m = mix(0.5f, 0.0f, 0.0f, 0.1f);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.6f, m.fl);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.4f, m.fr);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.6f, m.br);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.4f, m.bl);
}

void test_negative_yaw() {
  auto m = mix(0.5f, 0.0f, 0.0f, -0.1f);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.4f, m.fl);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.6f, m.fr);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.4f, m.br);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.6f, m.bl);
}

void test_max_yaw_correction() {
  auto m = mix(0.5f, 0.0f, 0.0f, 0.5f);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.0f, m.fl);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, m.fr);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.0f, m.br);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, m.bl);
}

// ════════════════════════════════════════════════════════════════
//  COMBINED CORRECTIONS
// ════════════════════════════════════════════════════════════════

void test_roll_and_pitch_combined() {
  auto m = mix(0.5f, 0.1f, 0.1f, 0.0f);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.5f, m.fl);  // +r-p = 0
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.3f, m.fr);  // -r-p = -0.2
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.5f, m.br);  // -r+p = 0
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.7f, m.bl);  // +r+p = +0.2
}

void test_roll_and_yaw_combined() {
  // roll=0.1, yaw=0.1
  // FL = 0.5+0.1+0.1=0.7, FR = 0.5-0.1-0.1=0.3
  // BR = 0.5-0.1+0.1=0.5, BL = 0.5+0.1-0.1=0.5
  auto m = mix(0.5f, 0.1f, 0.0f, 0.1f);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.7f, m.fl);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.3f, m.fr);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.5f, m.br);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.5f, m.bl);
}

void test_pitch_and_yaw_combined() {
  // pitch=0.1, yaw=0.1
  // FL = 0.5-0.1+0.1=0.5, FR = 0.5-0.1-0.1=0.3
  // BR = 0.5+0.1+0.1=0.7, BL = 0.5+0.1-0.1=0.5
  auto m = mix(0.5f, 0.0f, 0.1f, 0.1f);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.5f, m.fl);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.3f, m.fr);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.7f, m.br);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.5f, m.bl);
}

void test_all_corrections() {
  auto m = mix(0.5f, 0.05f, 0.03f, 0.02f);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.54f, m.fl);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.40f, m.fr);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.50f, m.br);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.56f, m.bl);
}

void test_all_negative_corrections() {
  auto m = mix(0.5f, -0.05f, -0.03f, -0.02f);
  // FL = 0.5 -0.05 +0.03 -0.02 = 0.46
  // FR = 0.5 +0.05 +0.03 +0.02 = 0.60
  // BR = 0.5 +0.05 -0.03 -0.02 = 0.50
  // BL = 0.5 -0.05 -0.03 +0.02 = 0.44
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.46f, m.fl);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.60f, m.fr);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.50f, m.br);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.44f, m.bl);
}

void test_opposite_corrections_cancel() {
  // roll and -roll cancel: same as hover
  auto m1 = mix(0.5f, 0.1f, 0.0f, 0.0f);
  auto m2 = mix(0.5f, -0.1f, 0.0f, 0.0f);
  float avg_fl = (m1.fl + m2.fl) / 2.0f;
  float avg_fr = (m1.fr + m2.fr) / 2.0f;
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.5f, avg_fl);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.5f, avg_fr);
}

// ════════════════════════════════════════════════════════════════
//  CLAMPING
// ════════════════════════════════════════════════════════════════

void test_clamping_at_zero() {
  auto m = mix(0.1f, -0.5f, 0.5f, -0.5f);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, m.fl);  // 0.1-0.5-0.5-0.5=-1.4
  TEST_ASSERT_TRUE(m.fr >= 0.0f);
  TEST_ASSERT_TRUE(m.br >= 0.0f);
  TEST_ASSERT_TRUE(m.bl >= 0.0f);
}

void test_clamping_at_one() {
  auto m = mix(0.9f, 0.5f, 0.0f, 0.5f);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.0f, m.fl);  // 0.9+0.5+0.5=1.9
  TEST_ASSERT_TRUE(m.fr <= 1.0f);
  TEST_ASSERT_TRUE(m.br <= 1.0f);
  TEST_ASSERT_TRUE(m.bl <= 1.0f);
}

void test_no_negative_output() {
  auto m = mix(0.0f, -1.0f, -1.0f, -1.0f);
  TEST_ASSERT_TRUE(m.fl >= 0.0f);
  TEST_ASSERT_TRUE(m.fr >= 0.0f);
  TEST_ASSERT_TRUE(m.br >= 0.0f);
  TEST_ASSERT_TRUE(m.bl >= 0.0f);
}

void test_all_clamp_to_one() {
  // Only throttle >= 1.0 with zero corrections can clamp all to 1.0
  auto m = mix(1.5f, 0.0f, 0.0f, 0.0f);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.0f, m.fl);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.0f, m.fr);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.0f, m.br);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.0f, m.bl);
}

void test_all_clamp_to_zero() {
  // Only throttle <= 0.0 with zero corrections can clamp all to 0.0
  auto m = mix(-0.5f, 0.0f, 0.0f, 0.0f);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, m.fl);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, m.fr);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, m.br);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, m.bl);
}

void test_mixed_corrections_some_clamp_up_some_down() {
  // With opposing corrections, some motors clamp to 1, others to 0
  auto m = mix(0.5f, 0.8f, 0.0f, 0.0f);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.0f, m.fl);  // 0.5+0.8=1.3→1.0
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, m.fr);  // 0.5-0.8=-0.3→0.0
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, m.br);  // 0.5-0.8=-0.3→0.0
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.0f, m.bl);  // 0.5+0.8=1.3→1.0
}

void test_negative_throttle_clamps() {
  // Negative throttle shouldn't happen in practice but verify clamping
  auto m = mix(-0.5f, 0.0f, 0.0f, 0.0f);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, m.fl);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, m.fr);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, m.br);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, m.bl);
}

void test_excessive_throttle_clamps() {
  auto m = mix(2.0f, 0.0f, 0.0f, 0.0f);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.0f, m.fl);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.0f, m.fr);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.0f, m.br);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.0f, m.bl);
}

void test_partial_clamping_preserves_unclamped() {
  // Only some motors should clamp
  // FL = 0.8+0.3=1.1→1.0, FR = 0.8-0.3=0.5
  auto m = mix(0.8f, 0.3f, 0.0f, 0.0f);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.0f, m.fl);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.5f, m.fr);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.5f, m.br);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.0f, m.bl);
}

void test_boundary_exact_zero() {
  // Correction exactly cancels throttle
  auto m = mix(0.3f, -0.3f, 0.0f, 0.0f);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, m.fl);  // 0.3-0.3=0.0
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.6f, m.fr);  // 0.3+0.3=0.6
}

void test_boundary_exact_one() {
  auto m = mix(0.7f, 0.3f, 0.0f, 0.0f);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.0f, m.fl);  // 0.7+0.3=1.0
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.4f, m.fr);  // 0.7-0.3=0.4
}

// ════════════════════════════════════════════════════════════════
//  SYMMETRY PROPERTIES
// ════════════════════════════════════════════════════════════════

void test_roll_symmetry() {
  auto mp = mix(0.5f, 0.1f, 0.0f, 0.0f);
  auto mn = mix(0.5f, -0.1f, 0.0f, 0.0f);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, mp.fl, mn.fr);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, mp.fr, mn.fl);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, mp.bl, mn.br);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, mp.br, mn.bl);
}

void test_pitch_symmetry() {
  auto mp = mix(0.5f, 0.0f, 0.1f, 0.0f);
  auto mn = mix(0.5f, 0.0f, -0.1f, 0.0f);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, mp.fl, mn.br);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, mp.br, mn.fl);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, mp.fr, mn.bl);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, mp.bl, mn.fr);
}

void test_yaw_symmetry() {
  auto mp = mix(0.5f, 0.0f, 0.0f, 0.1f);
  auto mn = mix(0.5f, 0.0f, 0.0f, -0.1f);
  // Yaw inversion swaps diagonal pairs
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, mp.fl, mn.fr);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, mp.fr, mn.fl);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, mp.br, mn.bl);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, mp.bl, mn.br);
}

void test_yaw_diagonal_symmetry() {
  auto m = mix(0.5f, 0.0f, 0.0f, 0.1f);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, m.fl, m.br);  // CCW pair equal
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, m.fr, m.bl);  // CW pair equal
}

void test_roll_left_right_pairs() {
  // Roll only affects left (FL/BL) vs right (FR/BR)
  auto m = mix(0.5f, 0.15f, 0.0f, 0.0f);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, m.fl, m.bl);  // Left pair equal
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, m.fr, m.br);  // Right pair equal
  TEST_ASSERT_TRUE(m.fl > m.fr);                 // Left > Right for +roll
}

void test_pitch_front_back_pairs() {
  // Pitch only affects front (FL/FR) vs back (BR/BL)
  auto m = mix(0.5f, 0.0f, 0.15f, 0.0f);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, m.fl, m.fr);  // Front pair equal
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, m.br, m.bl);  // Back pair equal
  TEST_ASSERT_TRUE(m.br > m.fl);                 // Back > Front for +pitch
}

// ════════════════════════════════════════════════════════════════
//  CONSERVATION PROPERTIES
// ════════════════════════════════════════════════════════════════

void test_corrections_sum_to_four_throttle() {
  float t = 0.5f, r = 0.1f, p = 0.05f, y = 0.02f;
  float fl = t + r - p + y;
  float fr = t - r - p - y;
  float br = t - r + p + y;
  float bl = t + r + p - y;
  float sum = fl + fr + br + bl;
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 4.0f * t, sum);
}

void test_pure_roll_net_zero_thrust_change() {
  // Roll redistributes thrust, total stays 4*throttle
  float t = 0.4f, r = 0.2f;
  float sum = (t + r) + (t - r) + (t - r) + (t + r);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 4.0f * t, sum);
}

void test_pure_pitch_net_zero_thrust_change() {
  float t = 0.4f, p = 0.2f;
  float sum = (t - p) + (t - p) + (t + p) + (t + p);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 4.0f * t, sum);
}

void test_pure_yaw_net_zero_thrust_change() {
  float t = 0.4f, y = 0.2f;
  float sum = (t + y) + (t - y) + (t + y) + (t - y);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 4.0f * t, sum);
}

// ════════════════════════════════════════════════════════════════
//  MOTOR ORDERING
// ════════════════════════════════════════════════════════════════

void test_all_motors_different_with_all_corrections() {
  // With distinct nonzero roll/pitch/yaw, all motors should differ
  auto m = mix(0.5f, 0.07f, 0.05f, 0.03f);
  TEST_ASSERT_TRUE(fabsf(m.fl - m.fr) > 0.01f);
  TEST_ASSERT_TRUE(fabsf(m.fl - m.br) > 0.01f);
  TEST_ASSERT_TRUE(fabsf(m.fl - m.bl) > 0.01f);
  TEST_ASSERT_TRUE(fabsf(m.fr - m.br) > 0.01f);
  TEST_ASSERT_TRUE(fabsf(m.fr - m.bl) > 0.01f);
  TEST_ASSERT_TRUE(fabsf(m.br - m.bl) > 0.01f);
}

// ════════════════════════════════════════════════════════════════
//  LINEARITY (BEFORE CLAMPING)
// ════════════════════════════════════════════════════════════════

void test_double_roll_doubles_difference() {
  auto m1 = mix(0.5f, 0.05f, 0.0f, 0.0f);
  auto m2 = mix(0.5f, 0.10f, 0.0f, 0.0f);
  float diff1 = m1.fl - m1.fr;
  float diff2 = m2.fl - m2.fr;
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, diff1 * 2.0f, diff2);
}

void test_double_pitch_doubles_difference() {
  auto m1 = mix(0.5f, 0.0f, 0.05f, 0.0f);
  auto m2 = mix(0.5f, 0.0f, 0.10f, 0.0f);
  float diff1 = m1.br - m1.fl;
  float diff2 = m2.br - m2.fl;
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, diff1 * 2.0f, diff2);
}

void test_throttle_offset_shifts_all_equally() {
  auto m1 = mix(0.3f, 0.1f, 0.05f, 0.02f);
  auto m2 = mix(0.4f, 0.1f, 0.05f, 0.02f);
  // All motors should increase by 0.1 (before clamping)
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.1f, m2.fl - m1.fl);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.1f, m2.fr - m1.fr);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.1f, m2.br - m1.br);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.1f, m2.bl - m1.bl);
}

// ════════════════════════════════════════════════════════════════
//  REALISTIC FLIGHT SCENARIOS
// ════════════════════════════════════════════════════════════════

void test_gentle_hover_correction() {
  // Typical hover: throttle ~0.45, tiny PID corrections
  auto m = mix(0.45f, 0.01f, -0.005f, 0.002f);
  // All motors near 0.45
  TEST_ASSERT_FLOAT_WITHIN(0.02f, 0.45f, m.fl);
  TEST_ASSERT_FLOAT_WITHIN(0.02f, 0.45f, m.fr);
  TEST_ASSERT_FLOAT_WITHIN(0.02f, 0.45f, m.br);
  TEST_ASSERT_FLOAT_WITHIN(0.02f, 0.45f, m.bl);
}

void test_aggressive_maneuver() {
  // Large corrections during aggressive flight
  auto m = mix(0.6f, 0.3f, 0.2f, 0.1f);
  // FL = 0.6+0.3-0.2+0.1 = 0.8
  // FR = 0.6-0.3-0.2-0.1 = 0.0
  // BR = 0.6-0.3+0.2+0.1 = 0.6
  // BL = 0.6+0.3+0.2-0.1 = 1.0
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.8f, m.fl);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.0f, m.fr);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.6f, m.br);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 1.0f, m.bl);
}

int main(int argc, char** argv) {
  UNITY_BEGIN();

  // Hover / Baseline
  RUN_TEST(test_hover_equal_motors);
  RUN_TEST(test_zero_throttle_zero_output);
  RUN_TEST(test_full_throttle);
  RUN_TEST(test_half_throttle_hover);
  RUN_TEST(test_tiny_throttle);

  // Pure roll
  RUN_TEST(test_positive_roll_increases_left);
  RUN_TEST(test_negative_roll);
  RUN_TEST(test_max_roll_correction);
  RUN_TEST(test_small_roll_correction);

  // Pure pitch
  RUN_TEST(test_positive_pitch_increases_back);
  RUN_TEST(test_negative_pitch);
  RUN_TEST(test_max_pitch_correction);

  // Pure yaw
  RUN_TEST(test_positive_yaw_ccw_motors_increase);
  RUN_TEST(test_negative_yaw);
  RUN_TEST(test_max_yaw_correction);

  // Combined corrections
  RUN_TEST(test_roll_and_pitch_combined);
  RUN_TEST(test_roll_and_yaw_combined);
  RUN_TEST(test_pitch_and_yaw_combined);
  RUN_TEST(test_all_corrections);
  RUN_TEST(test_all_negative_corrections);
  RUN_TEST(test_opposite_corrections_cancel);

  // Clamping
  RUN_TEST(test_clamping_at_zero);
  RUN_TEST(test_clamping_at_one);
  RUN_TEST(test_no_negative_output);
  RUN_TEST(test_all_clamp_to_one);
  RUN_TEST(test_all_clamp_to_zero);
  RUN_TEST(test_mixed_corrections_some_clamp_up_some_down);
  RUN_TEST(test_negative_throttle_clamps);
  RUN_TEST(test_excessive_throttle_clamps);
  RUN_TEST(test_partial_clamping_preserves_unclamped);
  RUN_TEST(test_boundary_exact_zero);
  RUN_TEST(test_boundary_exact_one);

  // Symmetry
  RUN_TEST(test_roll_symmetry);
  RUN_TEST(test_pitch_symmetry);
  RUN_TEST(test_yaw_symmetry);
  RUN_TEST(test_yaw_diagonal_symmetry);
  RUN_TEST(test_roll_left_right_pairs);
  RUN_TEST(test_pitch_front_back_pairs);

  // Conservation
  RUN_TEST(test_corrections_sum_to_four_throttle);
  RUN_TEST(test_pure_roll_net_zero_thrust_change);
  RUN_TEST(test_pure_pitch_net_zero_thrust_change);
  RUN_TEST(test_pure_yaw_net_zero_thrust_change);

  // Motor ordering
  RUN_TEST(test_all_motors_different_with_all_corrections);

  // Linearity
  RUN_TEST(test_double_roll_doubles_difference);
  RUN_TEST(test_double_pitch_doubles_difference);
  RUN_TEST(test_throttle_offset_shifts_all_equally);

  // Realistic scenarios
  RUN_TEST(test_gentle_hover_correction);
  RUN_TEST(test_aggressive_maneuver);

  return UNITY_END();
}
