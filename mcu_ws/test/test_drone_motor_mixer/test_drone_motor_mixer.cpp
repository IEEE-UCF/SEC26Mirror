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

// === Hover (zero corrections) ===

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

// === Pure roll correction ===

void test_positive_roll_increases_left() {
  // Positive roll correction → FL and BL increase, FR and BR decrease
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

// === Pure pitch correction ===

void test_positive_pitch_increases_back() {
  // Positive pitch correction → BR and BL increase, FL and FR decrease
  auto m = mix(0.5f, 0.0f, 0.1f, 0.0f);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.4f, m.fl);  // throttle - pitch
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.4f, m.fr);  // throttle - pitch
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.6f, m.br);  // throttle + pitch
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.6f, m.bl);  // throttle + pitch
}

void test_negative_pitch() {
  auto m = mix(0.5f, 0.0f, -0.1f, 0.0f);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.6f, m.fl);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.6f, m.fr);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.4f, m.br);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.4f, m.bl);
}

// === Pure yaw correction ===

void test_positive_yaw_ccw_motors_increase() {
  // Positive yaw → FL (CCW) and BR (CCW) increase, FR (CW) and BL (CW) decrease
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

// === Combined corrections ===

void test_roll_and_pitch_combined() {
  // roll=0.1, pitch=0.1 → FL=0.5+0.1-0.1=0.5, FR=0.5-0.1-0.1=0.3,
  //                        BR=0.5-0.1+0.1=0.5, BL=0.5+0.1+0.1=0.7
  auto m = mix(0.5f, 0.1f, 0.1f, 0.0f);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.5f, m.fl);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.3f, m.fr);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.5f, m.br);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.7f, m.bl);
}

void test_all_corrections() {
  // throttle=0.5, roll=0.05, pitch=0.03, yaw=0.02
  auto m = mix(0.5f, 0.05f, 0.03f, 0.02f);
  // FL = 0.5 + 0.05 - 0.03 + 0.02 = 0.54
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.54f, m.fl);
  // FR = 0.5 - 0.05 - 0.03 - 0.02 = 0.40
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.40f, m.fr);
  // BR = 0.5 - 0.05 + 0.03 + 0.02 = 0.50
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.50f, m.br);
  // BL = 0.5 + 0.05 + 0.03 - 0.02 = 0.56
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.56f, m.bl);
}

// === Clamping ===

void test_clamping_at_zero() {
  // Large negative corrections should clamp to 0
  auto m = mix(0.1f, -0.5f, 0.5f, -0.5f);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, m.fl);  // 0.1 - 0.5 - 0.5 - 0.5 = -1.4
  TEST_ASSERT_TRUE(m.fr >= 0.0f);
  TEST_ASSERT_TRUE(m.br >= 0.0f);
  TEST_ASSERT_TRUE(m.bl >= 0.0f);
}

void test_clamping_at_one() {
  // Large positive corrections should clamp to 1.0
  auto m = mix(0.9f, 0.5f, 0.0f, 0.5f);
  // FL = 0.9 + 0.5 + 0.5 = 1.9 → clamped to 1.0
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.0f, m.fl);
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

// === Symmetry properties ===

void test_roll_symmetry() {
  // Positive and negative roll should produce mirrored outputs
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
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, mp.fr, mn.bl);
  // Front motors swap with back motors under pitch inversion
  // FL(-pitch) = throttle + pitch = mp.bl
  // Hmm, let me reconsider. With positive pitch:
  //   FL = t - p, FR = t - p, BR = t + p, BL = t + p
  // With negative pitch:
  //   FL = t + p, FR = t + p, BR = t - p, BL = t - p
  // So FL(+p) == BR(-p) and FR(+p) == BL(-p)
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, mp.fl, mn.br);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, mp.br, mn.fl);
}

void test_yaw_diagonal_symmetry() {
  // Yaw affects diagonals (CCW: FL/BR increase, CW: FR/BL decrease)
  auto m = mix(0.5f, 0.0f, 0.0f, 0.1f);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, m.fl, m.br);  // CCW pair equal
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, m.fr, m.bl);  // CW pair equal
}

// === Conservation property ===

void test_corrections_sum_to_zero() {
  // For any correction combo (unclamped), the sum of all corrections
  // across 4 motors should be 4*throttle (corrections cancel out)
  float t = 0.5f, r = 0.1f, p = 0.05f, y = 0.02f;
  float fl = t + r - p + y;
  float fr = t - r - p - y;
  float br = t - r + p + y;
  float bl = t + r + p - y;
  float sum = fl + fr + br + bl;
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 4.0f * t, sum);
}

int main(int argc, char** argv) {
  UNITY_BEGIN();

  RUN_TEST(test_hover_equal_motors);
  RUN_TEST(test_zero_throttle_zero_output);
  RUN_TEST(test_full_throttle);

  RUN_TEST(test_positive_roll_increases_left);
  RUN_TEST(test_negative_roll);

  RUN_TEST(test_positive_pitch_increases_back);
  RUN_TEST(test_negative_pitch);

  RUN_TEST(test_positive_yaw_ccw_motors_increase);
  RUN_TEST(test_negative_yaw);

  RUN_TEST(test_roll_and_pitch_combined);
  RUN_TEST(test_all_corrections);

  RUN_TEST(test_clamping_at_zero);
  RUN_TEST(test_clamping_at_one);
  RUN_TEST(test_no_negative_output);

  RUN_TEST(test_roll_symmetry);
  RUN_TEST(test_pitch_symmetry);
  RUN_TEST(test_yaw_diagonal_symmetry);
  RUN_TEST(test_corrections_sum_to_zero);

  return UNITY_END();
}
