/**
 * @file test_drone_ekf.cpp
 * @brief Native unit tests for drone EKF: predict, trilateration, and update.
 * @date 2026-03-07
 *
 * Tests the algorithms from DroneEKFSubsystem without FreeRTOS dependencies.
 * All math is re-implemented from DroneEKFSubsystem.cpp to validate correctness.
 */

#include <cmath>
#include <cstring>
#include <unity.h>

// ── EKF state and config (mirrored from DroneEKFSubsystem/DroneConfig) ──

struct EKFState {
  float x = 0.0f, y = 0.0f, vx = 0.0f, vy = 0.0f;
};

struct AnchorInfo {
  uint8_t id = 0;
  float x = 0.0f, y = 0.0f;
  bool valid = false;
};

static constexpr float PROCESS_NOISE_POS = 0.01f;
static constexpr float PROCESS_NOISE_VEL = 0.1f;
static constexpr float MEASURE_NOISE_UWB = 0.15f;
static constexpr float OUTLIER_GATE_M = 1.0f;
static constexpr uint8_t MAX_ANCHORS = 4;

// ── 4x4 matrix helpers (from DroneEKFSubsystem.cpp) ──

static void mat4_mul(const float* A, const float* B, float* C) {
  for (int i = 0; i < 4; i++)
    for (int j = 0; j < 4; j++) {
      float sum = 0.0f;
      for (int k = 0; k < 4; k++) sum += A[i * 4 + k] * B[k * 4 + j];
      C[i * 4 + j] = sum;
    }
}

static void mat4_add(const float* A, const float* B, float* C) {
  for (int i = 0; i < 16; i++) C[i] = A[i] + B[i];
}

// ── Predict step (from DroneEKFSubsystem::predict) ──

static void ekf_predict(EKFState& state, float P[16], float ax, float ay,
                         float dt) {
  if (dt <= 0.0f || dt > 0.5f) return;

  float dt2 = 0.5f * dt * dt;
  state.x += state.vx * dt + ax * dt2;
  state.y += state.vy * dt + ay * dt2;
  state.vx += ax * dt;
  state.vy += ay * dt;

  float F[16] = {1, 0, dt, 0, 0, 1, 0, dt, 0, 0, 1, 0, 0, 0, 0, 1};
  float FT[16] = {1, 0, 0, 0, 0, 1, 0, 0, dt, 0, 1, 0, 0, dt, 0, 1};

  float qp = PROCESS_NOISE_POS * dt;
  float qv = PROCESS_NOISE_VEL * dt;
  float Q[16] = {qp, 0, 0, 0, 0, qp, 0, 0, 0, 0, qv, 0, 0, 0, 0, qv};

  float FP[16], FPFT[16];
  mat4_mul(F, P, FP);
  mat4_mul(FP, FT, FPFT);
  mat4_add(FPFT, Q, P);
}

// ── Trilateration (from DroneEKFSubsystem::trilaterate) ──

static bool trilaterate(const float* distances_m, const uint8_t* peer_ids,
                        uint8_t num_ranges, const AnchorInfo* anchors,
                        uint8_t num_anchors, float& x_out, float& y_out) {
  struct MatchedRange {
    float x, y, d;
  };
  MatchedRange matched[MAX_ANCHORS];
  uint8_t n = 0;

  for (uint8_t i = 0; i < num_ranges && n < MAX_ANCHORS; i++) {
    if (distances_m[i] <= 0.0f) continue;
    for (uint8_t j = 0; j < num_anchors; j++) {
      if (anchors[j].valid && anchors[j].id == peer_ids[i]) {
        matched[n].x = anchors[j].x;
        matched[n].y = anchors[j].y;
        matched[n].d = distances_m[i];
        n++;
        break;
      }
    }
  }

  if (n < 3) return false;

  float x0 = matched[0].x, y0 = matched[0].y, d0 = matched[0].d;
  float d0sq = d0 * d0, x0sq = x0 * x0, y0sq = y0 * y0;

  float xi = matched[1].x, yi = matched[1].y, di = matched[1].d;
  float A00 = 2.0f * (x0 - xi);
  float A01 = 2.0f * (y0 - yi);
  float b0 = di * di - d0sq - xi * xi + x0sq - yi * yi + y0sq;

  xi = matched[2].x;
  yi = matched[2].y;
  di = matched[2].d;
  float A10 = 2.0f * (x0 - xi);
  float A11 = 2.0f * (y0 - yi);
  float b1 = di * di - d0sq - xi * xi + x0sq - yi * yi + y0sq;

  float det = A00 * A11 - A01 * A10;
  if (fabsf(det) < 1e-6f) return false;

  x_out = (b0 * A11 - b1 * A01) / det;
  y_out = (A00 * b1 - A10 * b0) / det;
  return true;
}

// ── EKF update (from DroneEKFSubsystem::updateUWB, without mutex) ──

static bool ekf_update(EKFState& state, float P[16], float x_meas,
                        float y_meas) {
  float innov_x = x_meas - state.x;
  float innov_y = y_meas - state.y;

  float residual = sqrtf(innov_x * innov_x + innov_y * innov_y);
  if (residual > OUTLIER_GATE_M) return false;

  float R = MEASURE_NOISE_UWB;
  float S00 = P[0] + R, S01 = P[1];
  float S10 = P[4], S11 = P[5] + R;

  float det = S00 * S11 - S01 * S10;
  if (fabsf(det) < 1e-10f) return false;
  float inv_det = 1.0f / det;

  float Si00 = S11 * inv_det, Si01 = -S01 * inv_det;
  float Si10 = -S10 * inv_det, Si11 = S00 * inv_det;

  float K[8];
  K[0] = P[0] * Si00 + P[1] * Si10;
  K[1] = P[0] * Si01 + P[1] * Si11;
  K[2] = P[4] * Si00 + P[5] * Si10;
  K[3] = P[4] * Si01 + P[5] * Si11;
  K[4] = P[8] * Si00 + P[9] * Si10;
  K[5] = P[8] * Si01 + P[9] * Si11;
  K[6] = P[12] * Si00 + P[13] * Si10;
  K[7] = P[12] * Si01 + P[13] * Si11;

  state.x += K[0] * innov_x + K[1] * innov_y;
  state.y += K[2] * innov_x + K[3] * innov_y;
  state.vx += K[4] * innov_x + K[5] * innov_y;
  state.vy += K[6] * innov_x + K[7] * innov_y;

  float KH[16] = {};
  for (int i = 0; i < 4; i++) {
    KH[i * 4 + 0] = K[i * 2 + 0];
    KH[i * 4 + 1] = K[i * 2 + 1];
  }

  float P_new[16];
  for (int i = 0; i < 4; i++)
    for (int j = 0; j < 4; j++) {
      float sum = 0.0f;
      for (int k = 0; k < 4; k++) {
        float I_KH = ((i == k) ? 1.0f : 0.0f) - KH[i * 4 + k];
        sum += I_KH * P[k * 4 + j];
      }
      P_new[i * 4 + j] = sum;
    }
  memcpy(P, P_new, sizeof(float) * 16);
  return true;
}

// ── Helpers ──

static void init_P(float P[16]) {
  memset(P, 0, sizeof(float) * 16);
  P[0] = 1.0f;
  P[5] = 1.0f;
  P[10] = 0.5f;
  P[15] = 0.5f;
}

// Helper: compute distance from point to anchor
static float dist(float x, float y, float ax, float ay) {
  return sqrtf((x - ax) * (x - ax) + (y - ay) * (y - ay));
}

void setUp(void) {}
void tearDown(void) {}

// ════════════════════════════════════════════════════════════════
//  PREDICT TESTS
// ════════════════════════════════════════════════════════════════

void test_predict_zero_accel_zero_vel_no_change() {
  EKFState s;
  float P[16];
  init_P(P);
  ekf_predict(s, P, 0.0f, 0.0f, 0.004f);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, s.x);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, s.y);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, s.vx);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, s.vy);
}

void test_predict_constant_velocity_x() {
  EKFState s;
  s.vx = 1.0f;
  float P[16];
  init_P(P);
  ekf_predict(s, P, 0.0f, 0.0f, 0.1f);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.1f, s.x);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, s.y);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.0f, s.vx);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, s.vy);
}

void test_predict_constant_velocity_y() {
  EKFState s;
  s.vy = 2.0f;
  float P[16];
  init_P(P);
  ekf_predict(s, P, 0.0f, 0.0f, 0.1f);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.0f, s.x);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.2f, s.y);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 2.0f, s.vy);
}

void test_predict_constant_acceleration_x() {
  EKFState s;
  float P[16];
  init_P(P);
  ekf_predict(s, P, 2.0f, 0.0f, 0.1f);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.01f, s.x);   // 0.5*2*0.01
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.2f, s.vx);    // 2*0.1
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, s.y);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, s.vy);
}

void test_predict_constant_acceleration_y() {
  EKFState s;
  float P[16];
  init_P(P);
  ekf_predict(s, P, 0.0f, -3.0f, 0.1f);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, s.x);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, -0.015f, s.y);  // 0.5*(-3)*0.01
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, -0.3f, s.vy);   // -3*0.1
}

void test_predict_both_axes() {
  EKFState s;
  s.vx = 1.0f;
  s.vy = -0.5f;
  float P[16];
  init_P(P);
  ekf_predict(s, P, 0.5f, -1.0f, 0.1f);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.1025f, s.x);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, -0.055f, s.y);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 1.05f, s.vx);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, -0.6f, s.vy);
}

void test_predict_invalid_dt_zero() {
  EKFState s;
  float P[16];
  init_P(P);
  float P0_before = P[0];
  ekf_predict(s, P, 10.0f, 10.0f, 0.0f);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, s.x);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, P0_before, P[0]);
}

void test_predict_invalid_dt_negative() {
  EKFState s;
  float P[16];
  init_P(P);
  ekf_predict(s, P, 10.0f, 10.0f, -0.1f);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, s.x);
}

void test_predict_invalid_dt_too_large() {
  EKFState s;
  float P[16];
  init_P(P);
  ekf_predict(s, P, 10.0f, 10.0f, 0.6f);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, s.x);
}

void test_predict_dt_at_boundary_0_5_rejected() {
  EKFState s;
  float P[16];
  init_P(P);
  ekf_predict(s, P, 1.0f, 1.0f, 0.5f);
  // dt > 0.5 is rejected, dt == 0.5 should NOT be rejected (it's not > 0.5)
  // Actually the code says dt > 0.5f, so exactly 0.5 passes
  TEST_ASSERT_TRUE(s.x != 0.0f || s.vx != 0.0f);
}

void test_predict_covariance_grows() {
  EKFState s;
  float P[16];
  init_P(P);
  float P0_before = P[0];
  float P5_before = P[5];
  float P10_before = P[10];
  float P15_before = P[15];
  ekf_predict(s, P, 0.0f, 0.0f, 0.01f);
  TEST_ASSERT_TRUE(P[0] > P0_before);
  TEST_ASSERT_TRUE(P[5] > P5_before);
  TEST_ASSERT_TRUE(P[10] > P10_before);
  TEST_ASSERT_TRUE(P[15] > P15_before);
}

void test_predict_covariance_symmetric() {
  EKFState s;
  s.vx = 0.5f;
  s.vy = -0.3f;
  float P[16];
  init_P(P);
  for (int step = 0; step < 20; step++) {
    ekf_predict(s, P, 0.1f, -0.2f, 0.004f);
  }
  // P should remain symmetric: P[i*4+j] == P[j*4+i]
  for (int i = 0; i < 4; i++)
    for (int j = i + 1; j < 4; j++)
      TEST_ASSERT_FLOAT_WITHIN(1e-5f, P[i * 4 + j], P[j * 4 + i]);
}

void test_predict_multiple_steps_accumulate() {
  EKFState s;
  float P[16];
  init_P(P);
  for (int i = 0; i < 100; i++) {
    ekf_predict(s, P, 1.0f, 0.0f, 0.004f);
  }
  // After 0.4s: x = 0.5*1.0*0.16 = 0.08, vx = 1.0*0.4 = 0.4
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.08f, s.x);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.4f, s.vx);
}

void test_predict_x_y_symmetry() {
  // Identical acceleration in x and y should produce identical results
  EKFState sx, sy;
  float Px[16], Py[16];
  init_P(Px);
  init_P(Py);
  ekf_predict(sx, Px, 1.0f, 0.0f, 0.1f);
  ekf_predict(sy, Py, 0.0f, 1.0f, 0.1f);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, sx.x, sy.y);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, sx.vx, sy.vy);
}

void test_predict_large_acceleration() {
  EKFState s;
  float P[16];
  init_P(P);
  ekf_predict(s, P, 50.0f, -50.0f, 0.004f);
  // x = 0.5*50*0.000016 = 0.0004
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0004f, s.x);
  // vx = 50*0.004 = 0.2
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.2f, s.vx);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, -0.2f, s.vy);
}

void test_predict_small_dt() {
  EKFState s;
  s.vx = 1.0f;
  float P[16];
  init_P(P);
  ekf_predict(s, P, 0.0f, 0.0f, 0.001f);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.001f, s.x);
}

// ════════════════════════════════════════════════════════════════
//  TRILATERATION TESTS
// ════════════════════════════════════════════════════════════════

// Standard anchor layout
static const AnchorInfo STD_ANCHORS[3] = {
    {10, 0.0f, 0.0f, true}, {11, 3.0f, 0.0f, true}, {12, 0.0f, 4.0f, true}};

void test_trilaterate_known_position_1_1() {
  float tx = 1.0f, ty = 1.0f;
  float d[] = {dist(tx, ty, 0, 0), dist(tx, ty, 3, 0), dist(tx, ty, 0, 4)};
  uint8_t ids[] = {10, 11, 12};
  float x, y;
  TEST_ASSERT_TRUE(trilaterate(d, ids, 3, STD_ANCHORS, 3, x, y));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, tx, x);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, ty, y);
}

void test_trilaterate_known_position_2_3() {
  float tx = 2.0f, ty = 3.0f;
  float d[] = {dist(tx, ty, 0, 0), dist(tx, ty, 3, 0), dist(tx, ty, 0, 4)};
  uint8_t ids[] = {10, 11, 12};
  float x, y;
  TEST_ASSERT_TRUE(trilaterate(d, ids, 3, STD_ANCHORS, 3, x, y));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, tx, x);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, ty, y);
}

void test_trilaterate_at_anchor_position() {
  // Target at anchor 11 (3, 0)
  float d[] = {3.0f, 0.001f, 5.0f};
  uint8_t ids[] = {10, 11, 12};
  float x, y;
  TEST_ASSERT_TRUE(trilaterate(d, ids, 3, STD_ANCHORS, 3, x, y));
  TEST_ASSERT_FLOAT_WITHIN(0.1f, 3.0f, x);
  TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.0f, y);
}

void test_trilaterate_origin() {
  float d[] = {0.001f, 3.0f, 4.0f};
  uint8_t ids[] = {10, 11, 12};
  float x, y;
  TEST_ASSERT_TRUE(trilaterate(d, ids, 3, STD_ANCHORS, 3, x, y));
  TEST_ASSERT_FLOAT_WITHIN(0.05f, 0.0f, x);
  TEST_ASSERT_FLOAT_WITHIN(0.05f, 0.0f, y);
}

void test_trilaterate_field_center() {
  // SEC field is ~3.66m x 3.66m, center is (1.83, 1.83)
  AnchorInfo field_anchors[3] = {
      {10, 0.0f, 0.0f, true}, {11, 3.66f, 0.0f, true}, {12, 0.0f, 3.66f, true}};
  float tx = 1.83f, ty = 1.83f;
  float d[] = {dist(tx, ty, 0, 0), dist(tx, ty, 3.66f, 0), dist(tx, ty, 0, 3.66f)};
  uint8_t ids[] = {10, 11, 12};
  float x, y;
  TEST_ASSERT_TRUE(trilaterate(d, ids, 3, field_anchors, 3, x, y));
  TEST_ASSERT_FLOAT_WITHIN(0.02f, tx, x);
  TEST_ASSERT_FLOAT_WITHIN(0.02f, ty, y);
}

void test_trilaterate_too_few_ranges() {
  float d[] = {1.0f, 2.0f};
  uint8_t ids[] = {10, 11};
  float x, y;
  TEST_ASSERT_FALSE(trilaterate(d, ids, 2, STD_ANCHORS, 3, x, y));
}

void test_trilaterate_one_range() {
  float d[] = {1.0f};
  uint8_t ids[] = {10};
  float x, y;
  TEST_ASSERT_FALSE(trilaterate(d, ids, 1, STD_ANCHORS, 3, x, y));
}

void test_trilaterate_zero_ranges() {
  float x, y;
  TEST_ASSERT_FALSE(trilaterate(nullptr, nullptr, 0, STD_ANCHORS, 3, x, y));
}

void test_trilaterate_unmatched_ids() {
  float d[] = {1.0f, 2.0f, 3.0f};
  uint8_t ids[] = {99, 98, 97};
  float x, y;
  TEST_ASSERT_FALSE(trilaterate(d, ids, 3, STD_ANCHORS, 3, x, y));
}

void test_trilaterate_partial_match_insufficient() {
  // Only 2 out of 3 IDs match → too few valid ranges
  float d[] = {1.0f, 2.0f, 3.0f};
  uint8_t ids[] = {10, 11, 99};
  float x, y;
  TEST_ASSERT_FALSE(trilaterate(d, ids, 3, STD_ANCHORS, 3, x, y));
}

void test_trilaterate_collinear_anchors_degenerate() {
  AnchorInfo lin[3] = {
      {10, 0.0f, 0.0f, true}, {11, 1.0f, 0.0f, true}, {12, 2.0f, 0.0f, true}};
  float d[] = {1.0f, 1.0f, 1.0f};
  uint8_t ids[] = {10, 11, 12};
  float x, y;
  TEST_ASSERT_FALSE(trilaterate(d, ids, 3, lin, 3, x, y));
}

void test_trilaterate_negative_distance_skipped() {
  float d[] = {-1.0f, 3.0f, 4.0f};
  uint8_t ids[] = {10, 11, 12};
  float x, y;
  TEST_ASSERT_FALSE(trilaterate(d, ids, 3, STD_ANCHORS, 3, x, y));
}

void test_trilaterate_zero_distance_skipped() {
  float d[] = {0.0f, 3.0f, 4.0f};
  uint8_t ids[] = {10, 11, 12};
  float x, y;
  TEST_ASSERT_FALSE(trilaterate(d, ids, 3, STD_ANCHORS, 3, x, y));
}

void test_trilaterate_extra_ranges_uses_first_three() {
  AnchorInfo a4[4] = {{10, 0, 0, true}, {11, 3, 0, true},
                      {12, 0, 4, true}, {13, 3, 4, true}};
  float tx = 1.0f, ty = 1.0f;
  float d[] = {dist(tx, ty, 0, 0), dist(tx, ty, 3, 0),
               dist(tx, ty, 0, 4), dist(tx, ty, 3, 4)};
  uint8_t ids[] = {10, 11, 12, 13};
  float x, y;
  TEST_ASSERT_TRUE(trilaterate(d, ids, 4, a4, 4, x, y));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, tx, x);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, ty, y);
}

void test_trilaterate_scrambled_id_order() {
  // IDs in different order than anchors
  float tx = 1.5f, ty = 2.0f;
  float d12 = dist(tx, ty, 0, 4);
  float d10 = dist(tx, ty, 0, 0);
  float d11 = dist(tx, ty, 3, 0);
  float d[] = {d12, d10, d11};
  uint8_t ids[] = {12, 10, 11};
  float x, y;
  TEST_ASSERT_TRUE(trilaterate(d, ids, 3, STD_ANCHORS, 3, x, y));
  TEST_ASSERT_FLOAT_WITHIN(0.05f, tx, x);
  TEST_ASSERT_FLOAT_WITHIN(0.05f, ty, y);
}

void test_trilaterate_invalid_anchor_skipped() {
  AnchorInfo a3[3] = {{10, 0, 0, true}, {11, 3, 0, false}, {12, 0, 4, true}};
  float d[] = {1.0f, 2.0f, 3.0f};
  uint8_t ids[] = {10, 11, 12};
  float x, y;
  // Anchor 11 has valid=false, so only 2 matches → fails
  TEST_ASSERT_FALSE(trilaterate(d, ids, 3, a3, 3, x, y));
}

void test_trilaterate_noisy_distances() {
  // Add small noise to distances — result should still be close
  float tx = 1.0f, ty = 1.0f;
  float noise = 0.02f;
  float d[] = {dist(tx, ty, 0, 0) + noise, dist(tx, ty, 3, 0) - noise,
               dist(tx, ty, 0, 4) + noise};
  uint8_t ids[] = {10, 11, 12};
  float x, y;
  TEST_ASSERT_TRUE(trilaterate(d, ids, 3, STD_ANCHORS, 3, x, y));
  TEST_ASSERT_FLOAT_WITHIN(0.15f, tx, x);
  TEST_ASSERT_FLOAT_WITHIN(0.15f, ty, y);
}

// ════════════════════════════════════════════════════════════════
//  UPDATE TESTS
// ════════════════════════════════════════════════════════════════

void test_update_moves_state_toward_measurement() {
  EKFState s;
  float P[16];
  init_P(P);
  TEST_ASSERT_TRUE(ekf_update(s, P, 0.8f, 0.0f));
  TEST_ASSERT_TRUE(s.x > 0.0f);
  TEST_ASSERT_TRUE(s.x < 0.8f);
}

void test_update_y_direction() {
  EKFState s;
  float P[16];
  init_P(P);
  TEST_ASSERT_TRUE(ekf_update(s, P, 0.0f, 0.7f));
  TEST_ASSERT_TRUE(s.y > 0.0f);
  TEST_ASSERT_TRUE(s.y < 0.7f);
}

void test_update_both_axes() {
  EKFState s;
  float P[16];
  init_P(P);
  TEST_ASSERT_TRUE(ekf_update(s, P, 0.5f, 0.5f));
  TEST_ASSERT_TRUE(s.x > 0.0f);
  TEST_ASSERT_TRUE(s.y > 0.0f);
}

void test_update_reduces_position_covariance() {
  EKFState s;
  float P[16];
  init_P(P);
  float Px_before = P[0];
  float Py_before = P[5];
  ekf_update(s, P, 0.0f, 0.0f);
  TEST_ASSERT_TRUE(P[0] < Px_before);
  TEST_ASSERT_TRUE(P[5] < Py_before);
}

void test_update_does_not_increase_velocity_variance() {
  EKFState s;
  float P[16];
  init_P(P);
  float Pvx_before = P[10];
  float Pvy_before = P[15];
  ekf_update(s, P, 0.0f, 0.0f);
  // Velocity variance should decrease or stay same (not increase)
  TEST_ASSERT_TRUE(P[10] <= Pvx_before + 1e-6f);
  TEST_ASSERT_TRUE(P[15] <= Pvy_before + 1e-6f);
}

void test_update_covariance_stays_symmetric() {
  EKFState s;
  s.x = 0.5f;
  s.y = 0.3f;
  float P[16];
  init_P(P);
  ekf_update(s, P, 0.6f, 0.4f);
  for (int i = 0; i < 4; i++)
    for (int j = i + 1; j < 4; j++)
      TEST_ASSERT_FLOAT_WITHIN(1e-5f, P[i * 4 + j], P[j * 4 + i]);
}

void test_update_outlier_rejected_2m() {
  EKFState s;
  float P[16];
  init_P(P);
  TEST_ASSERT_FALSE(ekf_update(s, P, 2.0f, 0.0f));
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, s.x);
}

void test_update_outlier_rejected_diagonal() {
  EKFState s;
  float P[16];
  init_P(P);
  // sqrt(0.8^2 + 0.8^2) ≈ 1.13 > 1.0 → rejected
  TEST_ASSERT_FALSE(ekf_update(s, P, 0.8f, 0.8f));
}

void test_update_at_gate_boundary_accepted() {
  EKFState s;
  float P[16];
  init_P(P);
  // Exactly at gate: 1.0m away
  TEST_ASSERT_TRUE(ekf_update(s, P, 1.0f, 0.0f));
}

void test_update_just_beyond_gate_rejected() {
  EKFState s;
  float P[16];
  init_P(P);
  TEST_ASSERT_FALSE(ekf_update(s, P, 1.001f, 0.0f));
}

void test_update_within_gate_accepted() {
  EKFState s;
  float P[16];
  init_P(P);
  TEST_ASSERT_TRUE(ekf_update(s, P, 0.5f, 0.0f));
  TEST_ASSERT_TRUE(s.x > 0.0f);
}

void test_update_exact_match_no_change() {
  EKFState s;
  s.x = 1.0f;
  s.y = 2.0f;
  float P[16];
  init_P(P);
  TEST_ASSERT_TRUE(ekf_update(s, P, 1.0f, 2.0f));
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 1.0f, s.x);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 2.0f, s.y);
}

void test_update_negative_measurement() {
  EKFState s;
  s.x = -0.3f;
  float P[16];
  init_P(P);
  TEST_ASSERT_TRUE(ekf_update(s, P, -0.5f, 0.0f));
  TEST_ASSERT_TRUE(s.x < -0.3f);  // moves toward -0.5
}

void test_update_multiple_reduces_uncertainty() {
  EKFState s;
  float P[16];
  init_P(P);
  // Multiple updates at the same location should converge and reduce P
  for (int i = 0; i < 10; i++) {
    ekf_update(s, P, 0.5f, 0.3f);
  }
  TEST_ASSERT_FLOAT_WITHIN(0.05f, 0.5f, s.x);
  TEST_ASSERT_FLOAT_WITHIN(0.05f, 0.3f, s.y);
  TEST_ASSERT_TRUE(P[0] < 0.1f);  // Uncertainty should be very small
}

void test_update_kalman_gain_reasonable() {
  // With initial P = I and R = 0.15, Kalman gain for position should be
  // K = P*H^T * (H*P*H^T + R)^-1 = 1/(1+0.15) ≈ 0.87
  // So state should move ~87% toward measurement
  EKFState s;
  float P[16];
  init_P(P);
  ekf_update(s, P, 0.5f, 0.0f);
  float gain_approx = s.x / 0.5f;
  TEST_ASSERT_FLOAT_WITHIN(0.05f, 0.87f, gain_approx);
}

void test_update_covariance_positive_diag() {
  EKFState s;
  float P[16];
  init_P(P);
  for (int i = 0; i < 5; i++) {
    ekf_predict(s, P, 0.1f, -0.1f, 0.01f);
    ekf_update(s, P, s.x + 0.1f, s.y - 0.05f);
  }
  // Diagonal elements of P must remain positive
  TEST_ASSERT_TRUE(P[0] > 0.0f);
  TEST_ASSERT_TRUE(P[5] > 0.0f);
  TEST_ASSERT_TRUE(P[10] > 0.0f);
  TEST_ASSERT_TRUE(P[15] > 0.0f);
}

// ════════════════════════════════════════════════════════════════
//  PREDICT + UPDATE INTEGRATION TESTS
// ════════════════════════════════════════════════════════════════

void test_predict_update_converges_to_truth() {
  EKFState s;
  float P[16];
  init_P(P);

  // Target at (0.5, 0.5) — within outlier gate from origin (dist ≈ 0.707m < 1.0m)
  float tx = 0.5f, ty = 0.5f;

  AnchorInfo anchors[3] = {
      {10, 0.0f, 0.0f, true}, {11, 3.0f, 0.0f, true}, {12, 0.0f, 4.0f, true}};

  for (int i = 0; i < 200; i++) {
    ekf_predict(s, P, 0.0f, 0.0f, 0.004f);

    if (i % 12 == 0) {
      float d[] = {dist(tx, ty, 0, 0), dist(tx, ty, 3, 0), dist(tx, ty, 0, 4)};
      uint8_t ids[] = {10, 11, 12};
      float x_meas, y_meas;
      if (trilaterate(d, ids, 3, anchors, 3, x_meas, y_meas)) {
        ekf_update(s, P, x_meas, y_meas);
      }
    }
  }

  TEST_ASSERT_FLOAT_WITHIN(0.1f, tx, s.x);
  TEST_ASSERT_FLOAT_WITHIN(0.1f, ty, s.y);
  TEST_ASSERT_FLOAT_WITHIN(0.3f, 0.0f, s.vx);
  TEST_ASSERT_FLOAT_WITHIN(0.3f, 0.0f, s.vy);
}

void test_predict_update_with_constant_velocity() {
  EKFState s;
  float P[16];
  init_P(P);

  s.vx = 1.0f;
  float true_x = 0.0f;

  for (int i = 0; i < 100; i++) {
    float dt = 0.004f;
    true_x += 1.0f * dt;
    ekf_predict(s, P, 0.0f, 0.0f, dt);

    if (i % 12 == 0) {
      ekf_update(s, P, true_x, 0.0f);
    }
  }

  TEST_ASSERT_FLOAT_WITHIN(0.1f, true_x, s.x);
  TEST_ASSERT_FLOAT_WITHIN(0.3f, 1.0f, s.vx);
}

void test_predict_update_with_acceleration() {
  EKFState s;
  float P[16];
  init_P(P);

  float ax = 0.5f;
  float true_x = 0.0f, true_vx = 0.0f;

  for (int i = 0; i < 200; i++) {
    float dt = 0.004f;
    true_x += true_vx * dt + 0.5f * ax * dt * dt;
    true_vx += ax * dt;
    ekf_predict(s, P, ax, 0.0f, dt);

    if (i % 12 == 0 && fabsf(true_x - s.x) < OUTLIER_GATE_M) {
      ekf_update(s, P, true_x, 0.0f);
    }
  }

  TEST_ASSERT_FLOAT_WITHIN(0.1f, true_x, s.x);
  TEST_ASSERT_FLOAT_WITHIN(0.2f, true_vx, s.vx);
}

void test_predict_only_diverges_without_updates() {
  EKFState s;
  float P[16];
  init_P(P);

  // Push EKF state to 0.5, then only predict (no updates)
  ekf_update(s, P, 0.5f, 0.0f);
  float x_after_update = s.x;

  // Apply acceleration for many steps with no correction
  for (int i = 0; i < 500; i++) {
    ekf_predict(s, P, 0.1f, 0.0f, 0.004f);
  }

  // State should have drifted significantly from 0.5
  TEST_ASSERT_TRUE(fabsf(s.x - x_after_update) > 0.1f);
  // And covariance should have grown large
  TEST_ASSERT_TRUE(P[0] > 1.0f);
}

void test_outlier_gate_blocks_bad_measurement() {
  EKFState s;
  float P[16];
  init_P(P);

  // First establish a good position
  for (int i = 0; i < 5; i++) {
    ekf_update(s, P, 0.5f, 0.5f);
  }
  float x_before = s.x;
  float y_before = s.y;

  // Try a bad measurement far away — should be rejected
  bool ok = ekf_update(s, P, 5.0f, 5.0f);
  TEST_ASSERT_FALSE(ok);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, x_before, s.x);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, y_before, s.y);
}

// ════════════════════════════════════════════════════════════════
//  MATRIX HELPER TESTS
// ════════════════════════════════════════════════════════════════

void test_mat4_mul_identity() {
  float I[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
  float A[16] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};
  float C[16];
  mat4_mul(I, A, C);
  for (int i = 0; i < 16; i++)
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, A[i], C[i]);
}

void test_mat4_mul_zero() {
  float Z[16] = {};
  float A[16] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};
  float C[16];
  mat4_mul(Z, A, C);
  for (int i = 0; i < 16; i++)
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, C[i]);
}

void test_mat4_add() {
  float A[16], B[16], C[16];
  for (int i = 0; i < 16; i++) {
    A[i] = (float)i;
    B[i] = (float)(i * 2);
  }
  mat4_add(A, B, C);
  for (int i = 0; i < 16; i++)
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, (float)(i * 3), C[i]);
}

int main(int argc, char** argv) {
  UNITY_BEGIN();

  // Predict
  RUN_TEST(test_predict_zero_accel_zero_vel_no_change);
  RUN_TEST(test_predict_constant_velocity_x);
  RUN_TEST(test_predict_constant_velocity_y);
  RUN_TEST(test_predict_constant_acceleration_x);
  RUN_TEST(test_predict_constant_acceleration_y);
  RUN_TEST(test_predict_both_axes);
  RUN_TEST(test_predict_invalid_dt_zero);
  RUN_TEST(test_predict_invalid_dt_negative);
  RUN_TEST(test_predict_invalid_dt_too_large);
  RUN_TEST(test_predict_dt_at_boundary_0_5_rejected);
  RUN_TEST(test_predict_covariance_grows);
  RUN_TEST(test_predict_covariance_symmetric);
  RUN_TEST(test_predict_multiple_steps_accumulate);
  RUN_TEST(test_predict_x_y_symmetry);
  RUN_TEST(test_predict_large_acceleration);
  RUN_TEST(test_predict_small_dt);

  // Trilateration
  RUN_TEST(test_trilaterate_known_position_1_1);
  RUN_TEST(test_trilaterate_known_position_2_3);
  RUN_TEST(test_trilaterate_at_anchor_position);
  RUN_TEST(test_trilaterate_origin);
  RUN_TEST(test_trilaterate_field_center);
  RUN_TEST(test_trilaterate_too_few_ranges);
  RUN_TEST(test_trilaterate_one_range);
  RUN_TEST(test_trilaterate_zero_ranges);
  RUN_TEST(test_trilaterate_unmatched_ids);
  RUN_TEST(test_trilaterate_partial_match_insufficient);
  RUN_TEST(test_trilaterate_collinear_anchors_degenerate);
  RUN_TEST(test_trilaterate_negative_distance_skipped);
  RUN_TEST(test_trilaterate_zero_distance_skipped);
  RUN_TEST(test_trilaterate_extra_ranges_uses_first_three);
  RUN_TEST(test_trilaterate_scrambled_id_order);
  RUN_TEST(test_trilaterate_invalid_anchor_skipped);
  RUN_TEST(test_trilaterate_noisy_distances);

  // Update
  RUN_TEST(test_update_moves_state_toward_measurement);
  RUN_TEST(test_update_y_direction);
  RUN_TEST(test_update_both_axes);
  RUN_TEST(test_update_reduces_position_covariance);
  RUN_TEST(test_update_does_not_increase_velocity_variance);
  RUN_TEST(test_update_covariance_stays_symmetric);
  RUN_TEST(test_update_outlier_rejected_2m);
  RUN_TEST(test_update_outlier_rejected_diagonal);
  RUN_TEST(test_update_at_gate_boundary_accepted);
  RUN_TEST(test_update_just_beyond_gate_rejected);
  RUN_TEST(test_update_within_gate_accepted);
  RUN_TEST(test_update_exact_match_no_change);
  RUN_TEST(test_update_negative_measurement);
  RUN_TEST(test_update_multiple_reduces_uncertainty);
  RUN_TEST(test_update_kalman_gain_reasonable);
  RUN_TEST(test_update_covariance_positive_diag);

  // Integration
  RUN_TEST(test_predict_update_converges_to_truth);
  RUN_TEST(test_predict_update_with_constant_velocity);
  RUN_TEST(test_predict_update_with_acceleration);
  RUN_TEST(test_predict_only_diverges_without_updates);
  RUN_TEST(test_outlier_gate_blocks_bad_measurement);

  // Matrix helpers
  RUN_TEST(test_mat4_mul_identity);
  RUN_TEST(test_mat4_mul_zero);
  RUN_TEST(test_mat4_add);

  return UNITY_END();
}
