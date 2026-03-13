/**
 * @file test_drone_ekf.cpp
 * @brief Native unit tests for drone EKF: predict, per-range update, integration.
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
static constexpr float MAHALANOBIS_GATE_SQ = 9.0f;
static constexpr float COV_MAX_POS = 10.0f;
static constexpr float COV_MAX_VEL = 5.0f;
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

// ── Covariance bounding (from DroneEKFSubsystem.cpp) ──

static void boundCovariance(float P[16]) {
  if (P[0] > COV_MAX_POS) P[0] = COV_MAX_POS;
  if (P[5] > COV_MAX_POS) P[5] = COV_MAX_POS;
  if (P[10] > COV_MAX_VEL) P[10] = COV_MAX_VEL;
  if (P[15] > COV_MAX_VEL) P[15] = COV_MAX_VEL;

  for (int i = 0; i < 4; i++)
    for (int j = i + 1; j < 4; j++) {
      float avg = 0.5f * (P[i * 4 + j] + P[j * 4 + i]);
      P[i * 4 + j] = avg;
      P[j * 4 + i] = avg;
    }
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

  boundCovariance(P);
}

// ── Per-range scalar Kalman update (from DroneEKFSubsystem::updateRange) ──

static bool ekf_update_range(EKFState& state, float P[16], float distance_m,
                              float anchor_x, float anchor_y) {
  float dx = state.x - anchor_x;
  float dy = state.y - anchor_y;
  float predicted_dist = sqrtf(dx * dx + dy * dy);
  if (predicted_dist < 1e-4f) predicted_dist = 1e-4f;

  float H[4];
  H[0] = dx / predicted_dist;
  H[1] = dy / predicted_dist;
  H[2] = 0.0f;
  H[3] = 0.0f;

  float innovation = distance_m - predicted_dist;

  // S = H*P*H^T + R
  float S = MEASURE_NOISE_UWB;
  for (int i = 0; i < 4; i++)
    for (int j = 0; j < 4; j++)
      S += H[i] * P[i * 4 + j] * H[j];

  if (S < 1e-10f) return false;
  float mahal_sq = (innovation * innovation) / S;
  if (mahal_sq > MAHALANOBIS_GATE_SQ) return false;

  // K = P*H^T / S
  float K[4];
  for (int i = 0; i < 4; i++) {
    K[i] = 0.0f;
    for (int j = 0; j < 4; j++)
      K[i] += P[i * 4 + j] * H[j];
    K[i] /= S;
  }

  state.x += K[0] * innovation;
  state.y += K[1] * innovation;
  state.vx += K[2] * innovation;
  state.vy += K[3] * innovation;

  // Joseph form: P = (I-KH)*P*(I-KH)^T + K*R*K^T
  float IKH[16];
  for (int i = 0; i < 4; i++)
    for (int j = 0; j < 4; j++)
      IKH[i * 4 + j] = ((i == j) ? 1.0f : 0.0f) - K[i] * H[j];

  float IKHT[16];
  for (int i = 0; i < 4; i++)
    for (int j = 0; j < 4; j++)
      IKHT[i * 4 + j] = IKH[j * 4 + i];

  float temp[16], P_new[16];
  mat4_mul(IKH, P, temp);
  mat4_mul(temp, IKHT, P_new);

  float R = MEASURE_NOISE_UWB;
  for (int i = 0; i < 4; i++)
    for (int j = 0; j < 4; j++)
      P_new[i * 4 + j] += K[i] * R * K[j];

  memcpy(P, P_new, sizeof(float) * 16);
  return true;
}

// ── Multi-range update (from DroneEKFSubsystem::updateUWB) ──

static uint8_t ekf_update_uwb(EKFState& state, float P[16],
                                const float* distances, const uint8_t* ids,
                                uint8_t num_ranges, const AnchorInfo* anchors,
                                uint8_t num_anchors) {
  uint8_t fused = 0;
  for (uint8_t i = 0; i < num_ranges; i++) {
    if (distances[i] <= 0.0f) continue;
    for (uint8_t j = 0; j < num_anchors; j++) {
      if (anchors[j].valid && anchors[j].id == ids[i]) {
        if (ekf_update_range(state, P, distances[i], anchors[j].x,
                             anchors[j].y)) {
          fused++;
        }
        break;
      }
    }
  }
  return fused;
}

// ── Helpers ──

static void init_P(float P[16]) {
  memset(P, 0, sizeof(float) * 16);
  P[0] = 1.0f;
  P[5] = 1.0f;
  P[10] = 0.5f;
  P[15] = 0.5f;
}

static float dist(float x, float y, float ax, float ay) {
  return sqrtf((x - ax) * (x - ax) + (y - ay) * (y - ay));
}

static bool is_symmetric(const float P[16], float tol = 1e-5f) {
  for (int i = 0; i < 4; i++)
    for (int j = i + 1; j < 4; j++)
      if (fabsf(P[i * 4 + j] - P[j * 4 + i]) > tol) return false;
  return true;
}

static bool diag_positive(const float P[16]) {
  return P[0] > 0.0f && P[5] > 0.0f && P[10] > 0.0f && P[15] > 0.0f;
}

void setUp(void) {}
void tearDown(void) {}

// Standard anchor layout
static const AnchorInfo STD_ANCHORS[3] = {
    {10, 0.0f, 0.0f, true}, {11, 3.0f, 0.0f, true}, {12, 0.0f, 4.0f, true}};

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
}

void test_predict_constant_velocity_y() {
  EKFState s;
  s.vy = 2.0f;
  float P[16];
  init_P(P);
  ekf_predict(s, P, 0.0f, 0.0f, 0.1f);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.2f, s.y);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 2.0f, s.vy);
}

void test_predict_constant_acceleration_x() {
  EKFState s;
  float P[16];
  init_P(P);
  ekf_predict(s, P, 2.0f, 0.0f, 0.1f);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.01f, s.x);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.2f, s.vx);
}

void test_predict_constant_acceleration_y() {
  EKFState s;
  float P[16];
  init_P(P);
  ekf_predict(s, P, 0.0f, -3.0f, 0.1f);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, -0.015f, s.y);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, -0.3f, s.vy);
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

void test_predict_dt_at_boundary_0_5_passes() {
  EKFState s;
  float P[16];
  init_P(P);
  ekf_predict(s, P, 1.0f, 1.0f, 0.5f);
  // dt == 0.5 is not > 0.5, so it passes
  TEST_ASSERT_TRUE(s.x != 0.0f || s.vx != 0.0f);
}

void test_predict_covariance_grows() {
  EKFState s;
  float P[16];
  init_P(P);
  float P0_before = P[0];
  float P10_before = P[10];
  ekf_predict(s, P, 0.0f, 0.0f, 0.01f);
  TEST_ASSERT_TRUE(P[0] > P0_before);
  TEST_ASSERT_TRUE(P[10] > P10_before);
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
  TEST_ASSERT_TRUE(is_symmetric(P));
}

void test_predict_multiple_steps_accumulate() {
  EKFState s;
  float P[16];
  init_P(P);
  for (int i = 0; i < 100; i++) {
    ekf_predict(s, P, 1.0f, 0.0f, 0.004f);
  }
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.08f, s.x);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.4f, s.vx);
}

void test_predict_x_y_symmetry() {
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
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0004f, s.x);
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
//  COVARIANCE BOUNDING TESTS
// ════════════════════════════════════════════════════════════════

void test_covariance_bounded_after_many_predicts() {
  EKFState s;
  float P[16];
  init_P(P);
  // Predict for a long time without updates — P should be clamped
  for (int i = 0; i < 10000; i++) {
    ekf_predict(s, P, 0.0f, 0.0f, 0.004f);
  }
  TEST_ASSERT_TRUE(P[0] <= COV_MAX_POS + 1e-6f);
  TEST_ASSERT_TRUE(P[5] <= COV_MAX_POS + 1e-6f);
  TEST_ASSERT_TRUE(P[10] <= COV_MAX_VEL + 1e-6f);
  TEST_ASSERT_TRUE(P[15] <= COV_MAX_VEL + 1e-6f);
}

void test_covariance_symmetry_enforced() {
  float P[16];
  init_P(P);
  // Manually break symmetry
  P[1] = 0.1f;
  P[4] = 0.2f;
  boundCovariance(P);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, P[1], P[4]);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.15f, P[1]);
}

// ════════════════════════════════════════════════════════════════
//  PER-RANGE UPDATE TESTS
// ════════════════════════════════════════════════════════════════

void test_update_range_moves_toward_anchor() {
  // State at origin, anchor at (3,0), true distance = 3.0
  // Measured distance = 2.5 → drone is closer than predicted → state moves right
  EKFState s;
  float P[16];
  init_P(P);
  TEST_ASSERT_TRUE(ekf_update_range(s, P, 2.5f, 3.0f, 0.0f));
  TEST_ASSERT_TRUE(s.x > 0.0f);  // moved toward anchor
}

void test_update_range_moves_away_from_anchor() {
  // State at origin, anchor at (3,0), measured distance = 3.5 → drone farther
  EKFState s;
  float P[16];
  init_P(P);
  TEST_ASSERT_TRUE(ekf_update_range(s, P, 3.5f, 3.0f, 0.0f));
  TEST_ASSERT_TRUE(s.x < 0.0f);  // moved away from anchor
}

void test_update_range_reduces_position_covariance() {
  EKFState s;
  s.x = 1.0f;
  s.y = 1.0f;
  float P[16];
  init_P(P);
  float Px_before = P[0];
  ekf_update_range(s, P, dist(1, 1, 0, 0), 0.0f, 0.0f);
  TEST_ASSERT_TRUE(P[0] < Px_before);
}

void test_update_range_covariance_stays_symmetric() {
  EKFState s;
  s.x = 1.0f;
  s.y = 0.5f;
  float P[16];
  init_P(P);
  ekf_update_range(s, P, dist(1, 0.5f, 3, 0), 3.0f, 0.0f);
  TEST_ASSERT_TRUE(is_symmetric(P));
}

void test_update_range_covariance_stays_positive_diagonal() {
  EKFState s;
  s.x = 1.0f;
  s.y = 1.0f;
  float P[16];
  init_P(P);
  // Multiple updates from different anchors
  for (int i = 0; i < 5; i++) {
    ekf_update_range(s, P, dist(s.x, s.y, 0, 0) + 0.05f, 0.0f, 0.0f);
    ekf_update_range(s, P, dist(s.x, s.y, 3, 0) - 0.03f, 3.0f, 0.0f);
    ekf_update_range(s, P, dist(s.x, s.y, 0, 4) + 0.02f, 0.0f, 4.0f);
  }
  TEST_ASSERT_TRUE(diag_positive(P));
}

void test_update_range_exact_distance_no_state_change() {
  EKFState s;
  s.x = 1.0f;
  s.y = 1.0f;
  float P[16];
  init_P(P);
  float d = dist(1, 1, 3, 0);
  ekf_update_range(s, P, d, 3.0f, 0.0f);
  // Innovation is zero → no state change
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 1.0f, s.x);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 1.0f, s.y);
}

void test_update_range_mahalanobis_gate_rejects_outlier() {
  EKFState s;
  float P[16];
  init_P(P);
  // State at origin, anchor at (3,0), predicted distance = 3
  // Measured distance = 100 → huge innovation → gated out
  TEST_ASSERT_FALSE(ekf_update_range(s, P, 100.0f, 3.0f, 0.0f));
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, s.x);
}

void test_update_range_mahalanobis_adapts_to_uncertainty() {
  // With large P (high uncertainty), same residual should pass gate
  EKFState s;
  float P[16];
  init_P(P);
  P[0] = 100.0f;  // very uncertain
  P[5] = 100.0f;
  // Large innovation that would fail with small P
  TEST_ASSERT_TRUE(ekf_update_range(s, P, 10.0f, 3.0f, 0.0f));
}

void test_update_range_single_range_partial_correction() {
  // With only 1 range, we can still update — it constrains the state
  // along the radial direction to/from that anchor
  EKFState s;
  float P[16];
  init_P(P);
  ekf_update_range(s, P, 2.0f, 3.0f, 0.0f);
  // State should move toward x=3 (anchor direction)
  TEST_ASSERT_TRUE(s.x > 0.0f);
}

void test_update_range_state_at_anchor_handled() {
  // Edge case: state exactly at anchor position
  EKFState s;
  s.x = 3.0f;
  s.y = 0.0f;
  float P[16];
  init_P(P);
  // Distance should be ~0 but measured as 0.5 → should still work
  bool ok = ekf_update_range(s, P, 0.5f, 3.0f, 0.0f);
  // Should handle gracefully (predicted_dist clamped to 1e-4)
  TEST_ASSERT_TRUE(ok || !ok);  // just don't crash
  TEST_ASSERT_TRUE(diag_positive(P));
}

// ════════════════════════════════════════════════════════════════
//  MULTI-RANGE UPDATE TESTS
// ════════════════════════════════════════════════════════════════

void test_multi_range_three_anchors_converges() {
  EKFState s;
  float P[16];
  init_P(P);

  float tx = 1.0f, ty = 1.0f;
  float d[] = {dist(tx, ty, 0, 0), dist(tx, ty, 3, 0), dist(tx, ty, 0, 4)};
  uint8_t ids[] = {10, 11, 12};

  // Multiple rounds of updates
  for (int i = 0; i < 20; i++) {
    ekf_update_uwb(s, P, d, ids, 3, STD_ANCHORS, 3);
  }
  TEST_ASSERT_FLOAT_WITHIN(0.1f, tx, s.x);
  TEST_ASSERT_FLOAT_WITHIN(0.1f, ty, s.y);
}

void test_multi_range_single_range_works() {
  EKFState s;
  s.x = 2.0f; // Offset from anchor so Jacobian is well-conditioned
  float P[16];
  init_P(P);
  float d[] = {3.0f};
  uint8_t ids[] = {10};
  uint8_t fused = ekf_update_uwb(s, P, d, ids, 1, STD_ANCHORS, 3);
  TEST_ASSERT_EQUAL(1, fused);
}

void test_multi_range_two_ranges_works() {
  EKFState s;
  s.x = 1.0f; // Start near true position to avoid Mahalanobis gate rejection
  s.y = 1.0f;
  float P[16];
  init_P(P);
  float tx = 1.0f, ty = 1.0f;
  float d[] = {dist(tx, ty, 0, 0), dist(tx, ty, 3, 0)};
  uint8_t ids[] = {10, 11};
  uint8_t fused = ekf_update_uwb(s, P, d, ids, 2, STD_ANCHORS, 3);
  TEST_ASSERT_EQUAL(2, fused);
}

void test_multi_range_unmatched_ids_zero_fused() {
  EKFState s;
  float P[16];
  init_P(P);
  float d[] = {1.0f, 2.0f};
  uint8_t ids[] = {99, 98};
  uint8_t fused = ekf_update_uwb(s, P, d, ids, 2, STD_ANCHORS, 3);
  TEST_ASSERT_EQUAL(0, fused);
}

void test_multi_range_negative_distance_skipped() {
  EKFState s;
  float P[16];
  init_P(P);
  float d[] = {-1.0f, 3.0f, 4.0f};
  uint8_t ids[] = {10, 11, 12};
  // First range negative → skipped, other two should fuse
  uint8_t fused = ekf_update_uwb(s, P, d, ids, 3, STD_ANCHORS, 3);
  TEST_ASSERT_EQUAL(2, fused);
}

void test_multi_range_scrambled_ids() {
  EKFState s;
  float P[16];
  init_P(P);
  float tx = 1.5f, ty = 2.0f;
  float d[] = {dist(tx, ty, 0, 4), dist(tx, ty, 0, 0), dist(tx, ty, 3, 0)};
  uint8_t ids[] = {12, 10, 11};
  for (int i = 0; i < 20; i++) {
    ekf_update_uwb(s, P, d, ids, 3, STD_ANCHORS, 3);
  }
  TEST_ASSERT_FLOAT_WITHIN(0.15f, tx, s.x);
  TEST_ASSERT_FLOAT_WITHIN(0.15f, ty, s.y);
}

// ════════════════════════════════════════════════════════════════
//  JOSEPH FORM SPECIFIC TESTS
// ════════════════════════════════════════════════════════════════

void test_joseph_form_symmetry_after_many_updates() {
  EKFState s;
  s.x = 1.0f;
  s.y = 1.0f;
  float P[16];
  init_P(P);

  for (int i = 0; i < 50; i++) {
    ekf_predict(s, P, 0.01f, -0.01f, 0.004f);
    if (i % 5 == 0) {
      ekf_update_range(s, P, dist(s.x, s.y, 0, 0) + 0.02f, 0.0f, 0.0f);
      ekf_update_range(s, P, dist(s.x, s.y, 3, 0) - 0.01f, 3.0f, 0.0f);
      ekf_update_range(s, P, dist(s.x, s.y, 0, 4) + 0.01f, 0.0f, 4.0f);
    }
  }
  TEST_ASSERT_TRUE(is_symmetric(P));
  TEST_ASSERT_TRUE(diag_positive(P));
}

void test_joseph_form_no_negative_diagonal_under_stress() {
  // Aggressive updates that might cause negative P with simple form
  EKFState s;
  s.x = 2.0f;
  s.y = 2.0f;
  float P[16];
  init_P(P);
  // Make P very small to stress the update
  P[0] = 0.001f;
  P[5] = 0.001f;
  P[10] = 0.001f;
  P[15] = 0.001f;

  for (int i = 0; i < 20; i++) {
    ekf_update_range(s, P, dist(s.x, s.y, 0, 0) + 0.1f, 0.0f, 0.0f);
    ekf_update_range(s, P, dist(s.x, s.y, 3, 0) - 0.1f, 3.0f, 0.0f);
  }
  TEST_ASSERT_TRUE(diag_positive(P));
}

// ════════════════════════════════════════════════════════════════
//  PREDICT + UPDATE INTEGRATION TESTS
// ════════════════════════════════════════════════════════════════

void test_integration_stationary_converges() {
  EKFState s;
  float P[16];
  init_P(P);

  float tx = 1.0f, ty = 1.5f;

  for (int i = 0; i < 200; i++) {
    ekf_predict(s, P, 0.0f, 0.0f, 0.004f);

    if (i % 12 == 0) {
      float d[] = {dist(tx, ty, 0, 0), dist(tx, ty, 3, 0),
                   dist(tx, ty, 0, 4)};
      uint8_t ids[] = {10, 11, 12};
      ekf_update_uwb(s, P, d, ids, 3, STD_ANCHORS, 3);
    }
  }

  TEST_ASSERT_FLOAT_WITHIN(0.15f, tx, s.x);
  TEST_ASSERT_FLOAT_WITHIN(0.15f, ty, s.y);
  TEST_ASSERT_FLOAT_WITHIN(0.3f, 0.0f, s.vx);
  TEST_ASSERT_FLOAT_WITHIN(0.3f, 0.0f, s.vy);
}

void test_integration_constant_velocity_tracks() {
  EKFState s;
  float P[16];
  init_P(P);

  s.vx = 0.5f;
  float true_x = 0.0f;

  for (int i = 0; i < 200; i++) {
    float dt = 0.004f;
    true_x += 0.5f * dt;
    ekf_predict(s, P, 0.0f, 0.0f, dt);

    if (i % 12 == 0) {
      float d[] = {dist(true_x, 0, 0, 0), dist(true_x, 0, 3, 0),
                   dist(true_x, 0, 0, 4)};
      uint8_t ids[] = {10, 11, 12};
      ekf_update_uwb(s, P, d, ids, 3, STD_ANCHORS, 3);
    }
  }

  TEST_ASSERT_FLOAT_WITHIN(0.15f, true_x, s.x);
  TEST_ASSERT_FLOAT_WITHIN(0.3f, 0.5f, s.vx);
}

void test_integration_with_acceleration() {
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

    if (i % 12 == 0) {
      float d[] = {dist(true_x, 0, 0, 0), dist(true_x, 0, 3, 0),
                   dist(true_x, 0, 0, 4)};
      uint8_t ids[] = {10, 11, 12};
      ekf_update_uwb(s, P, d, ids, 3, STD_ANCHORS, 3);
    }
  }

  TEST_ASSERT_FLOAT_WITHIN(0.15f, true_x, s.x);
  TEST_ASSERT_FLOAT_WITHIN(0.3f, true_vx, s.vx);
}

void test_integration_predict_only_diverges() {
  EKFState s;
  float P[16];
  init_P(P);

  // Give it a good starting point
  float d[] = {dist(0.5f, 0, 0, 0), dist(0.5f, 0, 3, 0),
               dist(0.5f, 0, 0, 4)};
  uint8_t ids[] = {10, 11, 12};
  for (int i = 0; i < 10; i++)
    ekf_update_uwb(s, P, d, ids, 3, STD_ANCHORS, 3);
  float x_good = s.x;

  // Now predict with acceleration and no updates
  for (int i = 0; i < 500; i++) {
    ekf_predict(s, P, 0.1f, 0.0f, 0.004f);
  }

  TEST_ASSERT_TRUE(fabsf(s.x - x_good) > 0.1f);
  TEST_ASSERT_TRUE(P[0] > 1.0f);
}

void test_integration_outlier_rejected() {
  EKFState s;
  float P[16];
  init_P(P);

  // Converge to (1, 1)
  for (int i = 0; i < 50; i++) {
    float d[] = {dist(1, 1, 0, 0), dist(1, 1, 3, 0), dist(1, 1, 0, 4)};
    uint8_t ids[] = {10, 11, 12};
    ekf_update_uwb(s, P, d, ids, 3, STD_ANCHORS, 3);
  }
  float x_before = s.x;
  float y_before = s.y;

  // Try to fuse bogus ranges (huge distances)
  float bad_d[] = {50.0f, 50.0f, 50.0f};
  uint8_t ids[] = {10, 11, 12};
  uint8_t fused = ekf_update_uwb(s, P, bad_d, ids, 3, STD_ANCHORS, 3);

  TEST_ASSERT_EQUAL(0, fused);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, x_before, s.x);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, y_before, s.y);
}

void test_integration_noisy_ranges_converge() {
  EKFState s;
  float P[16];
  init_P(P);

  float tx = 1.5f, ty = 2.0f;
  // Simple pseudo-noise (alternating bias)
  for (int i = 0; i < 100; i++) {
    ekf_predict(s, P, 0.0f, 0.0f, 0.004f);

    if (i % 5 == 0) {
      float noise = (i % 10 == 0) ? 0.05f : -0.03f;
      float d[] = {dist(tx, ty, 0, 0) + noise, dist(tx, ty, 3, 0) - noise,
                   dist(tx, ty, 0, 4) + noise * 0.5f};
      uint8_t ids[] = {10, 11, 12};
      ekf_update_uwb(s, P, d, ids, 3, STD_ANCHORS, 3);
    }
  }

  TEST_ASSERT_FLOAT_WITHIN(0.2f, tx, s.x);
  TEST_ASSERT_FLOAT_WITHIN(0.2f, ty, s.y);
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
  RUN_TEST(test_predict_dt_at_boundary_0_5_passes);
  RUN_TEST(test_predict_covariance_grows);
  RUN_TEST(test_predict_covariance_symmetric);
  RUN_TEST(test_predict_multiple_steps_accumulate);
  RUN_TEST(test_predict_x_y_symmetry);
  RUN_TEST(test_predict_large_acceleration);
  RUN_TEST(test_predict_small_dt);

  // Covariance bounding
  RUN_TEST(test_covariance_bounded_after_many_predicts);
  RUN_TEST(test_covariance_symmetry_enforced);

  // Per-range update
  RUN_TEST(test_update_range_moves_toward_anchor);
  RUN_TEST(test_update_range_moves_away_from_anchor);
  RUN_TEST(test_update_range_reduces_position_covariance);
  RUN_TEST(test_update_range_covariance_stays_symmetric);
  RUN_TEST(test_update_range_covariance_stays_positive_diagonal);
  RUN_TEST(test_update_range_exact_distance_no_state_change);
  RUN_TEST(test_update_range_mahalanobis_gate_rejects_outlier);
  RUN_TEST(test_update_range_mahalanobis_adapts_to_uncertainty);
  RUN_TEST(test_update_range_single_range_partial_correction);
  RUN_TEST(test_update_range_state_at_anchor_handled);

  // Multi-range update
  RUN_TEST(test_multi_range_three_anchors_converges);
  RUN_TEST(test_multi_range_single_range_works);
  RUN_TEST(test_multi_range_two_ranges_works);
  RUN_TEST(test_multi_range_unmatched_ids_zero_fused);
  RUN_TEST(test_multi_range_negative_distance_skipped);
  RUN_TEST(test_multi_range_scrambled_ids);

  // Joseph form
  RUN_TEST(test_joseph_form_symmetry_after_many_updates);
  RUN_TEST(test_joseph_form_no_negative_diagonal_under_stress);

  // Integration
  RUN_TEST(test_integration_stationary_converges);
  RUN_TEST(test_integration_constant_velocity_tracks);
  RUN_TEST(test_integration_with_acceleration);
  RUN_TEST(test_integration_predict_only_diverges);
  RUN_TEST(test_integration_outlier_rejected);
  RUN_TEST(test_integration_noisy_ranges_converge);

  // Matrix helpers
  RUN_TEST(test_mat4_mul_identity);
  RUN_TEST(test_mat4_mul_zero);
  RUN_TEST(test_mat4_add);

  return UNITY_END();
}
