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

void test_predict_constant_velocity() {
  EKFState s;
  s.vx = 1.0f;  // 1 m/s in x
  float P[16];
  init_P(P);
  ekf_predict(s, P, 0.0f, 0.0f, 0.1f);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.1f, s.x);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, s.y);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.0f, s.vx);
}

void test_predict_constant_acceleration() {
  EKFState s;
  float P[16];
  init_P(P);
  float ax = 2.0f, dt = 0.1f;
  ekf_predict(s, P, ax, 0.0f, dt);
  // x = 0.5 * 2 * 0.01 = 0.01
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.01f, s.x);
  // vx = 2 * 0.1 = 0.2
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.2f, s.vx);
}

void test_predict_both_axes() {
  EKFState s;
  s.vx = 1.0f;
  s.vy = -0.5f;
  float P[16];
  init_P(P);
  ekf_predict(s, P, 0.5f, -1.0f, 0.1f);
  // x = 0 + 1.0*0.1 + 0.5*0.5*0.01 = 0.1025
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.1025f, s.x);
  // y = 0 + (-0.5)*0.1 + 0.5*(-1.0)*0.01 = -0.055
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, -0.055f, s.y);
  // vx = 1.0 + 0.5*0.1 = 1.05
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 1.05f, s.vx);
  // vy = -0.5 + (-1.0)*0.1 = -0.6
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, -0.6f, s.vy);
}

void test_predict_invalid_dt_rejected() {
  EKFState s;
  float P[16];
  init_P(P);
  ekf_predict(s, P, 10.0f, 10.0f, 0.0f);  // dt=0 → rejected
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, s.x);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, s.vx);

  ekf_predict(s, P, 10.0f, 10.0f, -0.1f);  // dt<0 → rejected
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, s.x);

  ekf_predict(s, P, 10.0f, 10.0f, 0.6f);  // dt>0.5 → rejected
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, s.x);
}

void test_predict_covariance_grows() {
  EKFState s;
  float P[16];
  init_P(P);
  float P_diag_before = P[0];
  ekf_predict(s, P, 0.0f, 0.0f, 0.01f);
  // P should grow due to process noise
  TEST_ASSERT_TRUE(P[0] > P_diag_before);
}

void test_predict_multiple_steps_accumulate() {
  EKFState s;
  float P[16];
  init_P(P);
  // 100 steps of 0.004s with 1 m/s^2 acceleration
  for (int i = 0; i < 100; i++) {
    ekf_predict(s, P, 1.0f, 0.0f, 0.004f);
  }
  // After 0.4s: x = 0.5*1.0*0.16 = 0.08, vx = 1.0*0.4 = 0.4
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.08f, s.x);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.4f, s.vx);
}

// ════════════════════════════════════════════════════════════════
//  TRILATERATION TESTS
// ════════════════════════════════════════════════════════════════

void test_trilaterate_known_position() {
  // 3 anchors at known positions, distances computed to (1, 1)
  AnchorInfo anchors[3] = {
      {10, 0.0f, 0.0f, true}, {11, 3.0f, 0.0f, true}, {12, 0.0f, 4.0f, true}};
  float target_x = 1.0f, target_y = 1.0f;
  float d0 = sqrtf(1.0f + 1.0f);          // from (0,0) = sqrt(2)
  float d1 = sqrtf(4.0f + 1.0f);          // from (3,0) = sqrt(5)
  float d2 = sqrtf(1.0f + 9.0f);          // from (0,4) = sqrt(10)
  float distances[] = {d0, d1, d2};
  uint8_t ids[] = {10, 11, 12};

  float x, y;
  bool ok = trilaterate(distances, ids, 3, anchors, 3, x, y);
  TEST_ASSERT_TRUE(ok);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, target_x, x);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, target_y, y);
}

void test_trilaterate_origin() {
  AnchorInfo anchors[3] = {
      {10, 0.0f, 0.0f, true}, {11, 3.0f, 0.0f, true}, {12, 0.0f, 4.0f, true}};
  float distances[] = {0.0001f, 3.0f, 4.0f};
  uint8_t ids[] = {10, 11, 12};

  float x, y;
  bool ok = trilaterate(distances, ids, 3, anchors, 3, x, y);
  TEST_ASSERT_TRUE(ok);
  TEST_ASSERT_FLOAT_WITHIN(0.05f, 0.0f, x);
  TEST_ASSERT_FLOAT_WITHIN(0.05f, 0.0f, y);
}

void test_trilaterate_too_few_ranges() {
  AnchorInfo anchors[3] = {
      {10, 0.0f, 0.0f, true}, {11, 3.0f, 0.0f, true}, {12, 0.0f, 4.0f, true}};
  float distances[] = {1.0f, 2.0f};
  uint8_t ids[] = {10, 11};

  float x, y;
  bool ok = trilaterate(distances, ids, 2, anchors, 3, x, y);
  TEST_ASSERT_FALSE(ok);
}

void test_trilaterate_unmatched_ids() {
  AnchorInfo anchors[3] = {
      {10, 0.0f, 0.0f, true}, {11, 3.0f, 0.0f, true}, {12, 0.0f, 4.0f, true}};
  float distances[] = {1.0f, 2.0f, 3.0f};
  uint8_t ids[] = {99, 98, 97};  // no match

  float x, y;
  bool ok = trilaterate(distances, ids, 3, anchors, 3, x, y);
  TEST_ASSERT_FALSE(ok);
}

void test_trilaterate_collinear_anchors_degenerate() {
  // All anchors on X axis → y equation system is degenerate
  AnchorInfo anchors[3] = {
      {10, 0.0f, 0.0f, true}, {11, 1.0f, 0.0f, true}, {12, 2.0f, 0.0f, true}};
  float distances[] = {1.0f, 1.0f, 1.0f};
  uint8_t ids[] = {10, 11, 12};

  float x, y;
  bool ok = trilaterate(distances, ids, 3, anchors, 3, x, y);
  // With collinear anchors on X-axis, A01=0 and A11=0 → det=0 → should fail
  TEST_ASSERT_FALSE(ok);
}

void test_trilaterate_negative_distance_skipped() {
  AnchorInfo anchors[3] = {
      {10, 0.0f, 0.0f, true}, {11, 3.0f, 0.0f, true}, {12, 0.0f, 4.0f, true}};
  float distances[] = {-1.0f, 3.0f, 4.0f, 1.414f};
  uint8_t ids[] = {10, 11, 12, 10};

  float x, y;
  // First distance is negative → skipped, only 2 valid → should fail
  bool ok = trilaterate(distances, ids, 3, anchors, 3, x, y);
  TEST_ASSERT_FALSE(ok);
}

void test_trilaterate_extra_ranges_uses_first_three() {
  // 4 ranges but only first 3 are used for the 2x2 solve
  AnchorInfo anchors[4] = {{10, 0.0f, 0.0f, true},
                           {11, 3.0f, 0.0f, true},
                           {12, 0.0f, 4.0f, true},
                           {13, 3.0f, 4.0f, true}};
  float target_x = 1.0f, target_y = 1.0f;
  float d0 = sqrtf(2.0f);
  float d1 = sqrtf(5.0f);
  float d2 = sqrtf(10.0f);
  float d3 = sqrtf(13.0f);
  float distances[] = {d0, d1, d2, d3};
  uint8_t ids[] = {10, 11, 12, 13};

  float x, y;
  bool ok = trilaterate(distances, ids, 4, anchors, 4, x, y);
  TEST_ASSERT_TRUE(ok);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, target_x, x);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, target_y, y);
}

// ════════════════════════════════════════════════════════════════
//  UPDATE TESTS
// ════════════════════════════════════════════════════════════════

void test_update_moves_state_toward_measurement() {
  EKFState s;
  float P[16];
  init_P(P);
  // Measurement says we're at (1, 0) but state says (0, 0)
  bool ok = ekf_update(s, P, 1.0f, 0.0f);
  TEST_ASSERT_TRUE(ok);
  TEST_ASSERT_TRUE(s.x > 0.0f);  // State moves toward measurement
  TEST_ASSERT_TRUE(s.x < 1.0f);  // But not all the way (Kalman smoothing)
}

void test_update_reduces_covariance() {
  EKFState s;
  float P[16];
  init_P(P);
  float P_x_before = P[0];
  ekf_update(s, P, 0.0f, 0.0f);
  TEST_ASSERT_TRUE(P[0] < P_x_before);
}

void test_update_outlier_rejected() {
  EKFState s;
  float P[16];
  init_P(P);
  // Measurement 2m away → exceeds OUTLIER_GATE_M (1.0)
  bool ok = ekf_update(s, P, 2.0f, 0.0f);
  TEST_ASSERT_FALSE(ok);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, s.x);
}

void test_update_within_gate_accepted() {
  EKFState s;
  float P[16];
  init_P(P);
  // Measurement 0.5m away → within OUTLIER_GATE_M (1.0)
  bool ok = ekf_update(s, P, 0.5f, 0.0f);
  TEST_ASSERT_TRUE(ok);
  TEST_ASSERT_TRUE(s.x > 0.0f);
}

void test_update_exact_match_no_change() {
  EKFState s;
  s.x = 1.0f;
  s.y = 2.0f;
  float P[16];
  init_P(P);
  // Measurement matches state exactly → innovation is zero
  bool ok = ekf_update(s, P, 1.0f, 2.0f);
  TEST_ASSERT_TRUE(ok);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 1.0f, s.x);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 2.0f, s.y);
}

// ════════════════════════════════════════════════════════════════
//  PREDICT + UPDATE INTEGRATION TESTS
// ════════════════════════════════════════════════════════════════

void test_predict_update_converges_to_truth() {
  EKFState s;
  float P[16];
  init_P(P);

  // Simulate: drone at (1, 1), no motion, UWB measures (1, 1) repeatedly
  AnchorInfo anchors[3] = {
      {10, 0.0f, 0.0f, true}, {11, 3.0f, 0.0f, true}, {12, 0.0f, 4.0f, true}};

  for (int i = 0; i < 50; i++) {
    // Predict (no accel)
    ekf_predict(s, P, 0.0f, 0.0f, 0.004f);

    // Every 12 predictions (~48ms), do a UWB update
    if (i % 12 == 0) {
      float d0 = sqrtf(2.0f);
      float d1 = sqrtf(5.0f);
      float d2 = sqrtf(10.0f);
      float distances[] = {d0, d1, d2};
      uint8_t ids[] = {10, 11, 12};
      float x_meas, y_meas;
      if (trilaterate(distances, ids, 3, anchors, 3, x_meas, y_meas)) {
        ekf_update(s, P, x_meas, y_meas);
      }
    }
  }

  // After convergence, state should be near (1, 1)
  TEST_ASSERT_FLOAT_WITHIN(0.3f, 1.0f, s.x);
  TEST_ASSERT_FLOAT_WITHIN(0.3f, 1.0f, s.y);
  // Velocity should be near zero
  TEST_ASSERT_FLOAT_WITHIN(0.5f, 0.0f, s.vx);
  TEST_ASSERT_FLOAT_WITHIN(0.5f, 0.0f, s.vy);
}

void test_predict_update_with_motion() {
  EKFState s;
  float P[16];
  init_P(P);

  // Simulate constant velocity: vx=1.0 m/s, measure position periodically
  s.vx = 1.0f;
  float true_x = 0.0f;

  for (int i = 0; i < 100; i++) {
    float dt = 0.004f;
    true_x += 1.0f * dt;
    ekf_predict(s, P, 0.0f, 0.0f, dt);

    if (i % 12 == 0) {
      // Direct position measurement (simulating trilateration result)
      ekf_update(s, P, true_x, 0.0f);
    }
  }

  // After 0.4s at 1 m/s, true_x ≈ 0.4
  TEST_ASSERT_FLOAT_WITHIN(0.1f, true_x, s.x);
  TEST_ASSERT_FLOAT_WITHIN(0.3f, 1.0f, s.vx);
}

int main(int argc, char** argv) {
  UNITY_BEGIN();

  // Predict
  RUN_TEST(test_predict_zero_accel_zero_vel_no_change);
  RUN_TEST(test_predict_constant_velocity);
  RUN_TEST(test_predict_constant_acceleration);
  RUN_TEST(test_predict_both_axes);
  RUN_TEST(test_predict_invalid_dt_rejected);
  RUN_TEST(test_predict_covariance_grows);
  RUN_TEST(test_predict_multiple_steps_accumulate);

  // Trilateration
  RUN_TEST(test_trilaterate_known_position);
  RUN_TEST(test_trilaterate_origin);
  RUN_TEST(test_trilaterate_too_few_ranges);
  RUN_TEST(test_trilaterate_unmatched_ids);
  RUN_TEST(test_trilaterate_collinear_anchors_degenerate);
  RUN_TEST(test_trilaterate_negative_distance_skipped);
  RUN_TEST(test_trilaterate_extra_ranges_uses_first_three);

  // Update
  RUN_TEST(test_update_moves_state_toward_measurement);
  RUN_TEST(test_update_reduces_covariance);
  RUN_TEST(test_update_outlier_rejected);
  RUN_TEST(test_update_within_gate_accepted);
  RUN_TEST(test_update_exact_match_no_change);

  // Integration
  RUN_TEST(test_predict_update_converges_to_truth);
  RUN_TEST(test_predict_update_with_motion);

  return UNITY_END();
}
