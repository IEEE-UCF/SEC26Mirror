#include "DroneEKFSubsystem.h"

#include <cmath>

namespace Drone {

// Helper: 4×4 matrix multiply C = A * B (row-major)
static void mat4_mul(const float* A, const float* B, float* C) {
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      float sum = 0.0f;
      for (int k = 0; k < 4; k++) {
        sum += A[i * 4 + k] * B[k * 4 + j];
      }
      C[i * 4 + j] = sum;
    }
  }
}

// Helper: 4×4 matrix add C = A + B
static void mat4_add(const float* A, const float* B, float* C) {
  for (int i = 0; i < 16; i++) C[i] = A[i] + B[i];
}

void DroneEKFSubsystem::init() {
  mutex_ = xSemaphoreCreateMutex();
  state_ = {};

  // Initialize P as identity scaled by initial uncertainty
  for (int i = 0; i < 16; i++) P_[i] = 0.0f;
  P_[0] = 1.0f;   // x variance
  P_[5] = 1.0f;   // y variance
  P_[10] = 0.5f;  // vx variance
  P_[15] = 0.5f;  // vy variance
}

void DroneEKFSubsystem::predict(float ax_world, float ay_world, float dt) {
  if (dt <= 0.0f || dt > 0.5f) return;

  xSemaphoreTake(mutex_, portMAX_DELAY);

  // State prediction
  // x  += vx*dt + 0.5*ax*dt²
  // y  += vy*dt + 0.5*ay*dt²
  // vx += ax*dt
  // vy += ay*dt
  float dt2 = 0.5f * dt * dt;
  state_.x += state_.vx * dt + ax_world * dt2;
  state_.y += state_.vy * dt + ay_world * dt2;
  state_.vx += ax_world * dt;
  state_.vy += ay_world * dt;

  // F = state transition Jacobian
  // [1  0  dt  0 ]
  // [0  1  0   dt]
  // [0  0  1   0 ]
  // [0  0  0   1 ]
  float F[16] = {1, 0, dt, 0, 0, 1, 0, dt, 0, 0, 1, 0, 0, 0, 0, 1};

  // F^T
  float FT[16] = {1, 0, 0, 0, 0, 1, 0, 0, dt, 0, 1, 0, 0, dt, 0, 1};

  // Process noise Q
  float qp = Config::EKF_PROCESS_NOISE_POS * dt;
  float qv = Config::EKF_PROCESS_NOISE_VEL * dt;
  float Q[16] = {qp, 0, 0, 0, 0, qp, 0, 0, 0, 0, qv, 0, 0, 0, 0, qv};

  // P = F*P*F^T + Q
  float FP[16];
  mat4_mul(F, P_, FP);
  float FPFT[16];
  mat4_mul(FP, FT, FPFT);
  mat4_add(FPFT, Q, P_);

  xSemaphoreGive(mutex_);
}

bool DroneEKFSubsystem::updateUWB(const float* distances_m,
                                   const uint8_t* peer_ids,
                                   uint8_t num_ranges) {
  float x_meas, y_meas;
  if (!trilaterate(distances_m, peer_ids, num_ranges, x_meas, y_meas)) {
    return false;
  }

  xSemaphoreTake(mutex_, portMAX_DELAY);

  // Innovation: y = z - H*x where H = [I₂ | 0₂]
  float innov_x = x_meas - state_.x;
  float innov_y = y_meas - state_.y;

  // Outlier gate
  float residual = sqrtf(innov_x * innov_x + innov_y * innov_y);
  if (residual > Config::EKF_OUTLIER_GATE_M) {
    xSemaphoreGive(mutex_);
    return false;
  }

  // S = H*P*H^T + R (2×2)
  // H*P*H^T extracts top-left 2×2 of P
  float R = Config::EKF_MEASURE_NOISE_UWB;
  float S00 = P_[0] + R;
  float S01 = P_[1];
  float S10 = P_[4];
  float S11 = P_[5] + R;

  // S inverse (2×2)
  float det = S00 * S11 - S01 * S10;
  if (fabsf(det) < 1e-10f) {
    xSemaphoreGive(mutex_);
    return false;
  }
  float inv_det = 1.0f / det;
  float Si00 = S11 * inv_det;
  float Si01 = -S01 * inv_det;
  float Si10 = -S10 * inv_det;
  float Si11 = S00 * inv_det;

  // K = P*H^T*S^-1 (4×2)
  // P*H^T = first two columns of P
  float K[8];  // 4×2
  K[0] = P_[0] * Si00 + P_[1] * Si10;
  K[1] = P_[0] * Si01 + P_[1] * Si11;
  K[2] = P_[4] * Si00 + P_[5] * Si10;
  K[3] = P_[4] * Si01 + P_[5] * Si11;
  K[4] = P_[8] * Si00 + P_[9] * Si10;
  K[5] = P_[8] * Si01 + P_[9] * Si11;
  K[6] = P_[12] * Si00 + P_[13] * Si10;
  K[7] = P_[12] * Si01 + P_[13] * Si11;

  // State update: x = x + K*y
  state_.x += K[0] * innov_x + K[1] * innov_y;
  state_.y += K[2] * innov_x + K[3] * innov_y;
  state_.vx += K[4] * innov_x + K[5] * innov_y;
  state_.vy += K[6] * innov_x + K[7] * innov_y;

  // Covariance update: P = (I - K*H)*P
  // K*H is 4×4: [K col | zeros]
  float KH[16] = {};
  for (int i = 0; i < 4; i++) {
    KH[i * 4 + 0] = K[i * 2 + 0];
    KH[i * 4 + 1] = K[i * 2 + 1];
    // columns 2,3 are zero
  }

  // P_new = (I - KH) * P
  float P_new[16];
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      float sum = 0.0f;
      for (int k = 0; k < 4; k++) {
        float I_KH = ((i == k) ? 1.0f : 0.0f) - KH[i * 4 + k];
        sum += I_KH * P_[k * 4 + j];
      }
      P_new[i * 4 + j] = sum;
    }
  }
  memcpy(P_, P_new, sizeof(P_));

  xSemaphoreGive(mutex_);
  return true;
}

void DroneEKFSubsystem::setAnchors(const AnchorInfo* anchors, uint8_t count) {
  xSemaphoreTake(mutex_, portMAX_DELAY);
  num_anchors_ = (count > MAX_ANCHORS) ? MAX_ANCHORS : count;
  for (uint8_t i = 0; i < num_anchors_; i++) {
    anchors_[i] = anchors[i];
  }
  xSemaphoreGive(mutex_);
}

bool DroneEKFSubsystem::trilaterate(const float* distances_m,
                                     const uint8_t* peer_ids,
                                     uint8_t num_ranges, float& x_out,
                                     float& y_out) {
  // Match peer_ids to known anchors
  struct MatchedRange {
    float x, y, d;
  };
  MatchedRange matched[MAX_ANCHORS];
  uint8_t n = 0;

  for (uint8_t i = 0; i < num_ranges && n < MAX_ANCHORS; i++) {
    if (distances_m[i] <= 0.0f) continue;
    for (uint8_t j = 0; j < num_anchors_; j++) {
      if (anchors_[j].valid && anchors_[j].id == peer_ids[i]) {
        matched[n].x = anchors_[j].x;
        matched[n].y = anchors_[j].y;
        matched[n].d = distances_m[i];
        n++;
        break;
      }
    }
  }

  if (n < 3) return false;

  // Linearize by subtracting pairs of circle equations:
  //   (x - xi)² + (y - yi)² = di²
  // Subtract equation 0 from equations 1..n-1:
  //   2*(x0-xi)*x + 2*(y0-yi)*y = d_i² - d_0² - xi² + x0² - yi² + y0²
  // This gives a linear system Ax = b.
  // For n >= 3, use first two equations for a 2×2 solve.

  float x0 = matched[0].x, y0 = matched[0].y, d0 = matched[0].d;
  float d0sq = d0 * d0;
  float x0sq = x0 * x0, y0sq = y0 * y0;

  // Build 2×2 system from first two linearized equations
  float A00 = 0, A01 = 0, A10 = 0, A11 = 0;
  float b0 = 0, b1 = 0;

  float xi = matched[1].x, yi = matched[1].y, di = matched[1].d;
  A00 = 2.0f * (x0 - xi);
  A01 = 2.0f * (y0 - yi);
  b0 = di * di - d0sq - xi * xi + x0sq - yi * yi + y0sq;

  xi = matched[2].x;
  yi = matched[2].y;
  di = matched[2].d;
  A10 = 2.0f * (x0 - xi);
  A11 = 2.0f * (y0 - yi);
  b1 = di * di - d0sq - xi * xi + x0sq - yi * yi + y0sq;

  // Solve 2×2 via Cramer's rule
  float det = A00 * A11 - A01 * A10;
  if (fabsf(det) < 1e-6f) return false;

  x_out = (b0 * A11 - b1 * A01) / det;
  y_out = (A00 * b1 - A10 * b0) / det;
  return true;
}

}  // namespace Drone
