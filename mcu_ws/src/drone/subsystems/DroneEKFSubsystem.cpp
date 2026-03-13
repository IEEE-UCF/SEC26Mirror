#include "DroneEKFSubsystem.h"

#include <cmath>
#include <cstring>

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
  // Don't integrate accel noise when there are no anchors to correct drift
  if (num_anchors_ == 0) return;

  xSemaphoreTake(mutex_, portMAX_DELAY);

  // State prediction: constant-acceleration kinematic model
  float dt2 = 0.5f * dt * dt;
  state_.x += state_.vx * dt + ax_world * dt2;
  state_.y += state_.vy * dt + ay_world * dt2;
  state_.vx += ax_world * dt;
  state_.vy += ay_world * dt;

  // F = state transition Jacobian
  float F[16] = {1, 0, dt, 0, 0, 1, 0, dt, 0, 0, 1, 0, 0, 0, 0, 1};
  float FT[16] = {1, 0, 0, 0, 0, 1, 0, 0, dt, 0, 1, 0, 0, dt, 0, 1};

  // Process noise Q (scaled by dt)
  float qp = Config::EKF_PROCESS_NOISE_POS * dt;
  float qv = Config::EKF_PROCESS_NOISE_VEL * dt;
  float Q[16] = {qp, 0, 0, 0, 0, qp, 0, 0, 0, 0, qv, 0, 0, 0, 0, qv};

  // P = F*P*F^T + Q
  float FP[16];
  mat4_mul(F, P_, FP);
  float FPFT[16];
  mat4_mul(FP, FT, FPFT);
  mat4_add(FPFT, Q, P_);

  // Bound covariance to prevent divergence during UWB dropout
  boundCovariance();

  xSemaphoreGive(mutex_);
}

uint8_t DroneEKFSubsystem::updateUWB(const float* distances_m,
                                      const uint8_t* peer_ids,
                                      uint8_t num_ranges) {
  xSemaphoreTake(mutex_, portMAX_DELAY);

  // Snapshot anchors under mutex to avoid race conditions
  AnchorInfo anchors_local[MAX_ANCHORS];
  uint8_t num_anchors_local = num_anchors_;
  memcpy(anchors_local, anchors_, sizeof(anchors_));

  uint8_t fused = 0;

  // Process each range as an independent scalar Kalman update
  for (uint8_t i = 0; i < num_ranges; i++) {
    if (distances_m[i] <= 0.0f) continue;

    // Find matching anchor
    for (uint8_t j = 0; j < num_anchors_local; j++) {
      if (anchors_local[j].valid && anchors_local[j].id == peer_ids[i]) {
        if (updateRange(distances_m[i], anchors_local[j].x,
                        anchors_local[j].y)) {
          fused++;
        }
        break;
      }
    }
  }

  xSemaphoreGive(mutex_);
  return fused;
}

bool DroneEKFSubsystem::updateRange(float distance_m, float anchor_x,
                                     float anchor_y) {
  // Nonlinear observation model: h(x) = sqrt((x-ax)² + (y-ay)²)
  float dx = state_.x - anchor_x;
  float dy = state_.y - anchor_y;
  float predicted_dist = sqrtf(dx * dx + dy * dy);

  // Avoid division by zero when state is exactly at anchor
  if (predicted_dist < 1e-4f) predicted_dist = 1e-4f;

  // Jacobian H = dh/dstate = [(x-ax)/d, (y-ay)/d, 0, 0]
  // Only position states affect range observation; velocity states don't.
  float H[4];
  H[0] = dx / predicted_dist;
  H[1] = dy / predicted_dist;
  H[2] = 0.0f;
  H[3] = 0.0f;

  // Innovation: z - h(x)
  float innovation = distance_m - predicted_dist;

  // S = H*P*H^T + R (scalar)
  // H*P*H^T = sum over i,j of H[i]*P[i*4+j]*H[j]
  float S = Config::EKF_MEASURE_NOISE_UWB;
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      S += H[i] * P_[i * 4 + j] * H[j];
    }
  }

  // Mahalanobis distance gate: (innovation^2)/S < threshold
  if (S < 1e-10f) return false;
  float mahal_sq = (innovation * innovation) / S;
  if (mahal_sq > Config::EKF_MAHALANOBIS_GATE_SQ) return false;

  // Kalman gain K = P*H^T / S (4×1 vector)
  float K[4];
  for (int i = 0; i < 4; i++) {
    K[i] = 0.0f;
    for (int j = 0; j < 4; j++) {
      K[i] += P_[i * 4 + j] * H[j];
    }
    K[i] /= S;
  }

  // State update: x += K * innovation
  state_.x += K[0] * innovation;
  state_.y += K[1] * innovation;
  state_.vx += K[2] * innovation;
  state_.vy += K[3] * innovation;

  // Joseph form covariance update: P = (I-KH)*P*(I-KH)^T + K*R*K^T
  // This is numerically stable and guarantees P stays symmetric positive
  // semi-definite even with floating point rounding.

  // Compute (I - K*H) as a 4×4 matrix
  float IKH[16];
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      IKH[i * 4 + j] = ((i == j) ? 1.0f : 0.0f) - K[i] * H[j];
    }
  }

  // (I-KH)^T
  float IKHT[16];
  for (int i = 0; i < 4; i++)
    for (int j = 0; j < 4; j++)
      IKHT[i * 4 + j] = IKH[j * 4 + i];

  // temp = (I-KH) * P
  float temp[16];
  mat4_mul(IKH, P_, temp);

  // P_new = temp * (I-KH)^T
  float P_new[16];
  mat4_mul(temp, IKHT, P_new);

  // Add K*R*K^T (rank-1 outer product scaled by R)
  float R = Config::EKF_MEASURE_NOISE_UWB;
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      P_new[i * 4 + j] += K[i] * R * K[j];
    }
  }

  memcpy(P_, P_new, sizeof(P_));
  return true;
}

void DroneEKFSubsystem::boundCovariance() {
  // Clamp diagonal entries to prevent unbounded growth
  if (P_[0] > Config::EKF_COV_MAX_POS) P_[0] = Config::EKF_COV_MAX_POS;
  if (P_[5] > Config::EKF_COV_MAX_POS) P_[5] = Config::EKF_COV_MAX_POS;
  if (P_[10] > Config::EKF_COV_MAX_VEL) P_[10] = Config::EKF_COV_MAX_VEL;
  if (P_[15] > Config::EKF_COV_MAX_VEL) P_[15] = Config::EKF_COV_MAX_VEL;

  // Enforce symmetry by averaging off-diagonal pairs
  for (int i = 0; i < 4; i++) {
    for (int j = i + 1; j < 4; j++) {
      float avg = 0.5f * (P_[i * 4 + j] + P_[j * 4 + i]);
      P_[i * 4 + j] = avg;
      P_[j * 4 + i] = avg;
    }
  }
}

void DroneEKFSubsystem::setAnchors(const AnchorInfo* anchors, uint8_t count) {
  xSemaphoreTake(mutex_, portMAX_DELAY);
  num_anchors_ = (count > MAX_ANCHORS) ? MAX_ANCHORS : count;
  for (uint8_t i = 0; i < num_anchors_; i++) {
    anchors_[i] = anchors[i];
  }
  xSemaphoreGive(mutex_);
}

}  // namespace Drone
