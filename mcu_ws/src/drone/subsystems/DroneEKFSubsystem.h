#pragma once
/**
 * @file DroneEKFSubsystem.h
 * @brief Lightweight 4-state [x, y, vx, vy] EKF with per-range UWB updates.
 *
 * Height is handled separately by VL53L0X + LowPass1P.
 * Predict: 250Hz from flight task (IMU accel in world frame).
 * Update:  ~20Hz from UWB task (individual range measurements).
 *
 * Key design choices:
 *   - Per-range scalar Kalman updates (not trilaterate-then-fuse)
 *   - Nonlinear observation model h(x) = sqrt((x-ax)² + (y-ay)²)
 *   - Mahalanobis distance gating (chi² test, adapts to uncertainty)
 *   - Joseph form covariance update for numerical stability
 *   - Covariance bounding to prevent unbounded P growth during UWB dropout
 */

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include "../DroneConfig.h"

namespace Drone {

struct EKFState {
  float x = 0.0f;
  float y = 0.0f;
  float vx = 0.0f;
  float vy = 0.0f;
};

struct AnchorInfo {
  uint8_t id = 0;
  float x = 0.0f;
  float y = 0.0f;
  bool valid = false;
};

class DroneEKFSubsystem {
 public:
  DroneEKFSubsystem() = default;

  void init();

  // Called at 250Hz from flight task
  void predict(float ax_world, float ay_world, float dt);

  // Called at ~20Hz from UWB task with raw range measurements.
  // Processes each range individually as a scalar Kalman update.
  // Returns number of ranges successfully fused.
  uint8_t updateUWB(const float* distances_m, const uint8_t* peer_ids,
                    uint8_t num_ranges);

  // Thread-safe state access
  EKFState getState() const {
    xSemaphoreTake(mutex_, portMAX_DELAY);
    EKFState copy = state_;
    xSemaphoreGive(mutex_);
    return copy;
  }

  // Configure UWB anchors (thread-safe)
  void setAnchors(const AnchorInfo* anchors, uint8_t count);
  uint8_t getAnchorCount() const { return num_anchors_; }

 private:
  // Single-range scalar Kalman update. Returns false if gated out.
  bool updateRange(float distance_m, float anchor_x, float anchor_y);

  // Clamp covariance diagonal to prevent divergence during UWB dropout
  void boundCovariance();

  EKFState state_;
  mutable SemaphoreHandle_t mutex_ = nullptr;

  // Covariance matrix P (4×4, stored as flat array, row-major)
  float P_[16] = {};

  // Anchors (protected by mutex_)
  static constexpr uint8_t MAX_ANCHORS = 4;
  AnchorInfo anchors_[MAX_ANCHORS];
  uint8_t num_anchors_ = 0;
};

}  // namespace Drone
