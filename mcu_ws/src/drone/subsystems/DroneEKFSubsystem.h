#pragma once
/**
 * @file DroneEKFSubsystem.h
 * @brief Lightweight 4-state [x, y, vx, vy] EKF with UWB trilateration.
 *
 * Height is handled separately by VL53L0X + LowPass1P.
 * Predict: 250Hz from flight task (IMU accel in world frame).
 * Update:  ~20Hz from UWB task (trilaterated position).
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

  // Called at ~20Hz from UWB task with raw range measurements
  // Returns true if update was applied (enough valid ranges + passed outlier
  // gate)
  bool updateUWB(const float* distances_m, const uint8_t* peer_ids,
                 uint8_t num_ranges);

  // Thread-safe state access
  EKFState getState() const {
    xSemaphoreTake(mutex_, portMAX_DELAY);
    EKFState copy = state_;
    xSemaphoreGive(mutex_);
    return copy;
  }

  // Configure UWB anchors
  void setAnchors(const AnchorInfo* anchors, uint8_t count);
  uint8_t getAnchorCount() const { return num_anchors_; }

 private:
  // Trilaterate from UWB ranges → (x, y). Returns false if < 3 valid.
  bool trilaterate(const float* distances_m, const uint8_t* peer_ids,
                   uint8_t num_ranges, float& x_out, float& y_out);

  EKFState state_;
  mutable SemaphoreHandle_t mutex_ = nullptr;

  // Covariance matrix P (4×4, stored as flat array, row-major)
  float P_[16] = {};

  // Anchors
  static constexpr uint8_t MAX_ANCHORS = 4;
  AnchorInfo anchors_[MAX_ANCHORS];
  uint8_t num_anchors_ = 0;
};

}  // namespace Drone
