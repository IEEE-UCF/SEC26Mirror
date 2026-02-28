#include "HeightSubsystem.h"

#include <Wire.h>

namespace Drone {

bool HeightSubsystem::init() {
  if (cfg_.xshut_pin != 255) {
    pinMode(cfg_.xshut_pin, OUTPUT);
    digitalWrite(cfg_.xshut_pin, HIGH);
    delay(10);
  }

  sensor_.setTimeout(500);
  if (!sensor_.init()) {
    initialized_ = false;
    return false;
  }

  // Set distance mode to long (up to 4m)
  sensor_.setDistanceMode(VL53L1X::Long);
  sensor_.setMeasurementTimingBudget(cfg_.timing_budget_ms * 1000);  // us

  // Start continuous measurement
  sensor_.startContinuous(cfg_.timing_budget_ms);

  initialized_ = true;
  return true;
}

void HeightSubsystem::update() {
  if (!initialized_) return;

  if (sensor_.dataReady()) {
    uint16_t distance_mm = sensor_.read(false);  // non-blocking
    float raw_m = distance_mm / 1000.0f;

    if (raw_m >= cfg_.min_valid_m && raw_m <= cfg_.max_valid_m) {
      altitude_m_ = raw_m;
      valid_ = true;
    } else {
      valid_ = false;
    }
  }
}

}  // namespace Drone
