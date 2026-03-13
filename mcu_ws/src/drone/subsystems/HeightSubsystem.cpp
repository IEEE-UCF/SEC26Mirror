#include "HeightSubsystem.h"

namespace Drone {

bool HeightSubsystem::init() {
  data_mutex_ = xSemaphoreCreateMutex();

  if (cfg_.xshut_pin != 255) {
    pinMode(cfg_.xshut_pin, OUTPUT);
    digitalWrite(cfg_.xshut_pin, HIGH);
    delay(10);
  }

  sensor_.setBus(cfg_.wire);
  sensor_.setTimeout(500);
  if (!sensor_.init()) {
    initialized_ = false;
    return false;
  }

  sensor_.setMeasurementTimingBudget(cfg_.timing_budget_ms * 1000UL);
  sensor_.startContinuous(cfg_.timing_budget_ms);

  initialized_ = true;
  return true;
}

void HeightSubsystem::update() {
  if (!initialized_) return;

  // Acquire I2C mutex if available (shared bus with gyro)
  if (cfg_.i2c_mutex) xSemaphoreTake(*cfg_.i2c_mutex, portMAX_DELAY);

  uint16_t distance_mm = sensor_.readRangeContinuousMillimeters();
  bool timed_out = sensor_.timeoutOccurred();

  if (cfg_.i2c_mutex) xSemaphoreGive(*cfg_.i2c_mutex);

  if (timed_out) return;

  float raw_m = distance_mm / 1000.0f;

  xSemaphoreTake(data_mutex_, portMAX_DELAY);
  if (raw_m >= cfg_.min_valid_m && raw_m <= cfg_.max_valid_m) {
    altitude_m_ = raw_m;
    valid_ = true;
    last_valid_ms_ = millis();
  } else {
    valid_ = false;
  }
  xSemaphoreGive(data_mutex_);
}

}  // namespace Drone
