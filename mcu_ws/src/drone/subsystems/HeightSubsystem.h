#pragma once
// HeightSubsystem: VL53L1X TOF sensor for drone altitude measurement

#include <Arduino.h>
#include <VL53L1X.h>  // Pololu VL53L1X library

namespace Drone {

struct HeightConfig {
  uint8_t xshut_pin = 255;       // 255 = no XSHUT pin
  uint16_t timing_budget_ms = 20; // measurement timing budget
  float max_valid_m = 4.0f;      // reject readings above this
  float min_valid_m = 0.01f;     // reject readings below this
};

class HeightSubsystem {
 public:
  explicit HeightSubsystem(const HeightConfig& cfg = HeightConfig{})
      : cfg_(cfg) {}

  bool init();
  void update();

  float getAltitudeM() const { return altitude_m_; }
  bool isValid() const { return valid_; }
  bool isInitialized() const { return initialized_; }

 private:
  HeightConfig cfg_;
  VL53L1X sensor_;
  float altitude_m_ = 0.0f;
  bool valid_ = false;
  bool initialized_ = false;
};

}  // namespace Drone
