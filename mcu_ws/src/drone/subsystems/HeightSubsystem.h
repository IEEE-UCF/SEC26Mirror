#pragma once
/**
 * @file HeightSubsystem.h
 * @brief VL53L0X TOF sensor for drone altitude measurement, as an
 *        RTOSSubsystem. Runs at 50Hz via beginThreadedPinned().
 */

#include <Arduino.h>
#include <RTOSSubsystem.h>
#include <VL53L0X.h>
#include <Wire.h>

namespace Drone {

struct HeightConfig {
  TwoWire* wire = &Wire;
  uint8_t xshut_pin = 255;         // 255 = no XSHUT pin
  uint16_t timing_budget_ms = 20;  // measurement timing budget
  float max_valid_m = 4.0f;        // reject readings above this
  float min_valid_m = 0.01f;       // reject readings below this
  SemaphoreHandle_t* i2c_mutex = nullptr;  // shared I2C bus mutex
};

class HeightSubsystem : public Subsystem::RTOSSubsystem {
 public:
  explicit HeightSubsystem(const HeightConfig& cfg = HeightConfig{})
      : RTOSSubsystem(setup_), cfg_(cfg) {}

  // RTOSSubsystem lifecycle
  bool init() override;
  void begin() override {}
  void update() override;
  void pause() override {}
  void reset() override {}
  const char* getInfo() override { return "height"; }

  // Thread-safe accessors
  float getAltitudeM() const {
    if (data_mutex_) xSemaphoreTake(data_mutex_, portMAX_DELAY);
    float val = altitude_m_;
    if (data_mutex_) xSemaphoreGive(data_mutex_);
    return val;
  }

  bool isValid() const {
    if (data_mutex_) xSemaphoreTake(data_mutex_, portMAX_DELAY);
    bool val = valid_;
    if (data_mutex_) xSemaphoreGive(data_mutex_);
    return val;
  }

  bool isInitialized() const { return initialized_; }

  uint32_t lastValidMs() const {
    if (data_mutex_) xSemaphoreTake(data_mutex_, portMAX_DELAY);
    uint32_t val = last_valid_ms_;
    if (data_mutex_) xSemaphoreGive(data_mutex_);
    return val;
  }

 private:
  Classes::BaseSetup setup_{"height"};
  HeightConfig cfg_;
  VL53L0X sensor_;
  float altitude_m_ = 0.0f;
  bool valid_ = false;
  bool initialized_ = false;
  uint32_t last_valid_ms_ = 0;
  mutable SemaphoreHandle_t data_mutex_ = nullptr;
};

}  // namespace Drone
