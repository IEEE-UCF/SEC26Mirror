#pragma once
// GyroSubsystem: BNO085 IMU wrapper for the drone
// Uses onboard sensor fusion (no Madgwick needed), outputs Euler angles + gyro rates

#include <Arduino.h>
#include <Adafruit_BNO08x.h>
#include "DroneControlSubsystem.h"  // for IMUData struct

namespace Drone {

struct GyroConfig {
  uint8_t i2c_addr = 0x4A;   // BNO085 default I2C address
  uint8_t reset_pin = -1;    // -1 = no hardware reset pin
  uint32_t rotation_report_us = 5000;  // 5ms = 200Hz quaternion
  uint32_t gyro_report_us = 2500;      // 2.5ms = 400Hz gyro
};

class GyroSubsystem {
 public:
  explicit GyroSubsystem(const GyroConfig& cfg = GyroConfig{}) : cfg_(cfg) {}

  bool init();
  void update();

  // Get latest IMU data (populated each update() call)
  const IMUData& getData() const { return data_; }
  bool isInitialized() const { return initialized_; }

 private:
  GyroConfig cfg_;
  Adafruit_BNO08x bno_;
  IMUData data_;
  bool initialized_ = false;
};

}  // namespace Drone
