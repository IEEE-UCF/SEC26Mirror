#pragma once
// GyroSubsystem: BNO085 IMU wrapper for the drone
// Uses the shared BNO085Driver with gyro reports enabled for flight control.
// Outputs Euler angles (degrees) + gyro rates (deg/s) via IMUData.

#include <Arduino.h>
#include <BNO085.h>

#include "DroneControlSubsystem.h"  // for IMUData struct

namespace Drone {

struct GyroConfig {
  uint8_t i2c_addr = 0x4A;             // BNO085 default I2C address
  uint8_t reset_pin = 255;             // 255 = no hardware reset pin
  int8_t int_pin = -1;                 // -1 = no interrupt pin
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

  // Access the underlying driver for diagnostics
  const Drivers::BNO085Driver& getDriver() const { return *driver_; }

 private:
  static constexpr float RAD_TO_DEG = 57.29577951f;

  GyroConfig cfg_;
  IMUData data_;
  bool initialized_ = false;

  // Driver setup + driver (heap-allocated to defer construction until init())
  Drivers::BNO085DriverSetup* driver_setup_ = nullptr;
  Drivers::BNO085Driver* driver_ = nullptr;
};

}  // namespace Drone
