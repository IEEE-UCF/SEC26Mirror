#pragma once
// GyroSubsystem: BNO085 IMU wrapper for the drone
// Uses onboard sensor fusion (no Madgwick needed), outputs Euler angles + gyro
// rates + linear acceleration

#include <Adafruit_BNO08x.h>
#include <Arduino.h>
#include <Wire.h>

namespace Drone {

// IMU data fed into the flight controller each tick
struct IMUData {
  float roll = 0.0f;    // degrees, from BNO085
  float pitch = 0.0f;   // degrees
  float yaw = 0.0f;     // degrees
  float gyro_x = 0.0f;  // deg/s, raw gyro
  float gyro_y = 0.0f;
  float gyro_z = 0.0f;
  float accel_x = 0.0f;  // m/s², linear (gravity-free)
  float accel_y = 0.0f;
  float accel_z = 0.0f;
};

struct GyroConfig {
  TwoWire* wire = &Wire;
  uint8_t i2c_addr = 0x4A;             // BNO085 default I2C address
  int8_t reset_pin = -1;               // -1 = no hardware reset pin
  int8_t int_pin = -1;                 // -1 = no interrupt pin
  uint32_t rotation_report_us = 5000;  // 5ms = 200Hz quaternion
  uint32_t gyro_report_us = 2500;      // 2.5ms = 400Hz gyro
  uint32_t accel_report_us = 5000;     // 5ms = 200Hz linear accel
};

class GyroSubsystem {
 public:
  explicit GyroSubsystem(const GyroConfig& cfg = GyroConfig{}) : cfg_(cfg) {}

  bool init();
  void update();

  // Get latest IMU data (thread-safe copy via mutex)
  IMUData getData() const {
    if (mutex_) xSemaphoreTake(mutex_, portMAX_DELAY);
    IMUData copy = data_;
    if (mutex_) xSemaphoreGive(mutex_);
    return copy;
  }

  // Get world-frame XY acceleration (rotated by yaw)
  void getAccelWorld(float yaw_rad, float& ax_world, float& ay_world) const {
    IMUData d = getData();
    float cy = cosf(yaw_rad);
    float sy = sinf(yaw_rad);
    ax_world = d.accel_x * cy - d.accel_y * sy;
    ay_world = d.accel_x * sy + d.accel_y * cy;
  }

  bool isInitialized() const { return initialized_; }

 private:
  GyroConfig cfg_;
  Adafruit_BNO08x bno_;
  IMUData data_;
  bool initialized_ = false;
  SemaphoreHandle_t mutex_ = nullptr;
};

}  // namespace Drone
