#pragma once
// GyroSubsystem: BNO085 IMU wrapper for the drone
// Uses onboard sensor fusion (no Madgwick needed!), outputs Euler angles + gyro
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

  // Get world-frame XY acceleration using full ZYX Euler rotation.
  // BNO085 linear accel is body-frame with gravity removed — we need
  // the full roll/pitch/yaw rotation to project onto world XY.
  // Only rotating by yaw causes drift proportional to tilt angle.
  void getAccelWorld(float& ax_world, float& ay_world) const {
    IMUData d = getData();
    float roll_rad = d.roll * 0.01745329252f;
    float pitch_rad = d.pitch * 0.01745329252f;
    float yaw_rad = d.yaw * 0.01745329252f;

    float cr = cosf(roll_rad), sr = sinf(roll_rad);
    float cp = cosf(pitch_rad), sp = sinf(pitch_rad);
    float cy = cosf(yaw_rad), sy = sinf(yaw_rad);

    // ZYX rotation matrix (body → world), rows 0 and 1 only
    ax_world = (cy * cp) * d.accel_x +
               (cy * sp * sr - sy * cr) * d.accel_y +
               (cy * sp * cr + sy * sr) * d.accel_z;
    ay_world = (sy * cp) * d.accel_x +
               (sy * sp * sr + cy * cr) * d.accel_y +
               (sy * sp * cr - cy * sr) * d.accel_z;
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
