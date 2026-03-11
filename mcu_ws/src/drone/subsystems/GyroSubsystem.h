#pragma once
/**
 * @file GyroSubsystem.h
 * @brief BNO085 IMU wrapper for the drone, as an RTOSSubsystem.
 *
 * Uses onboard sensor fusion (no Madgwick needed!), outputs Euler angles +
 * gyro rates + linear acceleration.  Runs as a FreeRTOS task via
 * beginThreadedPinned() for precise timing at 250Hz on core 1.
 */

#include <Adafruit_BNO08x.h>
#include <Arduino.h>
#include <RTOSSubsystem.h>
#include <Wire.h>

#include "../DroneDebug.h"

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
  SemaphoreHandle_t* i2c_mutex = nullptr;  // shared I2C bus mutex
};

class GyroSubsystem : public Subsystem::RTOSSubsystem {
 public:
  explicit GyroSubsystem(const GyroConfig& cfg = GyroConfig{})
      : RTOSSubsystem(setup_), cfg_(cfg) {}

  // RTOSSubsystem lifecycle
  bool init() override;
  void begin() override {}
  void update() override;
  void pause() override {}
  void reset() override {}
  const char* getInfo() override { return "gyro"; }

  bool enableReports();

  // Get latest IMU data (thread-safe copy via data mutex)
  IMUData getData() const {
    if (data_mutex_) xSemaphoreTake(data_mutex_, portMAX_DELAY);
    IMUData copy = data_;
    if (data_mutex_) xSemaphoreGive(data_mutex_);
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

  // Tare: capture current roll/pitch/yaw as zero reference
  void tare() {
    IMUData d = getData();
    roll_offset_ += d.roll;
    pitch_offset_ += d.pitch;
    yaw_offset_ += d.yaw;
    DRONE_PRINTF("[Gyro] Tare: roll=%.1f pitch=%.1f yaw=%.1f deg\n",
                 roll_offset_, pitch_offset_, yaw_offset_);
  }

 private:
  Classes::BaseSetup setup_{"gyro"};
  GyroConfig cfg_;
  Adafruit_BNO08x bno_;
  IMUData data_;
  bool initialized_ = false;
  uint32_t reset_count_ = 0;
  uint32_t reset_wait_until_ = 0;  // non-blocking boot delay after BNO085 reset
  float roll_offset_ = 0.0f;
  float pitch_offset_ = 0.0f;
  float yaw_offset_ = 0.0f;
  bool auto_tare_pending_ = true;
  mutable SemaphoreHandle_t data_mutex_ = nullptr;
};

}  // namespace Drone
