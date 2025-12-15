/**
 * @file IMU.h
 * @author Trevor Cannon
 * @brief Defines IMU wrapper for BNO08x IMU
 * @date 12/10/2025
 */

#ifndef IMUWRAPPER_H
#define IMUWRAPPER_H

#include <Adafruit_BNO08x.h>
#include <BaseDriver.h>

// i2c address default is 0x4A

namespace Drivers {

class IMUDriverSetup : public Classes::BaseSetup {
 public:
  const int8_t reset_pin;

  ~IMUDriverSetup() = default;
  IMUDriverSetup() = delete;

  IMUDriverSetup(const char* _id, int8_t _pin = -1)
      : Classes::BaseSetup(_id), reset_pin(_pin) {};

 private:
};

struct IMUDriverData {
  float accel_x = 0.0f;
  float accel_y = 0.0f;
  float accel_z = 0.0f;

  float gyro_x = 0.0f;
  float gyro_y = 0.0f;
  float gyro_z = 0.0f;

  float qi = 0.0f;
  float qj = 0.0f;
  float qk = 0.0f;
  float qr = 0.0f;

  float yaw = 0.0f;
};

class IMUDriver : public Classes::BaseDriver {
 public:
  ~IMUDriver() override = default;

  IMUDriver(const IMUDriverSetup& setup)
      : setup_(setup), BaseDriver(setup), imu_(setup.reset_pin) {};

  bool init() override;
  void update() override;
  const char* getInfo() override;

  IMUDriverData getData() { return data_; };

  float calculateYaw(float qi, float qj, float qk, float qr);

 private:
  const IMUDriverSetup setup_;
  IMUDriverData data_;
  char infoBuffer_[64];

  Adafruit_BNO08x imu_;
  sh2_SensorValue_t sensorValue_;
};
}  // namespace Drivers
#endif