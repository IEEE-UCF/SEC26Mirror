/**
 * @file MPU6050.h
 * @author Trevor Cannon
 * @brief MPU6050 Driver Wrapper
 * @date 2/16/2026
 */

#pragma once
#include <Adafruit_MPU6050.h>
#include <BaseDriver.h>

namespace Drivers {

class MPU6050DriverSetup : public Classes::BaseSetup {
public:
  ~MPU6050DriverSetup() = default;
  MPU6050DriverSetup() = delete;

  MPU6050DriverSetup(const char *_id) : Classes::BaseSetup(_id) {};

private:
};

struct MPU6050DriverData {
  float a_x = 0.0f;
  float a_y = 0.0f;
  float a_z = 0.0f;

  float g_x = 0.0f;
  float g_y = 0.0f;
  float g_z = 0.0f;

  float temp = 0.0f;
};

class MPU6050Driver : public Classes::BaseDriver {
public:
  ~MPU6050Driver() = default;
  MPU6050Driver(const MPU6050DriverSetup &setup)
      : BaseDriver(setup), setup_(setup) {};

  bool init() override;
  void update() override;
  const char *getInfo() override;

  MPU6050DriverData getData() { return data_; }

private:
  const MPU6050DriverSetup setup_;
  MPU6050DriverData data_;
  Adafruit_MPU6050 mpu_;
  char infoBuffer_[64];

  sensors_event_t gyro_, accel_, temp_;
};
} // namespace Drivers