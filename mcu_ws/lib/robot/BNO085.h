/**
 * @file BNO085.h
 * @author Trevor Cannon
 * @brief Defines BNO085 IMU driver wrapper for Adafruit BNO08x
 * @date 12/10/2025
 * @refactored 12/22/2025 - Renamed from IMU.h to BNO085.h for clarity
 */

#ifndef BNO085_H
#define BNO085_H

#include <Adafruit_BNO08x.h>
#include <BaseDriver.h>

// i2c address default is 0x4A

namespace Drivers {

class BNO085DriverSetup : public Classes::BaseSetup {
 public:
  const int8_t reset_pin;

  ~BNO085DriverSetup() = default;
  BNO085DriverSetup() = delete;

  BNO085DriverSetup(const char* _id, int8_t _pin = -1)
      : Classes::BaseSetup(_id), reset_pin(_pin){};

 private:
};

struct BNO085DriverData {
  float accel_x = 0.0f;
  float accel_y = 0.0f;
  float accel_z = 0.0f;

  float gyro_x = 0.0f;
  float gyro_y = 0.0f;
  float gyro_z = 0.0f;

  float qx = 0.0f;
  float qy = 0.0f;
  float qz = 0.0f;
  float qw = 0.0f;

  float yaw = 0.0f;
  float roll = 0.0f;
  float pitch = 0.0f;
};

class BNO085Driver : public Classes::BaseDriver {
 public:
  ~BNO085Driver() override = default;

  BNO085Driver(const BNO085DriverSetup& setup)
      : BaseDriver(setup), setup_(setup), imu_(setup.reset_pin){};

  bool init() override;
  void update() override;//constantly read for values and update data in the BNO085Driver Structure
  const char* getInfo() override;

  BNO085DriverData getData() { return data_; };
  float calculateRoll(float qx,float qy,float qz,float qw);
  float calculatePitch(float qx,float qy,float qz,float qw);
  float calculateYaw(float qx, float qy, float qz, float qw);
  float radians_to_degree(float angle);
 private:
  const BNO085DriverSetup setup_;
  BNO085DriverData data_; //structure data of the 
  char infoBuffer_[64];

  Adafruit_BNO08x imu_; // object instance of the actual IMU sensor
  sh2_SensorValue_t sensorValue_; //structure for sensor value
};
}  // namespace Drivers
#endif
