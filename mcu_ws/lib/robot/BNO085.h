/**
 * @file BNO085.h
 * @author Trevor Cannon
 * @brief BNO085 IMU driver wrapper for Adafruit BNO08x.
 * @date 12/10/2025 (updated 2026-02-23)
 * @refactored 12/22/2025 - Renamed from IMU.h to BNO085.h for clarity
 *
 * Hardware: BNO085 on Wire1 (I2C address 0x4A default).
 */

#ifndef BNO085_H
#define BNO085_H

#include <Adafruit_BNO08x.h>
#include <BaseDriver.h>
#include <Wire.h>

#include "I2CBusLock.h"

namespace Drivers {

class BNO085DriverSetup : public Classes::BaseSetup {
 public:
  ~BNO085DriverSetup() = default;
  BNO085DriverSetup() = delete;

  /**
   * @param _id    Driver identifier string.
   * @param _pin   Reset pin, or -1 if not connected.
   * @param wire   I2C bus the BNO085 is on (default Wire1).
   */
  BNO085DriverSetup(const char* _id, int8_t _pin = -1, TwoWire& wire = Wire1)
      : Classes::BaseSetup(_id), reset_pin(_pin), wire_(wire) {}

  const int8_t reset_pin;
  TwoWire& wire_;
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

  explicit BNO085Driver(const BNO085DriverSetup& setup)
      : BaseDriver(setup), setup_(setup), imu_(setup.reset_pin) {}

  bool init() override;
  void update() override;//constantly read for values and update data in the BNO085Driver Structure
  const char* getInfo() override;

<<<<<<< HEAD
  BNO085DriverData getData() { return data_; };
  float calculateRoll(float qx,float qy,float qz,float qw);
  float calculatePitch(float qx,float qy,float qz,float qw);
=======
  BNO085DriverData getData() const { return data_; }

>>>>>>> 24148fbc914056c80cc70e98718723a719880f53
  float calculateYaw(float qx, float qy, float qz, float qw);
  float radians_to_degree(float angle);
 private:
  const BNO085DriverSetup setup_;
<<<<<<< HEAD
  BNO085DriverData data_; //structure data of the 
  char infoBuffer_[64];
=======
  BNO085DriverData data_;
  char infoBuffer_[64] = {};
>>>>>>> 24148fbc914056c80cc70e98718723a719880f53

  Adafruit_BNO08x imu_; // object instance of the actual IMU sensor
  sh2_SensorValue_t sensorValue_; //structure for sensor value
};

}  // namespace Drivers

#endif
