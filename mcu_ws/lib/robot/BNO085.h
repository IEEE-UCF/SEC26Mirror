/**
 * @file BNO085.h
 * @author Trevor Cannon
 * @brief BNO085 IMU driver wrapper for Adafruit BNO08x.
 * @date 12/10/2025 (updated 2026-02-23)
 * @refactored 12/22/2025 - Renamed from IMU.h to BNO085.h for clarity
 *
 * Hardware: BNO085 on Wire1 (I2C address 0x4B).
 */

#ifndef BNO085_H
#define BNO085_H

#include <Adafruit_BNO08x.h>
#include <BaseDriver.h>
#include <Wire.h>

#if defined(USE_FREERTOS)
#include "I2CBusLock.h"
#endif

// digitalWriteFast / digitalReadFast — Teensy-specific; fall back to standard on ESP32
#ifndef digitalWriteFast
#define digitalWriteFast(pin, val) digitalWrite(pin, val)
#endif
#ifndef digitalReadFast
#define digitalReadFast(pin) digitalRead(pin)
#endif

class I2CDMABus;

namespace Drivers {

class BNO085DriverSetup : public Classes::BaseSetup {
 public:
  ~BNO085DriverSetup() = default;
  BNO085DriverSetup() = delete;

  /**
   * @param _id       Driver identifier string.
   * @param _pin      Reset pin, or -1 if not connected.
   * @param wire      I2C bus the BNO085 is on (default Wire1).
   * @param addr      I2C address (default 0x4B).
   * @param int_pin   Interrupt pin (active LOW when data ready), or -1 if not
   *                  connected.
   */
  BNO085DriverSetup(const char* _id, int8_t _pin = -1, TwoWire& wire = Wire1,
                    uint8_t addr = 0x4B, int8_t int_pin = -1,
                    bool enable_gyro = false,
                    uint32_t rotation_report_us = 20000,
                    uint32_t gyro_report_us = 2500)
      : Classes::BaseSetup(_id),
        reset_pin(_pin),
        wire_(wire),
        addr_(addr),
        int_pin_(int_pin),
        enable_gyro_(enable_gyro),
        rotation_report_us_(rotation_report_us),
        gyro_report_us_(gyro_report_us) {}

  const int8_t reset_pin;
  TwoWire& wire_;
  uint8_t addr_;
  int8_t int_pin_;
  bool enable_gyro_;                 ///< Also enable calibrated gyro reports
  uint32_t rotation_report_us_;      ///< Game Rotation Vector report interval
  uint32_t gyro_report_us_;          ///< Gyro report interval (if enabled)
};

struct BNO085DriverData {
  float accel_x = 0.0f;
  float accel_y = 0.0f;
  float accel_z = 0.0f;

  float gyro_x = 0.0f;  ///< Calibrated gyro rate (rad/s)
  float gyro_y = 0.0f;
  float gyro_z = 0.0f;

  float qx = 0.0f;
  float qy = 0.0f;
  float qz = 0.0f;
  float qw = 0.0f;

  float roll = 0.0f;   ///< Euler roll (radians)
  float pitch = 0.0f;  ///< Euler pitch (radians)
  float yaw = 0.0f;    ///< Euler yaw (radians)
};

class BNO085Driver : public Classes::BaseDriver {
 public:
  ~BNO085Driver() override = default;

  explicit BNO085Driver(const BNO085DriverSetup& setup)
      : BaseDriver(setup), setup_(setup), imu_(-1) {}
  // Pass -1 to Adafruit lib to skip its internal hardware reset (only 10ms
  // delay — insufficient).  BNO085Driver::init() does a proper reset with
  // adequate delay using setup_.reset_pin directly.

  bool init() override;
  void update() override;
  const char* getInfo() override;

  BNO085DriverData getData() const { return data_; }

  float calculateYaw(float qx, float qy, float qz, float qw);
  bool enableReports();

  /** @brief Zero the IMU heading using BNO085 hardware tare (Z-axis only). */
  bool tare();

  volatile uint32_t resetCount_ = 0;

  /// DMA stub, BNO085 SHTP requires multi-step variable-length I2C
  /// reads that cannot be batched into a single DMA cycle.
  void setDMABus(I2CDMABus* /*bus*/) {}

 private:
  const BNO085DriverSetup setup_;
  BNO085DriverData data_;
  char infoBuffer_[64] = {};

  Adafruit_BNO08x imu_;
  sh2_SensorValue_t sensorValue_;
};

}  // namespace Drivers

#endif
