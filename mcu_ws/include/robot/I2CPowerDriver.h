/**
 * @file I2CPowerDriver.h
 * @author Alexander Peacock
 * @brief Defines the Driver for an INA228 I2C power sensor
 * @date 12/13/2025
 */

#ifndef I2CPOWERDRIVER_H
#define I2CPOWERDRIVER_H

#include <Adafruit_INA228.h>
#include <Arduino.h>
#include <BaseDriver.h>

// i2c address default is 0x40

namespace Drivers {

class I2CPowerDriverSetup : public Classes::BaseSetup {
 public:
  ~I2CPowerDriverSetup() = default;
  I2CPowerDriverSetup() = delete;

  I2CPowerDriverSetup(const char* _id, uint8_t address = 0x40,
                      uint8_t maxCurrent = 10, float shuntRes = 0.015)
      : Classes::BaseSetup(_id),
        _address(address),
        _maxCurrent(maxCurrent),
        _shuntRes(shuntRes) {};

  const uint8_t _address;
  const uint8_t _maxCurrent;
  const float _shuntRes;
};

struct PowerDriverData {
  float current = 0.0f;
  float busVoltage = 0.0f;
  float shuntVoltage = 0.0f;
  float power = 0.0f;
  float energy = 0.0f;
  float charge = 0.0f;
  float temp = 0.0f;
};

class I2CPowerDriver : public Classes::BaseDriver {
 public:
  I2CPowerDriver(const I2CPowerDriverSetup& setup)
      : BaseDriver(setup), _setup(setup) {};

  ~I2CPowerDriver() override = default;

  /// @brief  Initialize driver
  /// @return Success
  bool init() override;

  /// @brief Update driver
  /// @return float
  void update() override;

  /// @brief  Get bus voltage
  /// @return float
  float getVoltage();

  /// @brief  Get current
  /// @return float
  float getCurrent();

  /// @brief  Get power
  /// @return float
  float getPower();

  /// @brief  Get temperature
  /// @return float
  float getTemp();

  /// @brief Get info in the form of a data string
  /// @return data string
  const char* getInfo() override;

 private:
  const I2CPowerDriverSetup _setup;
  PowerDriverData _data;

  Adafruit_INA228 _sensor;
};

};  // namespace Drivers

#endif