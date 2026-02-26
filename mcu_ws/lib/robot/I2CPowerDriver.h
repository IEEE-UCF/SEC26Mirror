/**
 * @file I2CPowerDriver.h
 * @author Alexander Peacock
 * @brief Defines the Driver for an INA228 I2C power sensor
 * @date 12/14/2025
 */

#ifndef I2CPOWERDRIVER_H
#define I2CPOWERDRIVER_H

#include <Adafruit_INA228.h>
#include <Arduino.h>
#include <BaseDriver.h>

#include <string>

namespace Drivers {

class I2CPowerDriverSetup : public Classes::BaseSetup {
 public:
  ~I2CPowerDriverSetup() = default;
  I2CPowerDriverSetup() = delete;

  I2CPowerDriverSetup(const char* _id, uint8_t address = 0x40,
                      float maxCurrent = 10, float shuntRes = 0.015)
      : Classes::BaseSetup(_id),
        _address(address),
        _maxCurrent(maxCurrent),
        _shuntRes(shuntRes){};

  const uint8_t _address;   // i2c address default is 0x40
  const float _maxCurrent;  // max current
  const float _shuntRes;    // shunt resistance
};

struct PowerDriverData {
  float currentmA = 0.0f;
  float busVoltage = 0.0f;
  float shuntVoltagemW = 0.0f;
  float powermW = 0.0f;
  float energy = 0.0f;
  float charge = 0.0f;
  float temp = 0.0f;
};

class I2CPowerDriver : public Classes::BaseDriver {
 public:
  I2CPowerDriver(const I2CPowerDriverSetup& setup)
      : BaseDriver(setup), _setup(setup){};

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
  float getCurrentmA();

  /// @brief  Get power
  /// @return float
  float getPowermW();

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