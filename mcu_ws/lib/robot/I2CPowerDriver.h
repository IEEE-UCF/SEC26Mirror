/**
 * @file I2CPowerDriver.h
 * @author Alexander Peacock
 * @brief Driver for an INA219 I2C power sensor, optionally routed through a
 *        TCA9548A I2C multiplexer.
 * @date 12/14/2025 (updated 2026-02-23)
 */

#ifndef I2CPOWERDRIVER_H
#define I2CPOWERDRIVER_H

#include <Adafruit_INA219.h>
#include <Arduino.h>
#include <BaseDriver.h>
#include <Wire.h>

#include "I2CBusLock.h"

namespace Drivers {

// Forward declaration — keeps I2CPowerDriver.h independent of I2CMuxDriver.h.
class I2CMuxDriver;

class I2CPowerDriverSetup : public Classes::BaseSetup {
 public:
  ~I2CPowerDriverSetup() = default;
  I2CPowerDriverSetup() = delete;

  /**
   * @param _id         Driver identifier string.
   * @param address     INA219 I2C address (default 0x40, range 0x40–0x4F).
   * @param mux         Pointer to an initialised I2CMuxDriver, or nullptr when
   *                    the INA219 is wired directly on the I2C bus.
   * @param muxChannel  TCA9548A channel (0–7) the INA219 is behind.
   *                    Ignored when mux == nullptr.
   * @param wire        I2C bus the INA219 is on (must match the mux's bus).
   * @param shuntOhm    Shunt resistor value in ohms. The Adafruit library
   *                    assumes 0.1 Ω; override here for custom hardware.
   *                    Current and power are derived from the raw shunt voltage
   *                    so the library's calibration register is not used for
   *                    those readings.
   */
  I2CPowerDriverSetup(const char* _id, uint8_t address = 0x40,
                      I2CMuxDriver* mux = nullptr, uint8_t muxChannel = 0,
                      TwoWire& wire = Wire, float shuntOhm = 0.1f)
      : Classes::BaseSetup(_id),
        _address(address),
        _mux(mux),
        _muxChannel(muxChannel),
        _wire(wire),
        _shuntOhm(shuntOhm) {}

  const uint8_t  _address;
  I2CMuxDriver*  _mux;
  const uint8_t  _muxChannel;
  TwoWire&       _wire;
  const float    _shuntOhm;
};

struct PowerDriverData {
  float currentmA      = 0.0f;  ///< mA
  float busVoltage     = 0.0f;  ///< V
  float shuntVoltagemV = 0.0f;  ///< mV
  float loadVoltage    = 0.0f;  ///< V  (= busVoltage + shuntVoltagemV/1000)
  float powermW        = 0.0f;  ///< mW
};

class I2CPowerDriver : public Classes::BaseDriver {
 public:
  explicit I2CPowerDriver(const I2CPowerDriverSetup& setup)
      : BaseDriver(setup), _setup(setup), _sensor(setup._address) {}

  ~I2CPowerDriver() override = default;

  /// @brief Initialise I2C, select mux channel (if configured), begin INA219.
  bool init() override;

  /// @brief Read all sensor channels and cache the results.
  void update() override;

  /// @brief Load voltage in volts (= bus + shunt-drop compensation).
  float getVoltage() const;

  /// @brief Current in milliamperes.
  float getCurrentmA() const;

  /// @brief Power in milliwatts.
  float getPowermW() const;

  /// @brief Shunt voltage in millivolts.
  float getShuntVoltagemV() const;

  const char* getInfo() override;

 private:
  const I2CPowerDriverSetup _setup;
  PowerDriverData           _data;
  Adafruit_INA219           _sensor;
  char                      _infoBuf[128] = {};
};

}  // namespace Drivers

#endif
