/**
 * @file TCA9555Driver.h
 * @brief Driver for the TCA9555 16-bit I2C GPIO expander.
 * @date 2026-02-23
 *
 * The TCA9555 provides two 8-bit ports (Port 0 = pins 0–7, Port 1 = pins 8–15)
 * via I2C.  All 16 pins power-on as inputs.
 *
 * I2C address range: 0x20–0x27 (set by A0/A1/A2 hardware pins).
 *
 * Register map:
 *   0x00  Input  Port 0   (read-only)
 *   0x01  Input  Port 1   (read-only)
 *   0x02  Output Port 0   (read/write, default 0xFF)
 *   0x03  Output Port 1   (read/write, default 0xFF)
 *   0x04  Polarity Inversion Port 0
 *   0x05  Polarity Inversion Port 1
 *   0x06  Config Port 0   (1 = input, 0 = output, default 0xFF)
 *   0x07  Config Port 1   (1 = input, 0 = output, default 0xFF)
 */

#ifndef TCA9555DRIVER_H
#define TCA9555DRIVER_H

#include <Arduino.h>
#include <BaseDriver.h>
#include <Wire.h>

#include "I2CBusLock.h"

namespace Drivers {

class TCA9555DriverSetup : public Classes::BaseSetup {
 public:
  ~TCA9555DriverSetup() = default;
  TCA9555DriverSetup() = delete;

  /**
   * @param _id       Driver identifier string.
   * @param address   I2C address (0x20–0x27, default 0x20).
   * @param wire      I2C bus the expander is on (default Wire / Wire0).
   */
  TCA9555DriverSetup(const char* _id, uint8_t address = 0x20,
                     TwoWire& wire = Wire)
      : Classes::BaseSetup(_id), address_(address), wire_(wire) {}

  const uint8_t address_;
  TwoWire& wire_;
};

struct TCA9555DriverData {
  uint8_t inputPort0 = 0x00;   ///< Cached Port 0 input state
  uint8_t inputPort1 = 0x00;   ///< Cached Port 1 input state
  uint8_t outputPort0 = 0xFF;  ///< Port 0 output shadow register
  uint8_t outputPort1 = 0xFF;  ///< Port 1 output shadow register
  uint8_t configPort0 = 0xFF;  ///< Port 0 direction (1=in, 0=out)
  uint8_t configPort1 = 0xFF;  ///< Port 1 direction (1=in, 0=out)
};

class TCA9555Driver : public Classes::BaseDriver {
 public:
  explicit TCA9555Driver(const TCA9555DriverSetup& setup)
      : Classes::BaseDriver(setup), setup_(setup) {}

  ~TCA9555Driver() override = default;

  /// @brief Initialise I2C and push default config (all inputs) to the chip.
  bool init() override;

  /// @brief Refresh cached input port states from the chip.
  void update() override;

  const char* getInfo() override;

  // ── Pin-level API ─────────────────────────────────────────────────────────

  /**
   * Set the direction of a single pin.
   * @param pin  0–15 (0–7 = Port 0, 8–15 = Port 1).
   * @param mode Arduino INPUT or OUTPUT.
   * @return true on I2C success.
   */
  bool pinMode(uint8_t pin, uint8_t mode);

  /**
   * Write a single output pin.
   * @param pin  0–15.
   * @param val  true = HIGH, false = LOW.
   * @return true on I2C success.
   */
  bool digitalWrite(uint8_t pin, bool val);

  /**
   * Read a single pin from the cached input state (call update() first).
   * @param pin  0–15.
   */
  bool digitalRead(uint8_t pin) const;

  // ── Port-level API ────────────────────────────────────────────────────────

  /// Write an entire 8-bit output port (0 or 1). Returns true on success.
  bool writePort(uint8_t port, uint8_t value);

  /// Read an entire 8-bit port from the cache (call update() first).
  uint8_t readPort(uint8_t port) const;

  /// Set the direction register for port 0 or 1 (1=input, 0=output).
  bool configurePort(uint8_t port, uint8_t mask);

 private:
  bool writeReg(uint8_t reg, uint8_t value);
  uint8_t readReg(uint8_t reg);

  const TCA9555DriverSetup setup_;
  TCA9555DriverData data_;
  char infoBuf_[64] = {};
};

}  // namespace Drivers

#endif
