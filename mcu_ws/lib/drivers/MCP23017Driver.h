/**
 * @file MCP23017Driver.h
 * @brief Driver for MCP23017 16-bit I2C I/O expander
 * @author Claude Code
 * @date 12/24/2025
 */

#pragma once

#include <Adafruit_MCP23X17.h>
#include <Arduino.h>
#include <BaseDriver.h>

namespace Drivers {

/**
 * @brief Setup configuration for MCP23017 driver
 */
class MCP23017DriverSetup : public Classes::BaseSetup {
 public:
  MCP23017DriverSetup(const char* _id, uint8_t i2c_addr = 0x20)
      : Classes::BaseSetup(_id), i2c_addr_(i2c_addr) {}

  const uint8_t i2c_addr_;
};

/**
 * @brief Driver for MCP23017 I/O expander
 *
 * Provides digital I/O capabilities for 16 pins (GPA0-GPA7, GPB0-GPB7)
 * via I2C interface.
 */
class MCP23017Driver : public Classes::BaseDriver {
 public:
  explicit MCP23017Driver(const MCP23017DriverSetup& setup);
  ~MCP23017Driver() override = default;

  bool init() override;
  void update() override;
  const char* getInfo() override;

  /**
   * @brief Set pin mode
   * @param pin Pin number (0-15)
   * @param mode Pin mode (INPUT, OUTPUT, INPUT_PULLUP)
   */
  void pinMode(uint8_t pin, uint8_t mode);

  /**
   * @brief Write digital value to pin
   * @param pin Pin number (0-15)
   * @param value Digital value (HIGH or LOW)
   */
  void digitalWrite(uint8_t pin, uint8_t value);

  /**
   * @brief Read digital value from pin
   * @param pin Pin number (0-15)
   * @return Digital value (HIGH or LOW)
   */
  uint8_t digitalRead(uint8_t pin);

  /**
   * @brief Get direct access to the MCP23017 object
   * @return Reference to Adafruit MCP23X17 object
   */
  Adafruit_MCP23X17& getMCP() { return mcp_; }

 private:
  const MCP23017DriverSetup setup_;
  Adafruit_MCP23X17 mcp_;
  char infoBuffer_[64];
};

}  // namespace Drivers
