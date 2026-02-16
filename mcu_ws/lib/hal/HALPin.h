/**
 * @file HALPin.h
 * @brief Hardware Abstraction Layer for GPIO pins
 * @author Claude Code
 * @date 12/24/2025
 *
 * This HAL allows subsystems to access GPIO pins through different hardware
 * backends (direct MCU pins, PCA9685, MCP23017, CD74HC4067) without knowing
 * the underlying implementation.
 */

#pragma once

#include <Arduino.h>

#include <cstdint>

namespace HAL {

/**
 * @brief Pin modes for HAL pins
 * Note: Uses PIN_ prefix to avoid conflicts with Arduino INPUT/OUTPUT macros
 */
enum class PinMode : uint8_t { PIN_INPUT, PIN_OUTPUT, PIN_INPUT_PULLUP };

/**
 * @brief Digital pin states
 * Note: Uses STATE_ prefix to avoid conflicts with Arduino HIGH/LOW macros
 */
enum class DigitalState : uint8_t { STATE_LOW = 0, STATE_HIGH = 1 };

/**
 * @brief Abstract interface for a GPIO pin
 *
 * All HAL pin backends must implement this interface.
 * Not all backends support all operations (e.g., PCA9685 doesn't support
 * digitalRead).
 */
class HALPin {
 public:
  virtual ~HALPin() = default;

  /**
   * @brief Initialize the pin with a mode
   * @param mode Pin mode (INPUT, OUTPUT, INPUT_PULLUP)
   */
  virtual void pinMode(PinMode mode) = 0;

  /**
   * @brief Write a digital value to the pin
   * @param state Digital state (LOW or HIGH)
   */
  virtual void digitalWrite(DigitalState state) = 0;

  /**
   * @brief Read a digital value from the pin
   * @return Digital state (LOW or HIGH)
   * @note Not supported by all backends (e.g., PCA9685)
   */
  virtual DigitalState digitalRead() = 0;

  /**
   * @brief Write an analog (PWM) value to the pin
   * @param value PWM value (0-255 for 8-bit, 0-4095 for 12-bit)
   * @note Not supported by all backends (e.g., MCP23017)
   */
  virtual void analogWrite(uint16_t value) = 0;

  /**
   * @brief Read an analog value from the pin
   * @return Analog value (0-1023 for 10-bit ADC, 0-4095 for 12-bit)
   * @note Not supported by all backends
   */
  virtual uint16_t analogRead() = 0;

  /**
   * @brief Get information about this pin
   * @return String describing the pin
   */
  virtual const char* getInfo() = 0;

  /**
   * @brief Check if this pin supports digital read
   * @return true if digitalRead() is supported
   */
  virtual bool supportsDigitalRead() const = 0;

  /**
   * @brief Check if this pin supports analog write (PWM)
   * @return true if analogWrite() is supported
   */
  virtual bool supportsAnalogWrite() const = 0;

  /**
   * @brief Check if this pin supports analog read (ADC)
   * @return true if analogRead() is supported
   */
  virtual bool supportsAnalogRead() const = 0;
};

}  // namespace HAL
