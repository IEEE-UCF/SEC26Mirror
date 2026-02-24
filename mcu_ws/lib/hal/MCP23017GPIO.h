/**
 * @file MCP23017GPIO.h
 * @brief HAL implementation for MCP23017 I/O expander pins
 * @author Claude Code
 * @date 12/24/2025
 */

#pragma once

#include "HALPin.h"
#include "MCP23017Driver.h"

namespace HAL {

/**
 * @brief HAL pin implementation for MCP23017 I/O expander
 *
 * Provides digital I/O through MCP23017 pins.
 * Supports digitalRead and digitalWrite.
 * Does NOT support analogWrite or analogRead.
 */
class MCP23017GPIO : public HALPin {
 public:
  /**
   * @brief Construct an MCP23017 GPIO pin
   * @param driver Pointer to MCP23017 driver instance
   * @param pin Pin number on MCP23017 (0-15)
   */
  MCP23017GPIO(Drivers::MCP23017Driver* driver, uint8_t pin)
      : driver_(driver), pin_(pin) {}

  ~MCP23017GPIO() override = default;

  void pinMode(PinMode mode) override {
    if (!driver_) return;

    uint8_t arduino_mode;
    switch (mode) {
      case PinMode::PIN_INPUT:
        arduino_mode = INPUT;
        break;
      case PinMode::PIN_OUTPUT:
        arduino_mode = OUTPUT;
        break;
      case PinMode::PIN_INPUT_PULLUP:
        arduino_mode = INPUT_PULLUP;
        break;
    }
    driver_->pinMode(pin_, arduino_mode);
  }

  void digitalWrite(DigitalState state) override {
    if (!driver_) return;
    driver_->digitalWrite(pin_, state == DigitalState::STATE_HIGH ? HIGH : LOW);
  }

  DigitalState digitalRead() override {
    if (!driver_) return DigitalState::STATE_LOW;
    return driver_->digitalRead(pin_) == HIGH ? DigitalState::STATE_HIGH
                                              : DigitalState::STATE_LOW;
  }

  void analogWrite(uint16_t value) override {
    // MCP23017 does not support analog write
    (void)value;
  }

  uint16_t analogRead() override {
    // MCP23017 does not support analog read
    return 0;
  }

  const char* getInfo() override {
    snprintf(infoBuffer_, sizeof(infoBuffer_), "MCP23017 pin %d", pin_);
    return infoBuffer_;
  }

  bool supportsDigitalRead() const override { return true; }
  bool supportsAnalogWrite() const override { return false; }
  bool supportsAnalogRead() const override { return false; }

  /**
   * @brief Get the pin number
   * @return MCP23017 pin number (0-15)
   */
  uint8_t getPinNumber() const { return pin_; }

 private:
  Drivers::MCP23017Driver* driver_;
  const uint8_t pin_;
  char infoBuffer_[32];
};

}  // namespace HAL
