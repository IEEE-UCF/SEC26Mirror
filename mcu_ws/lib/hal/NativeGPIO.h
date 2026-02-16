/**
 * @file NativeGPIO.h
 * @brief HAL implementation for native MCU GPIO pins
 * @author Claude Code
 * @date 12/24/2025
 */

#pragma once

#include "HALPin.h"

namespace HAL {

/**
 * @brief HAL pin implementation for direct MCU GPIO pins
 *
 * This provides access to the native GPIO pins on the microcontroller
 * using standard Arduino functions (pinMode, digitalWrite, etc.)
 */
class NativeGPIO : public HALPin {
 public:
  /**
   * @brief Construct a native GPIO pin
   * @param pin_number MCU pin number
   */
  explicit NativeGPIO(uint8_t pin_number) : pin_(pin_number) {}

  ~NativeGPIO() override = default;

  void pinMode(PinMode mode) override {
    switch (mode) {
      case PinMode::PIN_INPUT:
        ::pinMode(pin_, INPUT);
        break;
      case PinMode::PIN_OUTPUT:
        ::pinMode(pin_, OUTPUT);
        break;
      case PinMode::PIN_INPUT_PULLUP:
        ::pinMode(pin_, INPUT_PULLUP);
        break;
    }
  }

  void digitalWrite(DigitalState state) override {
    ::digitalWrite(pin_, state == DigitalState::STATE_HIGH ? HIGH : LOW);
  }

  DigitalState digitalRead() override {
    return ::digitalRead(pin_) == HIGH ? DigitalState::STATE_HIGH
                                       : DigitalState::STATE_LOW;
  }

  void analogWrite(uint16_t value) override {
    // Clamp to 8-bit range for native analogWrite
    ::analogWrite(pin_, value > 255 ? 255 : value);
  }

  uint16_t analogRead() override { return ::analogRead(pin_); }

  const char* getInfo() override {
    snprintf(infoBuffer_, sizeof(infoBuffer_), "NativeGPIO pin %d", pin_);
    return infoBuffer_;
  }

  bool supportsDigitalRead() const override { return true; }
  bool supportsAnalogWrite() const override { return true; }
  bool supportsAnalogRead() const override { return true; }

  /**
   * @brief Get the pin number
   * @return MCU pin number
   */
  uint8_t getPinNumber() const { return pin_; }

 private:
  const uint8_t pin_;
  char infoBuffer_[32];
};

}  // namespace HAL
