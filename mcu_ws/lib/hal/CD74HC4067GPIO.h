/**
 * @file CD74HC4067GPIO.h
 * @brief HAL implementation for CD74HC4067 multiplexer channels
 * @author Claude Code
 * @date 12/24/2025
 */

#pragma once

#include "HALPin.h"
#include "drivers/CD74HC4067Driver.h"

namespace HAL {

/**
 * @brief HAL pin implementation for CD74HC4067 multiplexer
 *
 * Provides access to multiplexer channels.
 * Supports digitalRead, digitalWrite, and analogRead.
 * Does NOT support analogWrite (multiplexer is passive).
 *
 * IMPORTANT: Operations on this pin will automatically select the channel
 * on the multiplexer. If you have multiple pins on the same multiplexer,
 * each operation will change the selected channel.
 */
class CD74HC4067GPIO : public HALPin {
 public:
  /**
   * @brief Construct a CD74HC4067 GPIO pin
   * @param driver Pointer to CD74HC4067 driver instance
   * @param channel Channel number on multiplexer (0-15)
   */
  CD74HC4067GPIO(Drivers::CD74HC4067Driver* driver, uint8_t channel)
      : driver_(driver), channel_(channel) {}

  ~CD74HC4067GPIO() override = default;

  void pinMode(PinMode mode) override {
    // The signal pin mode is managed by the operations themselves
    // Store the desired mode for future reference
    (void)mode;
  }

  void digitalWrite(DigitalState state) override {
    if (!driver_) return;
    driver_->selectChannel(channel_);
    driver_->digitalWrite(state == DigitalState::STATE_HIGH ? HIGH : LOW);
  }

  DigitalState digitalRead() override {
    if (!driver_) return DigitalState::STATE_LOW;
    driver_->selectChannel(channel_);
    return driver_->digitalRead() == HIGH ? DigitalState::STATE_HIGH
                                          : DigitalState::STATE_LOW;
  }

  void analogWrite(uint16_t value) override {
    // Multiplexer does not support analog write
    (void)value;
  }

  uint16_t analogRead() override {
    if (!driver_) return 0;
    driver_->selectChannel(channel_);
    return driver_->analogRead();
  }

  const char* getInfo() override {
    snprintf(infoBuffer_, sizeof(infoBuffer_), "CD74HC4067 ch %d", channel_);
    return infoBuffer_;
  }

  bool supportsDigitalRead() const override { return true; }
  bool supportsAnalogWrite() const override { return false; }
  bool supportsAnalogRead() const override { return true; }

  /**
   * @brief Get the channel number
   * @return Multiplexer channel number (0-15)
   */
  uint8_t getChannel() const { return channel_; }

 private:
  Drivers::CD74HC4067Driver* driver_;
  const uint8_t channel_;
  char infoBuffer_[32];
};

}  // namespace HAL
