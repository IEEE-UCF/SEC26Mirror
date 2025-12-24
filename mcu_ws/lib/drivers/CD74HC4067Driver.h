/**
 * @file CD74HC4067Driver.h
 * @brief Driver for CD74HC4067 16-channel analog/digital multiplexer
 * @author Claude Code
 * @date 12/24/2025
 */

#pragma once

#include <Arduino.h>
#include <BaseDriver.h>

namespace Drivers {

/**
 * @brief Setup configuration for CD74HC4067 driver
 */
class CD74HC4067DriverSetup : public Classes::BaseSetup {
 public:
  CD74HC4067DriverSetup(const char* _id, uint8_t s0, uint8_t s1, uint8_t s2,
                        uint8_t s3, uint8_t sig_pin)
      : Classes::BaseSetup(_id),
        s0_(s0),
        s1_(s1),
        s2_(s2),
        s3_(s3),
        sig_pin_(sig_pin) {}

  const uint8_t s0_;      // Select bit 0
  const uint8_t s1_;      // Select bit 1
  const uint8_t s2_;      // Select bit 2
  const uint8_t s3_;      // Select bit 3
  const uint8_t sig_pin_; // Common signal pin
};

/**
 * @brief Driver for CD74HC4067 multiplexer
 *
 * Controls a 16-channel multiplexer using 4 select pins.
 * Provides access to 16 channels through a single signal pin.
 */
class CD74HC4067Driver : public Classes::BaseDriver {
 public:
  explicit CD74HC4067Driver(const CD74HC4067DriverSetup& setup);
  ~CD74HC4067Driver() override = default;

  bool init() override;
  void update() override;
  const char* getInfo() override;

  /**
   * @brief Select a channel
   * @param channel Channel number (0-15)
   */
  void selectChannel(uint8_t channel);

  /**
   * @brief Get the currently selected channel
   * @return Channel number (0-15)
   */
  uint8_t getCurrentChannel() const { return current_channel_; }

  /**
   * @brief Read digital value from the currently selected channel
   * @return Digital value (HIGH or LOW)
   */
  uint8_t digitalRead();

  /**
   * @brief Write digital value to the currently selected channel
   * @param value Digital value (HIGH or LOW)
   */
  void digitalWrite(uint8_t value);

  /**
   * @brief Read analog value from the currently selected channel
   * @return Analog value (0-1023 for 10-bit ADC)
   */
  uint16_t analogRead();

  /**
   * @brief Get the signal pin number
   * @return Signal pin number
   */
  uint8_t getSignalPin() const { return setup_.sig_pin_; }

 private:
  const CD74HC4067DriverSetup setup_;
  uint8_t current_channel_;
  char infoBuffer_[64];
};

}  // namespace Drivers
