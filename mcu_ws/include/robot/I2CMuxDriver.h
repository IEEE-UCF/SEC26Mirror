/**
 * @file I2CMuxDriver.h
 * @author Samuel Lane
 * @brief Defines the I2CMuxDriver wrapper for the 8-channel TCA9548A I2C
 * Multiplexer
 * @date 10/21/2025
 */

#ifndef TCA9548AWRAPPER_H
#define TCA9548AWRAPPER_H

#include <Arduino.h>
#include <BaseDriver.h>
#include <TCA9548.h>

#include <algorithm>
#include <array>

namespace Drivers {

struct i2cChannelAddress {
  uint8_t address;  // i2c channel address number
};

struct i2cChannels_ {
  std::array<i2cChannelAddress, 8> i2cChannelArray;

  // constructor fills array of i2c channels with default values
  explicit i2cChannels_(int initval) {
    std::fill(i2cChannelArray.begin(), i2cChannelArray.end(),
              i2cChannelAddress{(uint8_t)initval});
  }
};

class I2CMuxDriver : public Classes ::BaseDriver {
 public:
  /**
   * @brief Constructs the driver.
   * @param address The I2C address of the TCA9548A (default is 0x70).
   */
  explicit I2CMuxDriver(uint8_t address = 0x70);

  // destructor
  ~I2CMuxDriver() override = default;

  // initialize up to 8 muxing channels, 3 address pins, and input pin(?)
  // or just use library
  void init() override;

  // handle switching logic(?)
  void update() override;

  bool selectChannel(uint8_t channel);

  bool isDeviceConnected(uint8_t channel, uint8_t deviceAddress);

  // implement getError() ?

 private:
  // Instance of the TCA9548 library class
  TCA9548 mux;
  // Store the mux's own I2C address
  uint8_t _muxAddress;
};
};  // namespace Drivers

#endif