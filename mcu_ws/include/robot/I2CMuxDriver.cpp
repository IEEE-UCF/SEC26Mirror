#include "I2CMuxDriver.h"

#include <TCA9548.h>
#include <Wire.h>

namespace Drivers {

uint8_t initaddr = 0x70;
i2cChannels_ chanelArray_(initaddr);

/**
 * @brief Constructs the driver, initializing the TCA9548 object.
 * @param address The I2C address of the TCA9548A.
 */
I2CMuxDriver::I2CMuxDriver(uint8_t address)
    : Classes::BaseDriver(),  // Call base class constructor
      mux(address),           // Initialize the TCA9548 object from the library
      _muxAddress(address) {}

/**
 * @brief Starts I2C communication and checks if multiplexer exists
 */

void I2CMuxDriver::init() {
  // Todo: deselect all channels at start

  // Start the Arduino Wire library for I2C communication
  Wire.begin();

  // Initializes TCA9548 object
  if (mux.begin()) {
    Serial.print("I2C Mux (0x");
    Serial.print(_muxAddress, HEX);
    Serial.println("Initialized successfully.");
  } else {
    Serial.print("I2C Mux 0x");
    Serial.print(_muxAddress, HEX);
    Serial.println("Failed to initialize");
  }
}

/**
 * @brief  Override from BaseDriver.
 */
void I2CMuxDriver::update() {
  // No action needed for a multiplexer in a standard update loop.
}

/**
 * @brief Selects a single I2C channel (0-7) on the multiplexer.
 * @param channel The channel index to select (0 to 7).
 * @return true if the channel was selected successfully, false otherwise.
 */
bool I2CMuxDriver::selectChannel(uint8_t channel) {
  if (channel > 7) {
    Serial.println(
        "Error: Attempted to select invalid Mux channel (must be 0-7).");
    return false;
  }

  // Use the TCA9548 library's API to switch the channel.
  // The RobTillaart library's select() method turns on the specified channel
  // and turns off any others.
  mux.selectChannel(channel);

  Serial.print("Mux 0x");
  Serial.print(_muxAddress, HEX);
  Serial.print(" switched to Channel ");
  Serial.println(channel);

  return true;
}

/**
 * @brief Scans the specified channel for a device at a specific I2C address.
 * @param channel The channel index (0-7).
 * @param deviceAddress The I2C address of the device to check (e.g., 0x48).
 * @return true if the device is found, false otherwise.
 */
bool I2CMuxDriver::isDeviceConnected(uint8_t channel, uint8_t deviceAddress) {
  if (channel > 7) {
    Serial.println("Error: Invalid Mux channel for device scan.");
    return false;
  }

  // Selects a specific channel
  if (!selectChannel(channel)) {
    return false;
  }

  // Does a standard I2C address scan (Wire.endTransmission returns 0 on
  // success)
  Wire.beginTransmission(deviceAddress);
  uint8_t error = Wire.endTransmission();

  if (error == 0) {
    Serial.print("Device 0x");
    Serial.print(deviceAddress, HEX);
    Serial.print(" found on Mux Channel ");
    Serial.println(channel);
    return true;
  } else {
    // Error 2 is NACK on address, 4 is no device error
    // Leaving the serial output for debugging the connection process
    Serial.print("Device 0x");
    Serial.print(deviceAddress, HEX);
    Serial.print(" NOT found on Mux Channel ");
    Serial.print(channel);
    Serial.print(" (Wire Error: ");
    Serial.print(error);
    Serial.println(")");
    return false;
  }
}

}  // namespace Drivers
