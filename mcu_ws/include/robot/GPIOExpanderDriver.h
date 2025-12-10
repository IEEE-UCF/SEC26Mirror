/**
 * @file GPIOExpanderDriver.h
 * @author Aldem Pido
 * @brief Defines the GPIO Expander Driver for the MCP23017.
 * @date 10/23/25
 */

#ifndef MCP23017WRAPPER_H
#define MCP23017WRAPPER_H

#include <Adafruit_MCP23x17.h>
#include <BaseDriver.h>  // Assuming this is your project's base class

#include <array>
#include <cstdint>  // Use standard integer types

namespace Drivers {

struct GPIOExpanderSingleSetup : public Classes::BaseSetup {
  uint8_t address = 0x20;
  // use INPUT, OUTPUT, or INPUT_PULLUP
  std::array<uint8_t, 16> pinModes;
  std::array<bool, 16> initialOutputValues{false};
};

/**
 * @brief Driver class for a single MCP23017 16-port GPIO Expander.
 *
 * This class buffers inputs and outputs to minimize I2C traffic.
 * Outputs are buffered locally and written to the chip in a single
 * transaction during the update() call (if they have changed).
 * Inputs are read in a single transaction during the update() call
 * and stored in a local buffer.
 */
class GPIOExpanderSingleController : public Classes::BaseDriver {
 public:
  explicit GPIOExpanderSingleController(const GPIOExpanderSingleSetup& setup);

  ~GPIOExpanderSingleController() override = default;

  bool init() override;

  void update() override;

  std::string getInfo() override;

  void setPin(uint8_t pin, bool value);

  bool getPin(uint8_t pin) const;

  uint16_t getAllPins() const;

  const GPIOExpanderSingleSetup& getSetup() { return setup_; }

 private:
  Adafruit_MCP23X17 mcp_;
  const GPIOExpanderSingleSetup& setup_;

  /// @brief Local buffer for pin input states. Refreshed in update().
  std::array<bool, 16> inputBuffer_{false};

  /// @brief Local buffer for pin output states. Flushed to chip in update().
  std::array<bool, 16> outputBuffer_{false};

  /// @brief Flag to track if outputBuffer_ has changes to flush.
  bool outputDirty_ = false;
};

// *** BEGIN MULTI DRIVER MANAGER CLASS ***
struct GPIOExpanderSetup : public Classes::BaseSetup {
  std::vector<GPIOExpanderSingleSetup> driverSetups;
};

/**
 * @brief Represents a single pin on a specific GPIO expander device.
 * Used as a handle for cleaner configuration and access.
 */
struct GPIOPin {
  uint8_t device;  ///< The offset/index of the MCP23017 device.
  uint8_t pin;     ///< The pin number (0-15) on that device.
};

class GPIOExpanderDriver : public Classes::BaseDriver {
 public:
  ~GPIOExpanderDriver() override = default;
  explicit GPIOExpanderDriver(const GPIOExpanderSetup& setup);
  bool init() override;
  void update() override;
  std::string getInfo() override;

  /**
   * @brief Set the output state of a pin on a specific device.
   * @param pinHandle A struct containing the device and pin number.
   * @param value The value to set (true=HIGH, false=LOW).
   */
  void setPin(const GPIOPin& pinHandle, bool value);

  /**
   * @brief Get the buffered input state of a pin from a specific device.
   * @param pinHandle A struct containing the device and pin number.
   * @return The buffered state (true=HIGH, false=LOW).
   */
  bool getPin(const GPIOPin& pinHandle) const;

  /**
   * @brief Get all 16 buffered input pin states from a specific device.
   * @param deviceOffset The index of the MCP23017 device.
   * @return A 16-bit word representing all pin states.
   */
  uint16_t getAllPins(uint8_t deviceOffset) const;

  const GPIOExpanderSetup& getSetup() { return setup_; }

 private:
  std::vector<GPIOExpanderSingleController> controllers_;
  const GPIOExpanderSetup& setup_;
};

}  // namespace Drivers

#endif  // MCP23017WRAPPER_H
