/**
 * @file ServoDriver.h
 * @author Aldem Pido
 * @brief Defines the ServoDriver wrapper for the PCA9685.
 * @date 10/15/25
 */

#ifndef PCA9685WRAPPER_H
#define PCA9685WRAPPER_H
#include <BaseDriver.h>
#include <PCA9685.h>
#include <stdint.h>

#include <array>
#include <vector>

namespace Drivers {
// *** BEGIN SINGLE DRIVER CLASS ***
///< Configuration for a single PCA9685 driver IC.
struct ServoDriverSingleSetup : public Classes::BaseSetup {
  ///< The I2C address of the PCA9685 device. (0x40 default)
  byte address = 0x40;
  ///< Initial PWM values for each channel.
  std::array<uint16_t, 16> initialValues = {0};
};

///< Represents a PWM channel's value and a flag to track changes.
struct ServoPWMValue {
  ///< The raw PWM value (0-4095).
  uint16_t pwmValue = 0;
  ///< A "dirty" flag; true if the value needs to be written.
  bool flag = true;

  ServoPWMValue() = default;  ///< Default constructor.
  explicit ServoPWMValue(const uint16_t pwm, const bool fg = true)
      : pwmValue(pwm), flag(fg) {}
};

/**
 * @brief Represents a single pin on a specific PWM expander device.
 * Used as a handle for cleaner configuration and access.
 */
struct PWMPin {
  uint8_t device;  ///< The offset/index of the PCA9685 device.
  uint8_t pin;     ///< The pin number (0-15) on that device.
};

///< Represents the state and control for a single PCA9685 device.
class ServoDriverSingleController : public Classes::BaseDriver {
 public:
  ~ServoDriverSingleController() override = default;
  explicit ServoDriverSingleController(const ServoDriverSingleSetup& setup);
  bool init() override;
  void update() override;
  std::string getInfo() override;

  bool set(uint8_t channel, uint16_t value);

  const ServoDriverSingleSetup& getSetup() { return setup_; }

 private:
  PCA9685 controller_;
  const ServoDriverSingleSetup& setup_;
  std::array<ServoPWMValue, 16> pwmValues_;

  void checkIterateWrite();

  static bool hasBeenResetOnce;
  static std::array<ServoPWMValue, 16> createPWMValues(
      const std::array<uint16_t, 16>& initialValues);
};

// *** BEGIN MULTI DRIVER MANAGER CLASS ***
///< Configuration for the entire ServoDriver, consisting of multiple PCA9685
struct ServoDriverSetup : public Classes::BaseSetup {
  ///< Vector of configurations for each PCA9685 device.
  std::vector<ServoDriverSingleSetup> driverSetups;
};

///< High-level driver for managing one or more PCA9685 16-channel PWM
class ServoDriver : public Classes::BaseDriver {
 public:
  ~ServoDriver() override = default;
  explicit ServoDriver(const ServoDriverSetup& setup);
  bool init() override;
  void update() override;
  std::string getInfo() override;

  bool set(const PWMPin& pinHandle, uint16_t value);

  const ServoDriverSetup& getSetup() { return setup_; }

 private:
  std::vector<ServoDriverSingleController> controllers_;
  const ServoDriverSetup& setup_;
};
}  // namespace Drivers

#endif