#include "ServoDriver.h"

#include <sstream>

namespace Drivers {

bool ServoDriverSingleController::hasBeenResetOnce = false;

ServoDriverSingleController::ServoDriverSingleController(
    const ServoDriverSingleSetup& setup)
    : Classes::BaseDriver(setup),
      controller_(setup.address),
      setup_(setup),
      pwmValues_(createPWMValues(setup.initialValues)) {}

std::array<ServoPWMValue, 16> ServoDriverSingleController::createPWMValues(
    const std::array<uint16_t, 16>& initialValues) {
  std::array<ServoPWMValue, 16> result;
  for (size_t i = 0; i < result.size(); ++i) {
    // Flag is automatically set to 'true' by the ServoPWMValue constructor
    result[i] = ServoPWMValue(initialValues[i]);
  }
  return result;
}

bool ServoDriverSingleController::init() {
  Wire.begin();
  if (!hasBeenResetOnce) {
    controller_.resetDevices();
    hasBeenResetOnce = true;
  }
  controller_.init();
  initSuccess_ = true;
  return initSuccess_;
}

void ServoDriverSingleController::update() { checkIterateWrite(); }

bool ServoDriverSingleController::set(uint8_t channel, uint16_t value) {
  if (channel > 15 || value > 4095) {
    return false;
  }
  ServoPWMValue newValue(value);
  if (pwmValues_[channel].pwmValue != newValue.pwmValue) {
    pwmValues_[channel] = newValue;  // flag automatically set to true
  }
  return true;
}

void ServoDriverSingleController::checkIterateWrite() {
  for (uint8_t channel = 0; channel < pwmValues_.size(); ++channel) {
    auto& pwm = pwmValues_[channel];
    if (pwm.flag) {
      controller_.setChannelPWM(channel, pwm.pwmValue);
      pwm.flag = false;  // Reset the flag after writing the value
    }
  }
}

ServoDriver::ServoDriver(const ServoDriverSetup& setup)
    : Classes::BaseDriver(setup), setup_(setup) {
  controllers_.reserve(setup_.driverSetups.size());
  for (const ServoDriverSingleSetup& singleSetup : setup_.driverSetups) {
    controllers_.emplace_back(singleSetup);
  }
}

bool ServoDriver::init() {
  if (!controllers_.empty()) {
    bool allSuccess = true;
    for (auto& controller : controllers_) {
      if (!controller.init()) {
        allSuccess = false;
      }
    }
    initSuccess_ = allSuccess;
  } else {
    initSuccess_ = true;
  }
  return initSuccess_;
}

void ServoDriver::update() {
  if (!controllers_.empty()) {
    for (auto& controller : controllers_) {
      controller.update();
    }
  }
}

bool ServoDriver::set(const PWMPin& pinHandle, uint16_t value) {
  if (pinHandle.device < controllers_.size()) {
    return controllers_[pinHandle.device].set(pinHandle.pin, value);
  } else {
    return false;
  }
}

/**
 * @brief Gets a string with info about the single controller.
 */
std::string ServoDriverSingleController::getInfo() {
  std::ostringstream oss;
  oss << "Driver: PCA9685 Controller\n";
  oss << "  Address: 0x" << std::hex << static_cast<int>(setup_.address)
      << "\n";
  // isInitialized() is assumed to be a public method from BaseDriver
  oss << "  Initialized: " << (initSuccess() ? "Yes" : "No");
  return oss.str();
}

/**
 * @brief Gets a string with info about all managed PCA9685 controllers.
 */
std::string ServoDriver::getInfo() {
  std::ostringstream oss;
  oss << "Driver: PCA9685 Manager\n";
  oss << "  Managed Devices: " << controllers_.size() << "\n";
  oss << "  All Initialized: " << (initSuccess() ? "Yes" : "No") << "\n";

  for (size_t i = 0; i < controllers_.size(); ++i) {
    oss << "  [Device " << i << "]\n";
    // Use the getter to access the child's setup
    oss << "    Address: 0x" << std::hex
        << static_cast<int>(controllers_[i].getSetup().address) << "\n";
    // Use the BaseDriver's assumed public method
    oss << "    Initialized: "
        << (controllers_[i].initSuccess() ? "Yes" : "No");
    if (i < controllers_.size() - 1) {
      oss << "\n";  // Add newline between device entries
    }
  }
  return oss.str();
}

}  // namespace Drivers
