#include "GPIOExpanderDriver.h"

#include <Arduino.h>

#include <sstream>

namespace Drivers {

// --- GPIOExpanderSingleController Methods ---
// ... This implementation remains unchanged ...
GPIOExpanderSingleController::GPIOExpanderSingleController(
    const GPIOExpanderSingleSetup& setup)
    : Classes::BaseDriver(setup), mcp_(), setup_(setup), outputDirty_(false) {
  for (size_t i = 0; i < 16; ++i) {
    if (setup_.pinModes[i] == OUTPUT) {
      outputBuffer_[i] = setup_.initialOutputValues[i];
    }
  }
}

bool GPIOExpanderSingleController::init() {
  if (!mcp_.begin_I2C(setup_.address)) {
    initSuccess_ = false;
    return false;
  }
  uint16_t initialOutputWord = 0;
  for (uint8_t i = 0; i < 16; ++i) {
    mcp_.pinMode(i, setup_.pinModes[i]);
    if (setup_.pinModes[i] == OUTPUT) {
      if (outputBuffer_[i]) {
        initialOutputWord |= (1 << i);
      }
    }
  }
  mcp_.writeGPIOAB(initialOutputWord);
  outputDirty_ = false;
  initSuccess_ = true;
  return true;
}

void GPIOExpanderSingleController::update() {
  uint16_t inputs = mcp_.readGPIOAB();
  for (uint8_t i = 0; i < 16; ++i) {
    inputBuffer_[i] = (inputs >> i) & 0x1;
  }
  if (outputDirty_) {
    uint16_t outputWord = 0;
    for (uint8_t i = 0; i < 16; ++i) {
      if (outputBuffer_[i]) {
        outputWord |= (1 << i);
      }
    }
    mcp_.writeGPIOAB(outputWord);
    outputDirty_ = false;
  }
}

void GPIOExpanderSingleController::setPin(uint8_t pin, bool value) {
  if (pin >= 16) {
    return;
  }
  if (setup_.pinModes[pin] == OUTPUT) {
    if (outputBuffer_[pin] != value) {
      outputBuffer_[pin] = value;
      outputDirty_ = true;
    }
  }
}

bool GPIOExpanderSingleController::getPin(uint8_t pin) const {
  if (pin >= 16) {
    return false;
  }
  return inputBuffer_[pin];
}

uint16_t GPIOExpanderSingleController::getAllPins() const {
  uint16_t value = 0;
  for (uint8_t i = 0; i < 16; ++i) {
    if (inputBuffer_[i]) {
      value |= (1 << i);
    }
  }
  return value;
}

// --- GPIOExpanderDriver Methods ---

GPIOExpanderDriver::GPIOExpanderDriver(const GPIOExpanderSetup& setup)
    : Classes::BaseDriver(setup), setup_(setup) {
  controllers_.reserve(setup_.driverSetups.size());
  for (const GPIOExpanderSingleSetup& singleSetup : setup_.driverSetups) {
    controllers_.emplace_back(singleSetup);
  }
}

bool GPIOExpanderDriver::init() {
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

void GPIOExpanderDriver::update() {
  if (!controllers_.empty()) {
    for (auto& controller : controllers_) {
      controller.update();
    }
  }
}

void GPIOExpanderDriver::setPin(const GPIOPin& pinHandle, bool value) {
  // Check if the device offset is valid
  if (pinHandle.device < controllers_.size()) {
    // Call the single controller's setPin with the pin from the handle
    controllers_[pinHandle.device].setPin(pinHandle.pin, value);
  }
}

bool GPIOExpanderDriver::getPin(const GPIOPin& pinHandle) const {
  // Check if the device offset is valid
  if (pinHandle.device < controllers_.size()) {
    return controllers_[pinHandle.device].getPin(pinHandle.pin);
  } else {
    return false;  // Return LOW if device offset is invalid
  }
}

uint16_t GPIOExpanderDriver::getAllPins(uint8_t deviceOffset) const {
  if (deviceOffset < controllers_.size()) {
    return controllers_[deviceOffset].getAllPins();
  } else {
    return 0;  // Return 0 if device offset is invalid
  }
}

std::string GPIOExpanderSingleController::getInfo() {
  std::ostringstream oss;
  oss << "Driver: MCP23017 Controller\n";
  oss << "  Address: 0x" << std::hex << static_cast<int>(setup_.address)
      << "\n";
  // isInitialized() is assumed to be a public method from BaseDriver
  oss << "  Initialized: " << (initSuccess() ? "Yes" : "No");
  return oss.str();
}

std::string GPIOExpanderDriver::getInfo() {
  std::ostringstream oss;
  oss << "Driver: MCP23017 Manager\n";
  oss << "  Managed Devices: " << controllers_.size() << "\n";
  oss << "  All Initialized: " << (initSuccess() ? "Yes" : "No") << "\n";

  for (size_t i = 0; i < controllers_.size(); ++i) {
    oss << "  [Device " << i << "]\n";
    // Use the new getter to access the child's setup
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
