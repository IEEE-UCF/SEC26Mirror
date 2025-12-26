/**
 * @file MCP23017Driver.cpp
 * @brief Implementation of MCP23017 driver
 * @author Claude Code
 * @date 12/24/2025
 */

#include "MCP23017Driver.h"

namespace Drivers {

MCP23017Driver::MCP23017Driver(const MCP23017DriverSetup& setup)
    : BaseDriver(setup), setup_(setup) {}

bool MCP23017Driver::init() {
  if (!mcp_.begin_I2C(setup_.i2c_addr_)) {
    initSuccess_ = false;
    return false;
  }

  initSuccess_ = true;
  return true;
}

void MCP23017Driver::update() {
  // MCP23017 operations are synchronous, no buffering needed
  // This method is here to satisfy the BaseDriver interface
}

const char* MCP23017Driver::getInfo() {
  snprintf(infoBuffer_, sizeof(infoBuffer_), "MCP23017 @ 0x%02X",
           setup_.i2c_addr_);
  return infoBuffer_;
}

void MCP23017Driver::pinMode(uint8_t pin, uint8_t mode) {
  if (pin >= 16) return;  // MCP23017 has 16 pins

  mcp_.pinMode(pin, mode);
}

void MCP23017Driver::digitalWrite(uint8_t pin, uint8_t value) {
  if (pin >= 16) return;

  mcp_.digitalWrite(pin, value);
}

uint8_t MCP23017Driver::digitalRead(uint8_t pin) {
  if (pin >= 16) return LOW;

  return mcp_.digitalRead(pin);
}

}  // namespace Drivers
