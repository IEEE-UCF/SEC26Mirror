#include "I2CMuxDriver.h"

namespace Drivers {

bool I2CMuxDriver::init() {
  initSuccess_ = true;
  if (!mux.begin()) initSuccess_ = false;
  return initSuccess_;
}

void I2CMuxDriver::update() {}

const char* I2CMuxDriver::getInfo() {
  snprintf(infoBuf_, sizeof(infoBuf_), "I2C Mux: ID = %s, Addr = 0x%02x, Ch=%d",
           setup_.getId(), setup_.i2cAddress_);
  return infoBuf_;
}

bool I2CMuxDriver::isChannelEnabled(uint8_t channel) {
  if (channel > 7) return false;
  return mux.isEnabled(channel);
}

bool I2CMuxDriver::selectChannel(uint8_t channel) {
  if (channel > 7) return false;
  if (currentChannel_ == channel) return true;

  if (mux.selectChannel(channel)) {
    currentChannel_ = channel;
    return true;
  }
  return false;
}

bool I2CMuxDriver::deselectAll() {
  if (mux.disableAllChannels()) {
    currentChannel_ = 255;
    return true;
  }
  return false;
}

}  // namespace Drivers
