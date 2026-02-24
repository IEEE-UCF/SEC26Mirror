#include "I2CMuxDriver.h"

namespace Drivers {

bool I2CMuxDriver::init() {
  I2CBus::Lock lock(setup_.wire_);
  setup_.wire_.begin();
  initSuccess_ = mux_.begin();
  return initSuccess_;
}

const char* I2CMuxDriver::getInfo() {
  snprintf(infoBuf_, sizeof(infoBuf_),
           "I2CMux: ID=%s addr=0x%02X ch=%d",
           setup_.getId(), setup_.i2cAddress_, currentChannel_);
  return infoBuf_;
}

// NOTE: selectChannel, deselectAll, and isChannelEnabled do NOT lock the bus.
// Callers must hold I2CBus::Lock(wire) before calling these methods.

bool I2CMuxDriver::isChannelEnabled(uint8_t channel) {
  if (channel > 7) return false;
  return mux_.isEnabled(channel);
}

bool I2CMuxDriver::selectChannel(uint8_t channel) {
  if (channel > 7) return false;
  if (currentChannel_ == channel) return true;
  if (mux_.selectChannel(channel)) {
    currentChannel_ = channel;
    return true;
  }
  return false;
}

bool I2CMuxDriver::deselectAll() {
  if (mux_.disableAllChannels()) {
    currentChannel_ = 255;
    return true;
  }
  return false;
}

}  // namespace Drivers
