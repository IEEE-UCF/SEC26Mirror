#include "BH1750Driver.h"

#include "I2CDMABus.h"
#include "I2CMuxDriver.h"

namespace Drivers {

bool BH1750Driver::init() {
  I2CBus::Lock lock(setup_.wire_);
  setup_.wire_.begin();

  if (setup_.mux_ != nullptr &&
      !setup_.mux_->selectChannel(setup_.muxChannel_)) {
    initSuccess_ = false;
    return initSuccess_;
  }

  // Power on + set continuous high-resolution mode
  if (!sendCommand(CMD_POWER_ON)) {
    initSuccess_ = false;
    return initSuccess_;
  }
  delay(10);
  if (!sendCommand(CMD_CONTINUOUS_HIGH_RES)) {
    initSuccess_ = false;
    return initSuccess_;
  }

  initSuccess_ = true;
  return initSuccess_;
}

void BH1750Driver::update() {
  if (!initSuccess_) return;
  if (dma_bus_) return;

  I2CBus::Lock lock(setup_.wire_);

  if (setup_.mux_ != nullptr) {
    setup_.mux_->selectChannel(setup_.muxChannel_);
  }

  setup_.wire_.requestFrom(setup_.address_, (uint8_t)2);
  if (setup_.wire_.available() >= 2) {
    uint8_t hi = setup_.wire_.read();
    uint8_t lo = setup_.wire_.read();
    uint16_t raw = (hi << 8) | lo;
    lux_ = raw / 1.2f;
  }
}

// DMA support — queue a 2-byte read using the continuous mode command as
// the "register" byte.  The BH1750 interprets this as re-selecting the mode
// (harmless in continuous mode) and then returns the latest measurement.
void BH1750Driver::queueDMAReads() {
  if (!dma_bus_ || !initSuccess_) return;
  dma_bus_->queueRead(setup_.address_, CMD_CONTINUOUS_HIGH_RES, dma_rx_, 2);
}

void BH1750Driver::processDMAResults() {
  if (!initSuccess_) return;
  uint16_t raw = (uint16_t)((dma_rx_[0] << 8) | dma_rx_[1]);
  lux_ = raw / 1.2f;
}

bool BH1750Driver::sendCommand(uint8_t cmd) {
  setup_.wire_.beginTransmission(setup_.address_);
  setup_.wire_.write(cmd);
  return setup_.wire_.endTransmission() == 0;
}

const char* BH1750Driver::getInfo() {
  snprintf(infoBuf_, sizeof(infoBuf_), "BH1750: ID=%s addr=0x%02X lux=%.1f",
           setup_.getId(), setup_.address_, lux_);
  return infoBuf_;
}

}  // namespace Drivers
