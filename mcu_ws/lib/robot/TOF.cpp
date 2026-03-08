#include "TOF.h"

namespace Drivers {

bool TOFDriver::init() {
  I2CBus::Lock lock(setup_.wire_);
  sensor_.setBus(&setup_.wire_);
  setup_.wire_.begin();

  if (!sensor_.init()) {
    initSuccess_ = false;
  } else {
    initSuccess_ = true;
    sensor_.setTimeout(setup_.timeout);
    sensor_.startContinuous(setup_.cooldown);
  }
  return initSuccess_;
}

void TOFDriver::update() {
  if (!initSuccess_) return;
  I2CBus::Lock lock(setup_.wire_);
  range_.range = sensor_.readRangeContinuousMillimeters();
}

const char* TOFDriver::getInfo() {
  static char buf[64];
  snprintf(buf, sizeof(buf), "ID: %s\nData (mm): %u", setup_.getId(),
           range_.range);
  return buf;
}

}  // namespace Drivers
