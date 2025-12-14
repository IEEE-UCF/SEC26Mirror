#include "TOF.h"

namespace Drivers {
bool TOFDriver::init() {
  Wire.begin();
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
  range_.range = sensor_.readRangeContinuousMillimeters();
}

std::string TOFDriver::getInfo() {
  std::string info =
      "\nID: " + getId() + "\nData (mm): " + std::to_string(range_.range);
  return info;
}
}  // namespace Drivers
