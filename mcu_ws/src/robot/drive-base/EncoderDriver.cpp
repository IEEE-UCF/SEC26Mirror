#include "EncoderDriver.h"

namespace Drivers {

bool EncoderDriver::init() { return true; }

void EncoderDriver::update() { position_.position = encoder_.read(); }

const char* EncoderDriver::getInfo() {
  snprintf(infoBuffer_, sizeof(infoBuffer_), "Encoder Driver: %s | Pos: %ld",
           setup_.getId(), position_.position);
  return infoBuffer_;
}
}  // namespace Drivers
