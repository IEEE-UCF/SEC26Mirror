#include "EncoderDriver.h"

namespace Drivers {

bool EncoderDriver::init() { return true; }

void EncoderDriver::update() { position_.position = encoder_.read(); }

char* EncoderDriver::getInfo() {
  snprintf(infoBuffer_, sizeof(infoBuffer_), "Encoder Driver: %s | Pos: %ld",
           setup_.id, position_.position);
  return infoBuffer_;
}
}  // namespace Drivers
