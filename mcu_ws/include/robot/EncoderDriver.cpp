#include "EncoderDriver.h"

namespace Drivers {

bool EncoderDriver::init() { return true; }

void EncoderDriver::update() { position_.position = encoder_.read(); }

char* EncoderDriver::getInfo() {
  char info[] = "encoder";
  return info;
}
}  // namespace Drivers
