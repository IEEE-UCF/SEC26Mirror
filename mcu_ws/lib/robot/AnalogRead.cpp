#include "AnalogRead.h"

namespace Drivers {

int genAnalogRead(uint8_t pin) {
  int value = analogRead(pin);
  return value;
}

}  // namespace Drivers
