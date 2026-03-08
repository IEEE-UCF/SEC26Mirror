#include "I2CBusLock.h"

namespace I2CBus {

FRMutex wire0;
FRMutex wire1;
FRMutex wire2;

FRMutex& mutexFor(TwoWire& wire) {
  if (&wire == &Wire1) return wire1;
  if (&wire == &Wire2) return wire2;
  return wire0;  // Wire or any unrecognised bus
}

}  // namespace I2CBus
