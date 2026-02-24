#include "I2CBusLock.h"

namespace I2CBus {

Threads::Mutex wire0;
Threads::Mutex wire1;
Threads::Mutex wire2;

Threads::Mutex& mutexFor(TwoWire& wire) {
  if (&wire == &Wire1) return wire1;
  if (&wire == &Wire2) return wire2;
  return wire0;  // Wire or any unrecognised bus
}

}  // namespace I2CBus
