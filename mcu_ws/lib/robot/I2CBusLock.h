/**
 * @file I2CBusLock.h
 * @brief TeensyThreads mutex guards for the three I2C buses on Teensy 4.1.
 *
 * Uses Threads::Mutex from TeensyThreads — cooperative with the scheduler
 * and avoids a raw busy-wait.  Each bus has a dedicated mutex so Wire0,
 * Wire1, and Wire2 can be driven concurrently by separate tasks.
 *
 * Hardware layout (Teensy 4.1):
 *   Wire  (Wire0) — TCA9548A mux, TCA9555 GPIO expander, INA219 (via mux)
 *   Wire1         — BNO085 IMU
 *   Wire2         — PCA9685 PWM drivers (#1 and #2)
 *
 * Usage:
 * @code
 *   void MyDriver::update() {
 *     I2CBus::Lock lock(setup_.wire_);   // blocks until bus is free
 *     sensor_.read();
 *   }                                    // lock released here
 * @endcode
 *
 * initLocks() is a no-op — Threads::Mutex is default-constructible — but
 * existing call sites in setup() may remain for documentation clarity.
 */

#pragma once

#include <TeensyThreads.h>
#include <Wire.h>

namespace I2CBus {

extern Threads::Mutex wire0;  ///< Wire  — TCA9548A, TCA9555, INA219
extern Threads::Mutex wire1;  ///< Wire1 — BNO085
extern Threads::Mutex wire2;  ///< Wire2 — PCA9685 #1 & #2

/// No-op: Threads::Mutex is self-initialising. Kept so existing call sites compile.
inline void initLocks() {}

/// Return the mutex for the given Wire bus instance.
/// Falls back to wire0 for any unrecognised bus.
Threads::Mutex& mutexFor(TwoWire& wire);

/// RAII scoped lock for an I2C bus.
struct Lock {
  explicit Lock(TwoWire& wire) : m_(mutexFor(wire)) { m_.lock(); }
  ~Lock() { m_.unlock(); }

  Lock(const Lock&)            = delete;
  Lock& operator=(const Lock&) = delete;

 private:
  Threads::Mutex& m_;
};

}  // namespace I2CBus
