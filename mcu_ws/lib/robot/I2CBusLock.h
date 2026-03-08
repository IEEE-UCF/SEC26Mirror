/**
 * @file I2CBusLock.h
 * @brief FreeRTOS mutex guards for the three I2C buses on Teensy 4.1.
 *
 * Uses FRMutex (FreeRTOS semaphore with priority inheritance) for
 * thread-safe bus access.  Each bus has a dedicated mutex so Wire0,
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
 */

#pragma once

#include <FreeRTOSCompat.h>
#include <Wire.h>

namespace I2CBus {

extern FRMutex wire0;  ///< Wire  — TCA9548A, TCA9555, INA219
extern FRMutex wire1;  ///< Wire1 — BNO085
extern FRMutex wire2;  ///< Wire2 — PCA9685 #1 & #2

/// Explicitly initialise the mutexes (safe to call before scheduler starts).
inline void initLocks() {
  wire0.init();
  wire1.init();
  wire2.init();
}

/// Return the mutex for the given Wire bus instance.
/// Falls back to wire0 for any unrecognised bus.
FRMutex& mutexFor(TwoWire& wire);

/// RAII scoped lock for an I2C bus.
struct Lock {
  explicit Lock(TwoWire& wire) : m_(mutexFor(wire)) { m_.lock(); locked_ = true; }
  ~Lock() { if (locked_) m_.unlock(); }

  /// Temporarily release the lock (e.g. during a long delay).
  void unlock() { if (locked_) { m_.unlock(); locked_ = false; } }
  /// Re-acquire after unlock().
  void relock() { if (!locked_) { m_.lock(); locked_ = true; } }

  Lock(const Lock&) = delete;
  Lock& operator=(const Lock&) = delete;

 private:
  FRMutex& m_;
  bool locked_ = false;
};

}  // namespace I2CBus
