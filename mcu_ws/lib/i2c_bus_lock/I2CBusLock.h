/**
 * @file I2CBusLock.h
 * @brief RTOS mutex guards for the three I2C buses on Teensy 4.1.
 *
 * Uses Threads::Mutex (FreeRTOS recursive mutex via FreeRTOSCompat shim)
 * which is cooperative with the scheduler and avoids a raw busy-wait.
 * Each bus has a dedicated mutex so Wire0, Wire1, and Wire2 can be
 * driven concurrently by separate tasks.
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

#if defined(USE_FREERTOS)
#include "FreeRTOSCompat.h"
#endif
#include <Wire.h>

namespace I2CBus {

extern Threads::Mutex wire0;  ///< Wire  (Wire0)
extern Threads::Mutex wire1;  ///< Wire1
#if defined(__IMXRT1062__)     // Teensy 4.x has 3 I2C buses
extern Threads::Mutex wire2;  ///< Wire2 — PCA9685 #1 & #2
#endif

/// No-op: Threads::Mutex is self-initialising. Kept so existing call sites
/// compile.
inline void initLocks() {}

/// Return the mutex for the given Wire bus instance.
/// Falls back to wire0 for any unrecognised bus.
Threads::Mutex& mutexFor(TwoWire& wire);

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
  Threads::Mutex& m_;
  bool locked_ = false;
};

}  // namespace I2CBus
