/**
 * @file TimedSubsystem.h
 * @brief A thin wrapper over BaseSubsystem that provides simple periodic
 * timers.
 */
#pragma once

#include <Arduino.h>
#include <BaseSubsystem.h>
#include <elapsedMillis.h>

namespace Subsystem {

class TimedSubsystem : public Classes::BaseSubsystem {
 public:
  explicit TimedSubsystem(const Classes::BaseSetup& setup)
      : Classes::BaseSubsystem(setup) {}

 protected:
  // Up to 4 independent timers per subsystem; index selects which
  bool everyUs(uint32_t period_us, uint8_t index = 0) {
    if (index >= kMaxTimers) return false;
    if (period_us == 0) return true;
    if (elapsed_[index] > period_us) {
      elapsed_[index] = 0;
      return true;
    }
    return false;
  }

  bool everyMs(uint32_t period_ms, uint8_t index = 0) {
    return everyUs(period_ms * 1000U, index);
  }

 private:
  static constexpr uint8_t kMaxTimers = 4;
  elapsedMicros elapsed_[kMaxTimers] = {};
};

}  // namespace Subsystem
