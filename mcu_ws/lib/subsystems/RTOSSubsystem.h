/**
 * @file RTOSSubsystem.h
 * @brief RTOS-threaded subsystem base class — single beginThreaded()
 *        implementation via virtual dispatch, plus polling sub-timers.
 *
 * FreeRTOS backend (USE_FREERTOS) uses xTaskCreate / vTaskDelay.
 * TeensyThreads backend (USE_TEENSYTHREADS) kept for backward compatibility.
 */
#pragma once

#include <Arduino.h>
#include <BaseSubsystem.h>
#include <elapsedMillis.h>

#if defined(USE_FREERTOS)
#include "FreeRTOSCompat.h"
#elif defined(USE_TEENSYTHREADS)
#include <TeensyThreads.h>
#endif

namespace Subsystem {

class RTOSSubsystem : public Classes::BaseSubsystem {
 public:
  explicit RTOSSubsystem(const Classes::BaseSetup& setup)
      : Classes::BaseSubsystem(setup) {}

#if defined(USE_FREERTOS)
  void beginThreaded(uint32_t stackSize, int priority = 1,
                     uint32_t updateRateMs = 20) {
    task_delay_ms_ = updateRateMs;
    TaskHandle_t handle = nullptr;
    // FreeRTOS stack depth is in words (4 bytes); callers pass bytes.
    BaseType_t rc = xTaskCreate(taskFunction, getInfo(), stackSize / 4,
                                static_cast<void*>(this), priority, &handle);
    if (rc != pdPASS || handle == nullptr) {
      SerialUSB1.printf("[RTOS] FAIL xTaskCreate '%s' rc=%ld stack=%lu pri=%d heap=%lu\n",
                    getInfo(), (long)rc, stackSize, priority,
                    (unsigned long)xPortGetFreeHeapSize());
    } else {
      SerialUSB1.printf("[RTOS] Task '%s' created OK (heap free=%lu)\n",
                    getInfo(), (unsigned long)xPortGetFreeHeapSize());
    }
  }

 private:
  static void taskFunction(void* pv) {
    auto* self = static_cast<RTOSSubsystem*>(pv);
    SerialUSB1.printf("[RTOS] Task '%s' RUNNING (begin)\n", self->getInfo());
    self->begin();   // virtual dispatch
    SerialUSB1.printf("[RTOS] Task '%s' begin() done, entering loop\n", self->getInfo());
    while (true) {
      self->update();  // virtual dispatch
      vTaskDelay(pdMS_TO_TICKS(self->task_delay_ms_));
    }
  }
  uint32_t task_delay_ms_ = 20;

#elif defined(USE_TEENSYTHREADS)
  void beginThreaded(uint32_t stackSize, int priority = 1,
                     uint32_t updateRateMs = 20) {
    task_delay_ms_ = updateRateMs;
    threads.addThread(taskFunction, static_cast<void*>(this), stackSize);
  }

 private:
  static void taskFunction(void* pv) {
    auto* self = static_cast<RTOSSubsystem*>(pv);
    self->begin();   // virtual dispatch
    while (true) {
      self->update();  // virtual dispatch
      threads.delay(self->task_delay_ms_);
    }
  }
  uint32_t task_delay_ms_ = 20;
#endif

 protected:
  // Up to 4 independent polling sub-timers per subsystem (absorbed from
  // TimedSubsystem)
  static constexpr uint8_t kMaxTimers = 4;

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
  elapsedMicros elapsed_[kMaxTimers] = {};
};

}  // namespace Subsystem
