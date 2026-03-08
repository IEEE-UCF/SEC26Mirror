/**
 * @file RTOSSubsystem.h
 * @brief RTOS-threaded subsystem base class — single beginThreaded()
 *        implementation via virtual dispatch, plus polling sub-timers.
 *
 * FreeRTOS backend (USE_FREERTOS) uses xTaskCreate / vTaskDelay.
 */
#pragma once

#include <Arduino.h>
#include <BaseSubsystem.h>
#include <elapsedMillis.h>

#if defined(USE_FREERTOS)
#include "FreeRTOSCompat.h"
#endif

// Debug serial: Teensy has dual USB serial (SerialUSB1 for debug); ESP32 uses Serial.
#if defined(__IMXRT1062__)
#define RTOS_DEBUG_SERIAL SerialUSB1
#else
#define RTOS_DEBUG_SERIAL Serial
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
    // Teensy FreeRTOS: stack depth in words (4 bytes); ESP32: bytes.
#if defined(__IMXRT1062__)
    const uint32_t stackDepth = stackSize / 4;
#else
    const uint32_t stackDepth = stackSize;
#endif
    BaseType_t rc = xTaskCreate(taskFunction, getInfo(), stackDepth,
                                static_cast<void*>(this), priority, &handle);
    if (rc != pdPASS || handle == nullptr) {
      RTOS_DEBUG_SERIAL.printf("[RTOS] FAIL xTaskCreate '%s' rc=%ld stack=%lu pri=%d heap=%lu\n",
                    getInfo(), (long)rc, stackSize, priority,
                    (unsigned long)xPortGetFreeHeapSize());
    } else {
      RTOS_DEBUG_SERIAL.printf("[RTOS] Task '%s' created OK (heap free=%lu)\n",
                    getInfo(), (unsigned long)xPortGetFreeHeapSize());
    }
  }

 private:
  static void taskFunction(void* pv) {
    auto* self = static_cast<RTOSSubsystem*>(pv);
    RTOS_DEBUG_SERIAL.printf("[RTOS] Task '%s' RUNNING (begin)\n", self->getInfo());
    self->begin();   // virtual dispatch
    RTOS_DEBUG_SERIAL.printf("[RTOS] Task '%s' begin() done, entering loop\n", self->getInfo());
    while (true) {
      self->update();  // virtual dispatch
      vTaskDelay(pdMS_TO_TICKS(self->task_delay_ms_));
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
