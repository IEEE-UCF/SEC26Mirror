/**
 * @file FreeRTOSCompat.h
 * @brief Thin C++ wrapper over FreeRTOS primitives.
 *
 * Provides drop-in replacements:
 *   Threads::Mutex       -> FRMutex
 *   Threads::Scope       -> FRMutex::ScopedLock
 *   threads.delay(ms)    -> frDelay(ms)
 *   threads.addThread()  -> frCreateTask()
 */

#pragma once

#include <FreeRTOS_TEENSY4.h>

class FRMutex {
 public:
  FRMutex() = default;
  ~FRMutex() {
    if (handle_) vSemaphoreDelete(handle_);
  }

  void init() {
    if (!handle_) handle_ = xSemaphoreCreateMutex();
  }

  void lock() {
    if (!handle_) handle_ = xSemaphoreCreateMutex();
    xSemaphoreTake(handle_, portMAX_DELAY);
  }

  void unlock() {
    if (handle_) xSemaphoreGive(handle_);
  }

  struct ScopedLock {
    explicit ScopedLock(FRMutex& m) : m_(m) { m_.lock(); }
    ~ScopedLock() { m_.unlock(); }
    ScopedLock(const ScopedLock&) = delete;
    ScopedLock& operator=(const ScopedLock&) = delete;
   private:
    FRMutex& m_;
  };

 private:
  SemaphoreHandle_t handle_ = nullptr;
  FRMutex(const FRMutex&) = delete;
  FRMutex& operator=(const FRMutex&) = delete;
};

inline void frDelay(uint32_t ms) {
  vTaskDelay(pdMS_TO_TICKS(ms));
}

inline BaseType_t frCreateTask(TaskFunction_t fn, const char* name,
                               uint32_t stackBytes, void* arg,
                               UBaseType_t priority,
                               TaskHandle_t* handle = nullptr) {
  return xTaskCreate(fn, name, stackBytes / 4, arg, priority, handle);
}
