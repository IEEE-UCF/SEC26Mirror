/**
 * @file FreeRTOSCompat.h
 * @brief FreeRTOS compatibility shim — drop-in replacements for TeensyThreads
 *        types (Threads::Mutex, Threads::Scope) and the global `threads` object.
 *
 * This header lets all existing subsystem code compile unchanged when
 * switching from TeensyThreads to FreeRTOS.  It provides:
 *   - Threads::Mutex   (wraps FreeRTOS xSemaphoreCreateRecursiveMutex)
 *   - Threads::Scope   (RAII lock guard)
 *   - global `threads`  object with delay() and addThread()
 *
 * Guarded by USE_FREERTOS so it is a no-op on ESP32 / native builds.
 */
#pragma once

#ifdef USE_FREERTOS

#if defined(__IMXRT1062__)  // Teensy 4.x — FreeRTOS via freertos-teensy library
#include "arduino_freertos.h"
#include "semphr.h"
#else  // ESP32 and other platforms — FreeRTOS is native
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#endif

// ---------------------------------------------------------------------------
//  Threads::Mutex  /  Threads::Scope  — API-compatible with TeensyThreads
// ---------------------------------------------------------------------------
namespace Threads {

class Mutex {
 public:
  Mutex() : handle_(xSemaphoreCreateRecursiveMutex()) {}
  ~Mutex() {
    if (handle_) vSemaphoreDelete(handle_);
  }

  void lock() {
    if (handle_) xSemaphoreTakeRecursive(handle_, portMAX_DELAY);
  }
  void unlock() {
    if (handle_) xSemaphoreGiveRecursive(handle_);
  }

  Mutex(const Mutex&) = delete;
  Mutex& operator=(const Mutex&) = delete;

 private:
  SemaphoreHandle_t handle_;
};

class Scope {
 public:
  explicit Scope(Mutex& m) : m_(m) { m_.lock(); }
  ~Scope() { m_.unlock(); }

  Scope(const Scope&) = delete;
  Scope& operator=(const Scope&) = delete;

 private:
  Mutex& m_;
};

}  // namespace Threads

// ---------------------------------------------------------------------------
//  Global `threads` object — API-compatible with TeensyThreads
// ---------------------------------------------------------------------------
struct ThreadsCompat {
  /** Yield-friendly delay (calls vTaskDelay). */
  void delay(uint32_t ms) { vTaskDelay(pdMS_TO_TICKS(ms)); }

  /**
   * Create a new FreeRTOS task.
   * @param fn         Task function (must never return).
   * @param arg        Argument passed to fn.
   * @param stackBytes Stack size in BYTES (converted to words internally).
   * @return TaskHandle_t (nullptr on failure).
   */
  TaskHandle_t addThread(void (*fn)(void*), void* arg = nullptr,
                         uint32_t stackBytes = 1024) {
    TaskHandle_t handle = nullptr;
    // Teensy FreeRTOS: stack depth in words (4 bytes); ESP32: bytes.
#if defined(__IMXRT1062__)
    const uint32_t depth = stackBytes / 4;
#else
    const uint32_t depth = stackBytes;
#endif
    xTaskCreate(reinterpret_cast<TaskFunction_t>(fn), "task", depth,
                arg, 1, &handle);
    return handle;
  }
};

inline ThreadsCompat threads;

#endif  // USE_FREERTOS
