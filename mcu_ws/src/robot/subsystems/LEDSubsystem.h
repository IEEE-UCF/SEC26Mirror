/**
 * @file LEDSubsystem.h
 * @brief WS2812B addressable RGB LED subsystem (FastLED).
 * @date 2026-02-25
 *
 * Drives a strip of WS2812B LEDs using FastLED on Teensy 4.x.
 * WS2812B output requires precise bit timing; context switches during
 * show() corrupt the signal, so interrupts are briefly disabled (~150µs
 * for 5 LEDs).
 *
 * -- ROS2 interface (defaults) ---------------------------------------------
 *   /mcu_robot/led/set_all   subscription   mcu_msgs/msg/LedColor
 *       Sets every LED to the same (r, g, b) colour.
 */

#pragma once

#include <FastLED.h>
#include <BaseSubsystem.h>
#include <mcu_msgs/msg/led_color.h>
#include <microros_manager_robot.h>

#ifdef USE_TEENSYTHREADS
#include <TeensyThreads.h>
#endif

namespace Subsystem {

// Maximum number of LEDs supported by this subsystem.  The CRGB array is
// statically sized; the actual count used at runtime comes from the setup.
static constexpr uint8_t LED_MAX_COUNT = 32;

class LEDSubsystemSetup : public Classes::BaseSetup {
 public:
  /**
   * @param _id      Subsystem identifier.
   * @param pin      Teensy GPIO pin connected to WS2812B data line.
   * @param numLeds  Number of LEDs on the strip (max LED_MAX_COUNT).
   * @param topic    ROS2 topic for colour commands.
   */
  LEDSubsystemSetup(const char* _id, uint8_t pin, uint8_t numLeds = 5,
                    const char* topic = "/mcu_robot/led/set_all")
      : Classes::BaseSetup(_id), pin_(pin), numLeds_(numLeds), topic_(topic) {}
  uint8_t pin_ = 0;
  uint8_t numLeds_ = 5;
  const char* topic_ = "/mcu_robot/led/set_all";
};

class LEDSubsystem : public IMicroRosParticipant,
                     public Classes::BaseSubsystem {
 public:
  explicit LEDSubsystem(const LEDSubsystemSetup& setup)
      : Classes::BaseSubsystem(setup), setup_(setup) {}

  bool init() override {
    num_leds_ = (setup_.numLeds_ <= LED_MAX_COUNT) ? setup_.numLeds_
                                                    : LED_MAX_COUNT;
    // Force pin 35 to GPIO output mode.  Serial8.begin() (called by IBusBM
    // for IBUS RC) sets pin 35's IOMUX to UART8-TX, stealing it from
    // FastLED.  IBUS is RX-only (pin 34), so releasing TX is safe.
    pinMode(35, OUTPUT);
    // FastLED requires a compile-time pin for addLeds<>.  Pin 35 matches
    // PIN_RGB_LEDS in RobotPins.h (the only WS2812B data pin on our PCB).
    // If the hardware pin changes, update this template argument to match.
    FastLED.addLeds<WS2812B, 35, GRB>(leds_, num_leds_);
    FastLED.clear(true);
    return true;
  }

  void begin() override {}

  void update() override {
    if (dirty_) {
#ifdef USE_TEENSYTHREADS
      // WS2812B bit-bang requires precise timing (~1.25µs per bit).
      // A TeensyThreads context switch mid-show() corrupts the signal.
      // Briefly disable interrupts for the duration of show() (~30µs/LED).
      noInterrupts();
      FastLED.show();
      interrupts();
#else
      FastLED.show();
#endif
      dirty_ = false;
    }
  }

  void pause() override {
    FastLED.clear(true);
  }

  void reset() override { pause(); }

  const char* getInfo() override {
    static const char info[] = "LEDSubsystem";
    return info;
  }

  bool onCreate(rcl_node_t* node, rclc_executor_t* executor) override {
    node_ = node;

    if (rclc_subscription_init_best_effort(
            &sub_, node_, ROSIDL_GET_MSG_TYPE_SUPPORT(mcu_msgs, msg, LedColor),
            setup_.topic_) != RCL_RET_OK) {
      return false;
    }

    if (rclc_executor_add_subscription_with_context(
            executor, &sub_, &msg_, &LEDSubsystem::ledCallback, this,
            ON_NEW_DATA) != RCL_RET_OK) {
      return false;
    }
    return true;
  }

  void onDestroy() override {
    sub_ = rcl_get_zero_initialized_subscription();
    node_ = nullptr;
  }

  void setAll(uint8_t r, uint8_t g, uint8_t b) {
    for (uint8_t i = 0; i < num_leds_; i++) {
      leds_[i] = CRGB(r, g, b);
    }
    dirty_ = true;
  }

  void setPixel(uint8_t idx, uint8_t r, uint8_t g, uint8_t b) {
    if (idx < num_leds_) {
      leds_[idx] = CRGB(r, g, b);
      dirty_ = true;
    }
  }

#ifdef USE_TEENSYTHREADS
  void beginThreaded(uint32_t stackSize, int /*priority*/ = 1,
                     uint32_t updateRateMs = 50) {
    task_delay_ms_ = updateRateMs;
    threads.addThread(taskFunction, this, stackSize);
  }

 private:
  static void taskFunction(void* pv) {
    auto* self = static_cast<LEDSubsystem*>(pv);
    self->begin();
    while (true) {
      self->update();
      threads.delay(self->task_delay_ms_);
    }
  }
  uint32_t task_delay_ms_ = 50;
#endif

 private:
  static void ledCallback(const void* msg, void* ctx) {
    auto* self = static_cast<LEDSubsystem*>(ctx);
    auto* led = static_cast<const mcu_msgs__msg__LedColor*>(msg);
    self->setAll(led->r, led->g, led->b);
  }

  const LEDSubsystemSetup setup_;
  CRGB leds_[LED_MAX_COUNT] = {};
  uint8_t num_leds_ = 0;
  bool dirty_ = false;

  rcl_subscription_t sub_{};
  mcu_msgs__msg__LedColor msg_{};
  rcl_node_t* node_ = nullptr;
};

}  // namespace Subsystem
