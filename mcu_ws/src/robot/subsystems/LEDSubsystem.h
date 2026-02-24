/**
 * @file LEDSubsystem.h
 * @brief WS2812B addressable RGB LED subsystem.
 * @date 2026-02-24
 *
 * Drives a strip of WS2812B LEDs and exposes micro-ROS control.
 *
 * -- ROS2 interface (defaults) ---------------------------------------------
 *   /mcu_robot/led/set_all   subscription   mcu_msgs/msg/LedColor
 *       Sets every LED to the same (r, g, b) colour.
 */

#pragma once

#include <Adafruit_NeoPixel.h>
#include <BaseSubsystem.h>
#include <mcu_msgs/msg/led_color.h>
#include <microros_manager_robot.h>

#ifdef USE_TEENSYTHREADS
#include <TeensyThreads.h>
#endif

namespace Subsystem {

class LEDSubsystemSetup : public Classes::BaseSetup {
 public:
  /**
   * @param _id      Subsystem identifier.
   * @param pin      Teensy GPIO pin connected to WS2812B data line.
   * @param numLeds  Number of LEDs on the strip.
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
      : Classes::BaseSubsystem(setup),
        setup_(setup),
        strip_(setup.numLeds_, setup.pin_, NEO_GRB + NEO_KHZ800) {}

  bool init() override {
    strip_.begin();
    strip_.clear();
    strip_.show();
    return true;
  }

  void begin() override {}

  void update() override {
    if (dirty_) {
#ifdef USE_TEENSYTHREADS
      noInterrupts();
      strip_.show();
      interrupts();
#else
      strip_.show();
#endif
      dirty_ = false;
    }
  }

  void pause() override {
    strip_.clear();
    strip_.show();
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
    for (uint16_t i = 0; i < setup_.numLeds_; i++) {
      strip_.setPixelColor(i, strip_.Color(r, g, b));
    }
    dirty_ = true;
  }

  void setPixel(uint8_t idx, uint8_t r, uint8_t g, uint8_t b) {
    if (idx < setup_.numLeds_) {
      strip_.setPixelColor(idx, strip_.Color(r, g, b));
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
  Adafruit_NeoPixel strip_;
  bool dirty_ = false;

  rcl_subscription_t sub_{};
  mcu_msgs__msg__LedColor msg_{};
  rcl_node_t* node_ = nullptr;
};

}  // namespace Subsystem
