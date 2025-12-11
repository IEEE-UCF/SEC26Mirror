/**
 * @file raw-rgb-analog-led.h
 * @date 12/8/25
 * @author Aldem Pido
 * @brief Raw RGB Led Pins
 */
#ifndef RAW_RGB_ANALOG_LED_H
#define RAW_RGB_ANALOG_LED_H

#include <Arduino.h>
#include <BaseDriver.h>

#include <array>

namespace RawDrivers {
// color values for RGB
class RGBColor {
 public:
  ~RGBColor() = default;
  RGBColor() = delete;
  RGBColor(const uint8_t r, const uint8_t g, const uint8_t b)
      : r(r), g(g), b(b) {}
  const uint8_t r;
  const uint8_t g;
  const uint8_t b;
};
// setup for RGB LED
class RGBAnalogLEDSetup : public Classes::BaseSetup {
 public:
  ~RGBAnalogLEDSetup() = default;
  RGBAnalogLEDSetup() = delete;

  /**
   * @brief Constructor for RGBAnalogLED
   * @param _id A unique ID for the rgb leds
   * @param pins 3 raw esp32 pins, r, g, b
   */
  RGBAnalogLEDSetup(const char* _id, const std::array<uint8_t, 3>& pins)
      : Classes::BaseSetup(_id), rgbPins_(pins) {}
  const std::array<uint8_t, 3> rgbPins_;  // r, g, b
};

// main RGB LED class
class RGBAnalogLED : public Classes::BaseDriver {
 public:
  RGBAnalogLED(const RGBAnalogLEDSetup& setup);  // constructor
  bool init() override;                          // initialize LED
  void update() override;                        // update LED in some way??
  const char* getInfo() override;                // get info on LED

  void setColor(RGBColor color);  // set LED color

 private:
  const RGBAnalogLEDSetup& setup_;  // setup information
  char infoBuf_[96];                // ?? holds information in string
  RGBColor colorBuffer;             // holds LED color
};
};  // namespace RawDrivers

#endif