#include "raw-rgb-analog-led.h"

namespace RawDrivers {

/// @brief initialize pins of the RGB LED
bool RGBAnalogLED::init() {
  // Set up LEDC PWM channels for ESP32 (channels 0, 1, 2)
  ledcSetup(0, 5000, 8);  // channel 0, 5kHz, 8-bit resolution
  ledcSetup(1, 5000, 8);  // channel 1
  ledcSetup(2, 5000, 8);  // channel 2

  // Attach pins to channels
  ledcAttachPin(_setup.rgbPins_[0], 0);  // r -> channel 0
  ledcAttachPin(_setup.rgbPins_[1], 1);  // g -> channel 1
  ledcAttachPin(_setup.rgbPins_[2], 2);  // b -> channel 2

  // Initialize to off
  ledcWrite(0, 0);  // r
  ledcWrite(1, 0);  // g
  ledcWrite(2, 0);  // b

  return true;
}

/// @brief  update color of the RGB LED
void RGBAnalogLED::update() {
  ledcWrite(0, _colorBuffer._r);  // r
  ledcWrite(1, _colorBuffer._g);  // g
  ledcWrite(2, _colorBuffer._b);  // b
}

/// @brief  get information on the RGB LED
const char* RGBAnalogLED::getInfo() {
  return ("\nID: " + std::string(setup_.getId()) + "\nRed: pin-" +
          std::to_string(_setup.rgbPins_[0]) + " strength-" +
          std::to_string(_colorBuffer._r) + "\nGreen: pin-" +
          std::to_string(_setup.rgbPins_[1]) + " strength-" +
          std::to_string(_colorBuffer._g) + "\nBlue: pin-" +
          std::to_string(_setup.rgbPins_[2]) + " strength-" +
          std::to_string(_colorBuffer._b))
      .c_str();
}

/// @brief set color of the RGB LED
void RGBAnalogLED::setColor(RGBColor& color) { _colorBuffer = color; }

};  // namespace RawDrivers
