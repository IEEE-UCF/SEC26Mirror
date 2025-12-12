#include "raw-rgb-analog-led.h"

namespace RawDrivers {

/// @brief initialize pins of the RGB LED
bool RGBAnalogLED::init() {
  pinMode(_setup.rgbPins_[0], OUTPUT);  // r
  pinMode(_setup.rgbPins_[1], OUTPUT);  // g
  pinMode(_setup.rgbPins_[2], OUTPUT);  // b

  analogWrite(_setup.rgbPins_[0], 0);  // r
  analogWrite(_setup.rgbPins_[1], 0);  // g
  analogWrite(_setup.rgbPins_[2], 0);  // b

  return true;
}

/// @brief  update color of the RGB LED
void RGBAnalogLED::update() {
  analogWrite(_setup.rgbPins_[0], _colorBuffer._r);  // r
  analogWrite(_setup.rgbPins_[1], _colorBuffer._g);  // g
  analogWrite(_setup.rgbPins_[2], _colorBuffer._b);  // b
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