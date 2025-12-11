#include "raw-rgb-analog-led.h"

// main constructor for LEDs
RawDrivers::RGBAnalogLED::RGBAnalogLED(
    const RawDrivers::RGBAnalogLEDSetup& setup)
    : setup_(setup),                               // LED setup information
      infoBuf_(""),                                // Information string???
      colorBuffer(RawDrivers::RGBColor(0, 0, 0)),  // Set color initially??
      BaseDriver(setup)                            // assigns the base driver
{}

/// @brief initialize pins of the RGB LED
bool RawDrivers::RGBAnalogLED::init() {
  pinMode(setup_.rgbPins_[0], OUTPUT);  // r
  pinMode(setup_.rgbPins_[1], OUTPUT);  // g
  pinMode(setup_.rgbPins_[2], OUTPUT);  // b

  analogWrite(setup_.rgbPins_[0], 0);  // r
  analogWrite(setup_.rgbPins_[1], 0);  // g
  analogWrite(setup_.rgbPins_[2], 0);  // b

  return true;
}

/// @brief  update color of the RGB LED???
void RawDrivers::RGBAnalogLED::update() {
  analogWrite(setup_.rgbPins_[0], colorBuffer.r);  // r
  analogWrite(setup_.rgbPins_[1], colorBuffer.g);  // g
  analogWrite(setup_.rgbPins_[2], colorBuffer.b);  // b

  // something on updating infoBuf_
}

/// @brief  get information on the RGB LED
const char* RawDrivers::RGBAnalogLED::getInfo() { return infoBuf_; }

/// @brief  set color of the RGB LED
void RawDrivers::RGBAnalogLED::setColor(RGBColor color) {
  // colorBuffer = color; // rip need to figure this out
}
