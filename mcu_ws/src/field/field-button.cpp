#include <button.h>
#include <raw-rgb-analog-led.h>

// pin configs
const uint8_t ledPin = 35;
const uint8_t swPin = 5;

const std::array<uint8_t, 3> ryg = {26, 25, 33};

const uint8_t r = 15;
const uint8_t g = 2;
const uint8_t b = 4;

// RGB setup and class
RawDrivers::RGBAnalogLEDSetup setupRGB =
    RawDrivers::RGBAnalogLEDSetup("Antenna1", {r, g, b});

RawDrivers::RGBAnalogLED RGB1 = RawDrivers::RGBAnalogLED(setupRGB);

// RGB colors
RawDrivers::RGBColor clearColor = RawDrivers::RGBColor(0, 0, 0);
RawDrivers::RGBColor red = RawDrivers::RGBColor(150, 0, 0);
RawDrivers::RGBColor green = RawDrivers::RGBColor(0, 150, 0);
RawDrivers::RGBColor blue = RawDrivers::RGBColor(0, 0, 100);
RawDrivers::RGBColor purple = RawDrivers::RGBColor(200, 150, 0);

// button configuration
Field::ButtonSetup setupButton =
    Field::ButtonSetup("Button", ryg, ledPin, swPin);
Field::ButtonDriver Button1 = Field::ButtonDriver(setupButton);

int randomNum;  // for random number gen

void setup() {
  // initialize
  Button1.init();
  RGB1.init();
  randomNum = random(0, 3);
}

void loop() {
  Button1.update();  // call button functionality

  // output a color if task completed
  if (Button1.getStatus()) {
    switch (randomNum) {
      case 1:
        RGB1.setColor(red);
        RGB1.update();
        break;
      case 2:
        RGB1.setColor(blue);
        RGB1.update();
        break;
      case 3:
        RGB1.setColor(green);
        RGB1.update();
        break;
      default:
        RGB1.setColor(purple);
        RGB1.update();
        break;
    }
  }
}