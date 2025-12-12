#include "field/crank.h"
#include "field/raw-rgb-analog-led.h"

// pin configs
const uint8_t ledPin = 13;
const uint8_t clk_k = 25;
const uint8_t dt_k = 33;

const uint8_t r = 27;
const uint8_t b = 26;
const uint8_t g = 14;

// RGB setup and class
RawDrivers::RGBAnalogLEDSetup setup1 =
    RawDrivers::RGBAnalogLEDSetup("Antenna2", {r, b, g});

RawDrivers::RGBAnalogLED RGB1 = RawDrivers::RGBAnalogLED(setup1);

// RGB colors
RawDrivers::RGBColor clearColor = RawDrivers::RGBColor(0, 0, 0);
RawDrivers::RGBColor red = RawDrivers::RGBColor(150, 0, 0);
RawDrivers::RGBColor green = RawDrivers::RGBColor(0, 150, 0);
RawDrivers::RGBColor blue = RawDrivers::RGBColor(0, 0, 100);
RawDrivers::RGBColor purple = RawDrivers::RGBColor(200, 150, 0);

// crank configuration
Field::CrankSetup setupCrank = Field::CrankSetup("Crank", ledPin, clk_k, dt_k);
Field::CrankDriver Crank1 = Field::CrankDriver(setupCrank);

int randomNum;  // for random number gen

void setup() {
  // initialize
  Crank1.init();
  RGB1.init();
  randomNum = random(0, 3);
}

void loop() {
  Crank1.update();  // call crank functionality

  // output a color if task completed
  if (Crank1.getStatus()) {
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