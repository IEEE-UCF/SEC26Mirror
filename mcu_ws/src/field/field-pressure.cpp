
#include "field/pressure.h"
#include "field/raw-rgb-analog-led.h"

const uint8_t forcePin = 27;

const uint8_t r = 5;
const uint8_t g = 16;
const uint8_t b = 17;
const uint8_t ledPin = 4;

// RGB setup and class
RawDrivers::RGBAnalogLEDSetup setup1 =
    RawDrivers::RGBAnalogLEDSetup("Antenna3", {r, b, g});

RawDrivers::RGBAnalogLED RGB1 = RawDrivers::RGBAnalogLED(setup1);

// RGB colors
RawDrivers::RGBColor clearColor = RawDrivers::RGBColor(0, 0, 0);
RawDrivers::RGBColor red = RawDrivers::RGBColor(150, 0, 0);
RawDrivers::RGBColor green = RawDrivers::RGBColor(0, 150, 0);
RawDrivers::RGBColor blue = RawDrivers::RGBColor(0, 0, 100);
RawDrivers::RGBColor purple = RawDrivers::RGBColor(200, 150, 0);

// pressure configuration
Field::PressureSetup setupPressure = Field::PressureSetup("Pressure", forcePin);
Field::PressureDriver Pressure1 = Field::PressureDriver(setupPressure);

int randomNum;  // for random number gen

void setup() {
  Serial.begin(9600);
  RGB1.init();

  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);

  randomNum = random(0, 3);
}

void loop() {
  Pressure1.update();
  if (Pressure1.getStatus()) {
    digitalWrite(ledPin, HIGH);

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
