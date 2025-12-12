#include "field/keypad.h"
#include "field/raw-rgb-analog-led.h"

const int ROW_NUM = 4;
const int COLUMN_NUM = 3;

char keys[ROW_NUM][COLUMN_NUM] = {
    {'1', '2', '3'}, {'4', '5', '6'}, {'7', '8', '9'}, {'*', '0', '#'}

};
// in ascending order
byte row_pins[ROW_NUM] =
    // R1,R2,R3,R4
    // 7,2,3,5
    {23, 22, 21, 19};  // look at a adafruit 3x4 phone-style matrix keypad(model
                       // 1824 or PID 1824)
byte column_pins[COLUMN_NUM] =
    // C1,C2,C3
    // 6,8,4
    {33, 25, 18};
int led_pin = 4;
int r = 5;
int g = 16;
int b = 17;

// RGB setup and class
RawDrivers::RGBAnalogLEDSetup setup1 =
    RawDrivers::RGBAnalogLEDSetup("Antenna4", {r, b, g});

RawDrivers::RGBAnalogLED RGB1 = RawDrivers::RGBAnalogLED(setup1);

// RGB colors
RawDrivers::RGBColor clearColor = RawDrivers::RGBColor(0, 0, 0);
RawDrivers::RGBColor red = RawDrivers::RGBColor(150, 0, 0);
RawDrivers::RGBColor green = RawDrivers::RGBColor(0, 150, 0);
RawDrivers::RGBColor blue = RawDrivers::RGBColor(0, 0, 100);
RawDrivers::RGBColor purple = RawDrivers::RGBColor(200, 150, 0);

Field::KeypadSetup setupKeypad = Field::KeypadSetup("Keypad");
Field::KeypadDriver Keypad1 = Field::KeypadDriver(
    setupKeypad, makeKeymap(keys), row_pins, column_pins, ROW_NUM, COLUMN_NUM);

int randomNum;
void setup() {
  Serial.begin(9600);
  RGB1.init();
  Keypad1.init();

  pinMode(led_pin, OUTPUT);

  randomNum = random(0, 3);
}
void loop() {
  Keypad1.update();

  if (Keypad1.getStatus()) {
    digitalWrite(led_pin, HIGH);

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