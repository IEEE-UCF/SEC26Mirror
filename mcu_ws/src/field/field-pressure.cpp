
#include "field/pressure.h"
#include "field/raw-rgb-analog-led.h"

#define FORCE_SENSOR_PIN 27

// CHANGE THIS
int r = 5;
int g = 16;
int b = 17;

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

forcesensor forcesensepad = forcesensor(FORCE_SENSOR_PIN);

int randomNum;  // for random number gen

void setup() {
  Serial.begin(9600);

  RGB1.init();
  randomNum = random(0, 3);
}

void loop() {
  forcesensepad.update();
  if (forcesensepad.getStatus() == 1) {
    Serial.println("HOORAY!");

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
  // int analogReading = analogRead(FORCE_SENSOR_PIN);

  // Serial.print("Force sensor reading = ");
  // Serial.print(analogReading); // print the raw analog reading

  // if (analogReading < 10)       // from 0 to 9
  //   Serial.println(" -> no pressure");
  // else if (analogReading < 200) // from 10 to 199
  //   Serial.println(" -> light touch");
  // else if (analogReading < 500) // from 200 to 499
  //   Serial.println(" -> light squeeze");
  // else if (analogReading < 800) // from 500 to 799
  //   Serial.println(" -> medium squeeze");
  // else // from 800 to 1023
  //   Serial.println(" -> big squeeze");

  // delay(25);

  //  rgb.setColor(0);
  // rgb.reset();
  //  digitalWrite(led_pin,HIGH);
  //  delay(300);
  // keypad.update();
  // if(keypad.getStatus()==1)
  // {
  //   Serial.println("Task is completed!");
  //   digitalWrite(led_pin,HIGH);
  //   rgb.setColor(randomNumber);
  // rgb.setColor(3);
  //}else{
  // Serial.println("Task is not finished!");
  // Serial.println("Task is not completed!");
  // digitalWrite(led_pin,LOW);
  // }
}
