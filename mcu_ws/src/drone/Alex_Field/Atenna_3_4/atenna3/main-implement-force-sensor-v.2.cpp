#include <Arduino.h>
//#include "DIYables_Keypad.h"//the DIYable_Keypad Library
#include "rgbled.h"
#include "forcesensor.h"
#define FORCE_SENSOR_PIN 27


int red = 5;
int green = 16;
int blue = 17;
int led_pin = 4;
rgbled rgb = rgbled(red,green,blue);
forcesensor forcesensepad = forcesensor(FORCE_SENSOR_PIN);
void setup()
{
  Serial.begin(9600);
  rgb.reset();
  pinMode(led_pin, OUTPUT);
  digitalWrite(led_pin, LOW);  
}

void loop()
{


  forcesensepad.update();
  if(forcesensepad.getStatus()==1)
  {
    rgb.setColor(3);
    digitalWrite(led_pin,HIGH);
  }
}
