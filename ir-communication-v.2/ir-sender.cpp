#include "esp32-irNEC.hpp"
#include <Arduino.h>
IRSender *drone = NULL;

void setup() {
  Serial.begin(115200);
  Serial.printf(
      "Hello this is USB 0!!!\nWe're going to send some IR signals!\n");
  drone = new IRSender(GPIO_NUM_4);
}

void loop() {
  drone->send(antenna_three, red);
  delay(10);
}
