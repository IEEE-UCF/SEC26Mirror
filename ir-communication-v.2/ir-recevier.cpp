#include "esp32-irNEC.hpp"
#include <Arduino.h>
IRReceiver *earth = NULL;

void setup() {
  Serial.begin(115200);
  Serial.printf("This is USB 1!\nWe're going to be receving IR signals!!");
  earth = new IRReceiver(GPIO_NUM_4);
}
void loop() {
  uint16_t address = 0;
  uint16_t command = 0;

  // checkMessage returns TRUE only if a NEW packet is in the queue
  if (earth->checkMessage(address, command)) {

    // FIX: Use 'address' and 'command', NOT '0'!
    uint8_t clean_addr = address & 0xFF;
    uint8_t clean_cmd = command & 0xFF;

    Serial.printf("NEW SIGNAL -> Address: 0x%02X (%d) | Command: 0x%02X (%d)\n",
                  clean_addr, clean_addr, clean_cmd, clean_cmd);
  }
  // We removed the 'else' block so it doesn't spam "welp" when idle

  delay(60);
}