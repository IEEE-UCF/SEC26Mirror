/*
Intialize I2C interface
Intialize Panel I/O Handle
Configure Panel Device
Create Panel Handle
Reset Panel
Initialize Panel(Controller Init Commands)
Turn Display On/ Backlight On

Since our esp32 is sending data to the I2C module it is considerded as
a "master"
https://github.com/UncleRus/esp-idf-lib/blob/master/examples/hd44780/i2c/main/main.c
freeRTOS is meant for the "non-blocking" standards could probably start
implementing these for other tasks????

*/
#ifndef ESP32_i2c_LCD_H
#define ESP32_i2c_LCD_H
#include "driver/gpio.h" //pins
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <inttypes.h>
#include <iostream>
#include <sys/time.h>

#ifdef __cplusplus
extern "C" {
#endif
#include "hd44780.h"
#include "pcf8574.h"
#include <string.h>
#ifdef __cplusplus
}
#endif
struct LCDMessage {
  enum Command { PRINT, CLEAR, SET_CURSOR } cmd;
  char text[32];
  uint8_t x, y;
};
class LCD {
public:
  // constructor
  LCD(gpio_num_t sda_pin, gpio_num_t scl_pin, uint8_t i2c_address);
  void i2cinit(); // intialize the I2C bus

  // Operations
  void clear();
  void print(std::string message);
  void setCursor(uint8_t, uint8_t);

private:
  static esp_err_t write_lcd_data(const hd44780_t *lcd, uint8_t data);
  static void lcdTask(void *pvParameters);
  // we need the queue to make our tasks into background tasks.....for
  // non-blocking purposes
  QueueHandle_t lcdQueue;
  hd44780_t lcd;
  static i2c_dev_t pcf8574;
  uint8_t I2C_address;
  gpio_num_t PIN_NUM_SDA;
  gpio_num_t PIN_NUM_SCL;
};
#endif