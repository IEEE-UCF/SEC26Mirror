#include "esp32-I2C-LCD.h"

LCD::LCD(gpio_num_t sda_pin, gpio_num_t scl_pin, uint8_t i2c_address) {
  PIN_NUM_SDA = sda_pin;
  PIN_NUM_SCL = scl_pin;
  I2C_address = i2c_address;
  lcdQueue = xQueueCreate(10, sizeof(LCDMessage)); //"Buffer up" to 10 commands
}
void LCD::i2cinit() {
  // creating a a I2C bus, configuration time!
  // i2c_master_bus_handle_t i2c_bus = NULL;
  // i2c_master_bus_config_t bus_config = {
  //     .i2c_port = -1,
  //     .sda_io_num = PIN_NUM_SDA,
  //     .scl_io_num = PIN_NUM_SCL,
  //     .clk_source = I2C_CLK_SRC_DEFAULT,
  //     .glitch_ignore_cnt = 7,
  // };
  // bus_config.flags.enable_internal_pullup = 1;
  // ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &i2c_bus));

  // Configuration for LCD module
  lcd = {
      .write_cb = write_lcd_data, // use callback to send data to LCD by I2C
                                  // GPIO expander
      .font = HD44780_FONT_5X8,
      .lines = 2,
  };
  lcd.pins = {.rs = 0, .e = 2, .d4 = 4, .d5 = 5, .d6 = 6, .d7 = 7, .bl = 3};
  memset(&pcf8574, 0, sizeof(i2c_dev_t));

  ESP_ERROR_CHECK(pcf8574_init_desc(&pcf8574, I2C_NUM_0, I2C_address,
                                    PIN_NUM_SDA, PIN_NUM_SCL));
  ESP_ERROR_CHECK(hd44780_init(&lcd));
  hd44780_switch_backlight(&lcd, true);

  BaseType_t taskStatus = xTaskCreate(lcdTask, "lcd_task", 3072, this, 3, NULL);
  if (taskStatus != pdPASS) {
    printf("LCD Error: Could not create background task!\n");
  }
}
//we got a callback function here...I guess to see if we can even connect to the bus of PCF8574??
esp_err_t LCD::write_lcd_data(const hd44780_t *lcd, uint8_t data) {
  // return pcf8574_port_write(&pcf8574, data);
  // i2c_dev_t *dev = (i2c_dev_t *)lcd.;
  esp_err_t err = pcf8574_port_write(&pcf8574, data);
  if (err != ESP_OK) {
    printf("I2C Write Error: %s\n", esp_err_to_name(err));
  }
  return err;
}
//this is needed so to make our tasks non-blocking like clear, print, setCursor we need to be able to put them in the background
//hopefully this works
void LCD::lcdTask(void *pvParameters) {
  LCD *instance = (LCD *)pvParameters;
  LCDMessage msg;
  while (1) {
    if (xQueueReceive(instance->lcdQueue, &msg, portMAX_DELAY)) {
      switch (msg.cmd) {
      case LCDMessage::PRINT:
        hd44780_puts(&(instance->lcd),
                     msg.text); // blocking call????what does that mean?
        break;
      case LCDMessage::CLEAR:
        hd44780_clear(&(instance->lcd));
        break;
      case LCDMessage::SET_CURSOR:
        hd44780_gotoxy(&(instance->lcd), msg.x, msg.y);
        break;
      }
    }
  }
}
//all of the functions below will simply place a "state" for the background task
//so if we need the lCD to clear for example, it'll leave it as a background task let the esp do its main operation
//and then conduct the task..which happens pretty fast!

// clears the screen and sets cursor to (0,0)
void LCD::clear() {
  LCDMessage msg;
  msg.cmd = LCDMessage::CLEAR;
  xQueueSend(lcdQueue, &msg, pdMS_TO_TICKS(10));
  // hd44780_clear(&lcd);
}
//Write NULL-terminated string at cursor position
void LCD::print(std::string message) {
  LCDMessage msg;
  msg.cmd = LCDMessage::PRINT;
  strncpy(msg.text, message.c_str(), sizeof(msg.text));
  // xQueueSend(lcdQueue,&msg,0);
  msg.text[sizeof(msg.text) - 1] = '\0'; // every array always ends with \0 but
                                         // i guess we check size here.....
  // hd44780_puts(&lcd, message.c_str());
  if (xQueueSend(lcdQueue, &msg, pdMS_TO_TICKS(10)) != pdPASS) {
    printf("LCD WarningL Task Queue FULL, command dropped!");
  }
}
//we got a 16x2 HD44780 LCD, so 0-15 x_positions, 0-1 y_positions
void LCD::setCursor(uint8_t x_position, uint8_t y_position) {
  // hd44780_gotoxy(&lcd,x_position,y_position);
  LCDMessage msg;
  msg.cmd = LCDMessage::SET_CURSOR;
  msg.x = x_position;
  msg.y = y_position;
  xQueueSend(lcdQueue, &msg, pdMS_TO_TICKS(10));
}
