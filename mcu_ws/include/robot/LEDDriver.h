#ifndef LEDDRIVER_H
#define LEDDRIVER_H

#include <Arduino.h>
#include <BaseDriver.h>
#include <stdint.h>

namespace Drivers {

class LEDDriver : public Classes::BaseDriver {
 public:
  explicit LEDDriver(uint8_t pin, bool startOn = false);

  bool init() override;
  void update() override;
  std::string getInfo() override;

  void on();
  void off();
  bool getState() const;

 private:
  uint8_t pin_;  // GPIO pin number
  bool state_;   // LED state (on/off)
};

}  // namespace Drivers

#endif
