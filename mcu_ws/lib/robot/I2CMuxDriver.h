/**
 * @file I2CMuxDriver.h
 * @author Trevor Cannon
 * @brief TCA95448 I2C Mux Wrapper
 * @date 12/17/2025
 */

#ifndef I2CMUXDRIVER_H
#define I2CMUXDRIVER_H

#include <BaseDriver.h>
#include <TCA9548.h>

namespace Drivers {

class I2CMuxDriverSetup : public Classes::BaseSetup {
 public:
  const uint8_t i2cAddress_;

  ~I2CMuxDriverSetup() = default;
  I2CMuxDriverSetup(const char* _id, uint8_t _i2cAddress = 0x70)
      : Classes::BaseSetup(_id), i2cAddress_(_i2cAddress) {}
};

class I2CMuxDriver : public Classes::BaseDriver {
 public:
  ~I2CMuxDriver() override = default;
  I2CMuxDriver(const I2CMuxDriverSetup& setup)
      : Classes::BaseDriver(setup),
        mux(setup.i2cAddress_),
        setup_(setup),
        currentChannel_(255) {}

  bool init() override;
  void update() override;
  const char* getInfo() override;

  uint8_t getCurrentChannel() const { return currentChannel_; }

  bool isChannelEnabled(uint8_t channel);
  bool selectChannel(uint8_t channel);
  bool deselectAll();

 private:
  TCA9548 mux;
  const I2CMuxDriverSetup& setup_;
  char infoBuf_[64];

  uint8_t currentChannel_;
};
};  // namespace Drivers

#endif