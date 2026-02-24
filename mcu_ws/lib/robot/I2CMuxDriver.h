/**
 * @file I2CMuxDriver.h
 * @author Trevor Cannon
 * @brief TCA9548A I2C mux driver wrapper.
 * @date 12/17/2025 (updated 2026-02-23)
 *
 * selectChannel(), deselectAll(), and isChannelEnabled() do NOT acquire the
 * bus lock — callers are responsible for holding I2CBus::Lock(wire) before
 * calling them.  init() acquires the lock internally since it runs standalone.
 */

#ifndef I2CMUXDRIVER_H
#define I2CMUXDRIVER_H

#include <BaseDriver.h>
#include <TCA9548.h>
#include <Wire.h>

#include "I2CBusLock.h"

namespace Drivers {

class I2CMuxDriverSetup : public Classes::BaseSetup {
 public:
  ~I2CMuxDriverSetup() = default;
  I2CMuxDriverSetup() = delete;

  /**
   * @param _id          Driver identifier string.
   * @param _i2cAddress  TCA9548A I2C address (default 0x70, range 0x70–0x77).
   * @param wire         I2C bus the mux is on (default Wire / Wire0).
   */
  I2CMuxDriverSetup(const char* _id, uint8_t _i2cAddress = 0x70,
                    TwoWire& wire = Wire)
      : Classes::BaseSetup(_id), i2cAddress_(_i2cAddress), wire_(wire) {}

  const uint8_t i2cAddress_;
  TwoWire&      wire_;
};

class I2CMuxDriver : public Classes::BaseDriver {
 public:
  ~I2CMuxDriver() override = default;

  explicit I2CMuxDriver(const I2CMuxDriverSetup& setup)
      : Classes::BaseDriver(setup),
        mux_(setup.i2cAddress_, &setup.wire_),
        setup_(setup),
        currentChannel_(255) {}

  bool        init()    override;
  void        update()  override {}
  const char* getInfo() override;

  uint8_t getCurrentChannel() const { return currentChannel_; }

  bool isChannelEnabled(uint8_t channel);
  bool selectChannel(uint8_t channel);
  bool deselectAll();

 private:
  TCA9548                   mux_;
  const I2CMuxDriverSetup&  setup_;
  char                      infoBuf_[64] = {};
  uint8_t                   currentChannel_;
};

}  // namespace Drivers

#endif
