/**
 * @file BH1750Driver.h
 * @brief BH1750 ambient light sensor driver, optionally routed through a
 *        TCA9548A I2C multiplexer.
 *
 * The BH1750 is a simple 2-byte-read I2C light sensor.
 *   - Power on:  write 0x01
 *   - Continuous high-resolution mode: write 0x10 (1 lx, 120ms)
 *   - Read: 2 bytes, lux = raw / 1.2
 *
 * Supports both blocking I2C (update()) and DMA (queueDMAReads /
 * processDMAResults) following the same pattern as I2CPowerDriver.
 */

#ifndef BH1750DRIVER_H
#define BH1750DRIVER_H

#include <Arduino.h>
#include <BaseDriver.h>
#include <Wire.h>

#include "I2CBusLock.h"

class I2CDMABus;

namespace Drivers {

class I2CMuxDriver;

class BH1750DriverSetup : public Classes::BaseSetup {
 public:
  ~BH1750DriverSetup() = default;
  BH1750DriverSetup() = delete;

  /**
   * @param _id         Driver identifier string.
   * @param address     BH1750 I2C address (0x23 ADDR=LOW, 0x5C ADDR=HIGH).
   * @param mux         Pointer to an initialised I2CMuxDriver, or nullptr.
   * @param muxChannel  TCA9548A channel (0-7) the BH1750 is behind.
   * @param wire        I2C bus the BH1750 is on.
   */
  BH1750DriverSetup(const char* _id, uint8_t address = 0x23,
                    I2CMuxDriver* mux = nullptr, uint8_t muxChannel = 0,
                    TwoWire& wire = Wire)
      : Classes::BaseSetup(_id),
        address_(address),
        mux_(mux),
        muxChannel_(muxChannel),
        wire_(wire) {}

  const uint8_t address_;
  I2CMuxDriver* mux_;
  const uint8_t muxChannel_;
  TwoWire& wire_;
};

class BH1750Driver : public Classes::BaseDriver {
 public:
  explicit BH1750Driver(const BH1750DriverSetup& setup)
      : BaseDriver(setup), setup_(setup) {}

  ~BH1750Driver() override = default;

  bool init() override;
  void update() override;

  float getLux() const { return lux_; }

  const char* getInfo() override;

  // DMA support
  void setDMABus(I2CDMABus* bus) { dma_bus_ = bus; }
  void queueDMAReads();
  void processDMAResults();

 private:
  // BH1750 command opcodes
  static constexpr uint8_t CMD_POWER_ON = 0x01;
  static constexpr uint8_t CMD_CONTINUOUS_HIGH_RES = 0x10;

  bool sendCommand(uint8_t cmd);

  const BH1750DriverSetup setup_;
  float lux_ = 0.0f;
  char infoBuf_[64] = {};

  I2CDMABus* dma_bus_ = nullptr;
  uint8_t dma_rx_[2] = {};
};

}  // namespace Drivers

#endif
