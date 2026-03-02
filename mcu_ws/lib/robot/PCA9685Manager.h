/**
 * @file PCA9685DMAManager.h
 * @brief DMA manager for multiple PCA9685 ICs on a shared I2C bus.
 *
 * Owns a single DMA channel!!! On update(), builds one combined MTDR
 * command buffer for all registered drivers. Only dirty channels are
 * written (selective write with auto-increment grouping), then fires
 * a single DMA transfer that updates every IC in one shot.
 */

#pragma once

#include <DMAChannel.h>
#include <Wire.h>

#include "PCA9685Driver.h"

class PCA9685DMAManager
{
public:
  static constexpr uint8_t MAX_DRIVERS = 4;

  // worst case per driver: 1 START + 1 REG + (16ch * 4 bytes) + 1 STOP = 67
  static constexpr uint16_t MAX_BUF_PER_DRIVER = 67;
  static constexpr uint16_t MAX_BUF_ENTRIES = MAX_BUF_PER_DRIVER * MAX_DRIVERS;

  explicit PCA9685DMAManager(TwoWire &wire = Wire2,
                             uint32_t clock_hz = 1000000);

  bool addDriver(PCA9685Driver *drv);

  uint8_t count() const { return num_drivers_; }
  PCA9685Driver *get(uint8_t i) { return i < num_drivers_ ? drivers_[i] : nullptr; }

  bool update(); // build selective buffer + fire DMA, non-blocking
  bool isComplete();

private:
  TwoWire &wire_;
  IMXRT_LPI2C_t *port_;
  uint8_t dma_src_;
  uint32_t clock_hz_;

  PCA9685Driver *drivers_[MAX_DRIVERS];
  uint8_t num_drivers_;

  DMAChannel dma_;
  uint16_t buf_[MAX_BUF_ENTRIES] __attribute__((aligned(32)));

  uint16_t buildForDriver(PCA9685Driver *drv, uint16_t *out);
};