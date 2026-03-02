/**
 * @file PCA9685Manager.h
 * @brief PCA9685 command builder for the shared I2CDMABus.
 *
 * No longer owns a DMA channel. Instead, call setDMABus() to link to the
 * shared I2CDMABus, then call buildInto() each cycle to append selective
 * PCA9685 writes into the bus's TX buffer.
 *
 * Legacy mode: if no DMA bus is set, update() still works as a standalone
 * DMA manager (backwards compatible).
 */

#pragma once

#include <DMAChannel.h>
#include <Wire.h>

#include "I2CDMABus.h"
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

  // ── Shared DMA bus mode ────────────────────────────────────────────
  void setDMABus(I2CDMABus *bus) { dma_bus_ = bus; }

  /// Append PCA9685 selective writes into the shared I2CDMABus TX buffer.
  /// Call this after dispatch() and before fire() each cycle.
  /// Returns number of MTDR words written (0 if nothing dirty).
  uint16_t buildInto();

  // ── Legacy standalone mode ─────────────────────────────────────────
  bool update(); // build selective buffer + fire DMA, non-blocking
  bool isComplete();

private:
  TwoWire &wire_;
  IMXRT_LPI2C_t *port_;
  uint8_t dma_src_;
  uint32_t clock_hz_;

  PCA9685Driver *drivers_[MAX_DRIVERS];
  uint8_t num_drivers_;

  I2CDMABus *dma_bus_ = nullptr;

  // Legacy standalone DMA (unused when dma_bus_ is set)
  DMAChannel dma_;
  uint16_t buf_[MAX_BUF_ENTRIES] __attribute__((aligned(32)));

  uint16_t buildForDriver(PCA9685Driver *drv, uint16_t *out);
};
