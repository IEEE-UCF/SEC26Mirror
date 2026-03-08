/**
 * @file PCA9685Manager.h
 * @brief DMA-accelerated PCA9685 multi-board manager for Teensy 4.1 (i.MX RT1062).
 *
 * Manages up to @ref MAX_DRIVERS PCA9685 boards on a single I2C bus.
 * Only dirty channels are written each cycle, using run-length-compressed
 * MTDR command sequences for minimal bus time.
 *
 * Two operating modes:
 *
 * **Shared bus mode** (preferred):
 *   Call setDMABus() to link to an I2CDMABus, then call buildInto() each
 *   cycle to append selective PCA9685 writes into the shared TX buffer.
 *   The I2CDMABus owns the DMA channels and bus lock.
 *
 * **Legacy standalone mode**:
 *   If no DMA bus is set, update() uses an internal DMA channel to fire
 *   transfers directly.  Retained for backwards compatibility.
 *
 * @note Thread safety: PCA9685Driver::bufferPWM() / bufferDigital() may be
 *       called from any thread.  buildInto() / update() should be called
 *       from a single dedicated flush task.
 *
 * @see I2CDMABus, Robot::PCA9685Driver
 */

#pragma once

#include <BaseDriver.h>
#include <DMAChannel.h>
#include <Wire.h>

#include "I2CDMABus.h"
#include "PCA9685Driver.h"

/**
 * @class PCA9685DMAManager
 * @brief Batches dirty-channel writes from multiple PCA9685 boards into
 *        DMA transfers on a shared I2C bus.
 */
class PCA9685DMAManager
{
public:
  /** @brief Maximum number of PCA9685 boards managed by one instance. */
  static constexpr uint8_t MAX_DRIVERS = 4;

  /**
   * @brief Worst-case MTDR words per driver per cycle.
   *
   * 1 START + 1 REG + (16 channels * 4 bytes) + 1 STOP = 67.
   */
  static constexpr uint16_t MAX_BUF_PER_DRIVER = 67;

  /** @brief Total standalone buffer capacity (words). */
  static constexpr uint16_t MAX_BUF_ENTRIES = MAX_BUF_PER_DRIVER * MAX_DRIVERS;

  /**
   * @brief Construct a PCA9685DMAManager.
   * @param wire      I2C bus the PCA9685 boards are on (Wire, Wire1, or Wire2).
   * @param clock_hz  I2C clock speed in Hz (default 1 MHz for fast mode+).
   */
  explicit PCA9685DMAManager(TwoWire &wire = Wire2,
                             uint32_t clock_hz = 1000000);

  /**
   * @brief Register a PCA9685 driver with this manager.
   * @param drv  Pointer to an initialised PCA9685Driver.
   * @return true on success, false if MAX_DRIVERS already registered.
   */
  bool addDriver(Robot::PCA9685Driver *drv);

  /** @brief Number of currently registered drivers. */
  uint8_t count() const { return num_drivers_; }

  /**
   * @brief Access a registered driver by index.
   * @param i  Driver index (0-based).
   * @return Pointer to the driver, or nullptr if index out of range.
   */
  Robot::PCA9685Driver *get(uint8_t i) { return i < num_drivers_ ? drivers_[i] : nullptr; }

  // ── Shared DMA bus mode ────────────────────────────────────────────

  /**
   * @brief Link this manager to a shared I2CDMABus.
   *
   * When set, buildInto() appends commands to the bus's TX buffer instead
   * of using the internal standalone DMA channel.
   *
   * @param bus  Pointer to the I2CDMABus (nullptr reverts to standalone).
   */
  void setDMABus(I2CDMABus *bus) { dma_bus_ = bus; }

  /**
   * @brief Append PCA9685 selective writes into the shared I2CDMABus TX buffer.
   *
   * Call sequence each cycle: dispatch() -> buildInto() -> fire().
   * Reserves worst-case space, builds only dirty channels, then rewinds
   * unused buffer space.
   *
   * @return Number of MTDR words written (0 if nothing dirty).
   */
  uint16_t buildInto();

  // ── Legacy standalone mode ─────────────────────────────────────────

  /**
   * @brief Build selective buffer and fire DMA (standalone mode, non-blocking).
   * @return true if a transfer was started, false if previous transfer
   *         still in progress or nothing dirty.
   */
  bool update();

  /**
   * @brief Check if the previous standalone DMA transfer has completed.
   * @return true when the DMA channel reports completion.
   */
  bool isComplete();

private:
  TwoWire &wire_;              ///< I2C bus reference.
  IMXRT_LPI2C_t *port_;        ///< LPI2C peripheral register block.
  uint8_t dma_src_;            ///< DMAMUX source for the I2C peripheral.
  uint32_t clock_hz_;          ///< I2C clock speed in Hz.

  Robot::PCA9685Driver *drivers_[MAX_DRIVERS];  ///< Registered driver pointers.
  uint8_t num_drivers_;                         ///< Number of registered drivers.

  I2CDMABus *dma_bus_ = nullptr;  ///< Shared DMA bus (nullptr = standalone mode).

  // Legacy standalone DMA (unused when dma_bus_ is set)
  DMAChannel dma_;  ///< Standalone DMA channel.
  uint16_t buf_[MAX_BUF_ENTRIES] __attribute__((aligned(32)));  ///< Standalone TX buffer.

  uint16_t buildForDriver(Robot::PCA9685Driver *drv, uint16_t *out);
};
