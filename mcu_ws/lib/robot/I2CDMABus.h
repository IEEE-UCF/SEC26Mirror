/**
 * @file I2CDMABus.h
 * @brief Two-phase DMA engine for I2C buses on Teensy 4.1 (i.MX RT1062).
 *
 * Batches writes AND reads into a single DMA transfer cycle:
 *   1. **fire()**: sets up both TX and RX DMA channels, enables TDDE + RDDE.
 *   2. **TX + RX run simultaneously**: TX pushes MTDR commands, RX drains MRDR.
 *   3. **TX-complete ISR**: RX already finished, marks transfer done.
 *
 * One instance per Wire bus.  Thread-safe via I2CBusLock — fire() acquires
 * the bus mutex, dispatch() releases it.
 *
 * Typical 1 kHz cycle (used by PCA9685Manager for motor PWM):
 * @code
 *   if (bus.isComplete()) {
 *     bus.dispatch();          // release lock, deliver RX, reset queues
 *     pca_mgr.buildInto();    // append dirty PCA9685 writes
 *     bus.fire();              // start DMA transfer
 *   }
 * @endcode
 *
 * MTDR command encoding (LPI2C on RT1062):
 *   - `0x0000 | data`  — Transmit byte
 *   - `0x0100 | (N-1)` — Receive N bytes
 *   - `0x0200`         — STOP
 *   - `0x0400 | addr`  — START + transmit address byte
 *
 * @see PCA9685Manager
 */

#pragma once

#include <DMAChannel.h>
#include <Wire.h>

#include "I2CBusLock.h"

#if defined(USE_FREERTOS)
#include "FreeRTOSCompat.h"
#endif

/**
 * @class I2CDMABus
 * @brief Shared two-phase (TX+RX) DMA engine for one I2C bus on Teensy 4.1.
 *
 * Multiple producers (e.g. PCA9685Manager, sensor drivers) queue
 * commands between dispatch() and fire().  All commands are sent in a
 * single DMA burst, minimising bus arbitration overhead.
 */
class I2CDMABus {
 public:
  // ── Buffer sizing ──────────────────────────────────────────────────

  /** @brief TX buffer capacity in MTDR command words (16-bit each). */
  static constexpr uint16_t TX_BUF_SIZE = 300;

  /** @brief RX buffer capacity in bytes. */
  static constexpr uint8_t RX_BUF_SIZE = 64;

  /** @brief Maximum pending read operations per cycle. */
  static constexpr uint8_t MAX_RX_SLOTS = 16;

  // ── MTDR command bits (RT1062 LPI2C) ───────────────────────────────

  static constexpr uint16_t CMD_TX = 0x0000;    ///< Transmit data byte.
  static constexpr uint16_t CMD_RX = 0x0100;    ///< Receive N bytes (OR with N-1).
  static constexpr uint16_t CMD_STOP = 0x0200;  ///< Generate STOP condition.
  static constexpr uint16_t CMD_START = 0x0400; ///< Generate START + address byte.

  /**
   * @brief Construct an I2CDMABus for the given Wire bus.
   * @param wire      TwoWire instance (Wire, Wire1, or Wire2).
   * @param clock_hz  I2C clock speed in Hz (default 1 MHz).
   */
  explicit I2CDMABus(TwoWire &wire, uint32_t clock_hz = 1000000);

  // ── Queue API (call between dispatch() and fire()) ─────────────────

  /**
   * @brief Queue a register write.
   *
   * Generates: START, addr+W, reg, data[0..len-1], STOP.
   *
   * @param addr  7-bit I2C device address.
   * @param reg   Register address byte.
   * @param data  Pointer to data bytes to write.
   * @param len   Number of data bytes.
   */
  void queueWrite(uint8_t addr, uint8_t reg, const uint8_t *data,
                  uint8_t len);

  /**
   * @brief Queue a single-byte write (e.g. mux channel select).
   *
   * Generates: START, addr+W, byte, STOP.
   *
   * @param addr  7-bit I2C device address.
   * @param byte  The single data byte to write.
   */
  void queueWriteByte(uint8_t addr, uint8_t byte);

  /**
   * @brief Queue a register read.
   *
   * Generates: START, addr+W, reg, RSTART, addr+R, RECEIVE(len), STOP.
   * Received bytes are copied to @p dest during dispatch().
   *
   * @param addr  7-bit I2C device address.
   * @param reg   Register address byte.
   * @param dest  Destination buffer (must remain valid until dispatch()).
   * @param len   Number of bytes to read.
   */
  void queueRead(uint8_t addr, uint8_t reg, uint8_t *dest, uint8_t len);

  /**
   * @brief Reserve raw MTDR buffer space for custom command sequences.
   *
   * Used by PCA9685Manager to write pre-built MTDR commands directly
   * into the TX buffer.  Caller must rewindTx() any unused space.
   *
   * @param count  Number of 16-bit MTDR words to reserve.
   * @return Pointer to the first reserved slot, or nullptr if insufficient space.
   */
  uint16_t *reserveTx(uint16_t count);

  /**
   * @brief Reclaim unused tail of a previous reserveTx() allocation.
   * @param pos  Absolute TX buffer position to rewind to.
   */
  void rewindTx(uint16_t pos);

  /**
   * @brief Current TX write position (for rewindTx bookkeeping).
   * @return Current write offset into the TX buffer.
   */
  uint16_t txPos() const { return tx_pos_; }

  // ── Transfer lifecycle ─────────────────────────────────────────────

  /**
   * @brief Flush caches, configure DMA, acquire bus lock, start TX phase.
   *
   * Non-blocking.  Returns immediately after starting DMA.  Poll
   * isComplete() or wait for the next cycle to check completion.
   *
   * @return false if the TX buffer is empty or a previous transfer is
   *         still in progress; true if the transfer was started.
   */
  bool fire();

  /**
   * @brief Check if both TX and RX DMA phases are complete.
   * @return true when the transfer is finished and dispatch() can be called.
   */
  bool isComplete() const { return transfer_done_; }

#if defined(USE_FREERTOS)
  /**
   * @brief Block until the transfer completes or timeout expires.
   *
   * Uses FreeRTOS task notification (set by DMA ISR) for zero-latency
   * wake-up instead of polling with vTaskDelay.
   *
   * @param timeout_ms  Maximum wait time in milliseconds.
   * @return true if the transfer completed, false on timeout.
   */
  bool waitComplete(uint32_t timeout_ms);
#endif

  /**
   * @brief Complete the current cycle: deliver RX data, release bus, reset.
   *
   * Copies received bytes to their destination buffers (from queueRead),
   * releases the I2C bus mutex, and resets the queue for the next cycle.
   * Safe to call even when no transfer was in progress.
   */
  void dispatch();

  /**
   * @brief Emergency cleanup: disable DMA, release lock, reset state.
   *
   * Call this if a transfer appears stuck or on error recovery paths.
   */
  void reset();

 private:
  /// @brief Bookkeeping for a pending read: destination buffer + length.
  struct RxSlot {
    uint8_t *dest;  ///< Destination for received bytes.
    uint8_t len;    ///< Number of bytes to copy.
  };

  // ── Hardware ──────────────────────────────────────────────────────
  TwoWire &wire_;              ///< I2C bus reference.
  IMXRT_LPI2C_t *port_;        ///< LPI2C peripheral register block.
  uint8_t dma_mux_src_;        ///< DMAMUX source for the I2C peripheral.
  uint32_t clock_hz_;          ///< I2C clock speed in Hz.

  // ── DMA channels ──────────────────────────────────────────────────
  DMAChannel tx_dma_;  ///< TX DMA channel (pushes MTDR commands).
  DMAChannel rx_dma_;  ///< RX DMA channel (drains MRDR bytes).

  // ── Buffers ───────────────────────────────────────────────────────
  uint16_t tx_buf_[TX_BUF_SIZE] __attribute__((aligned(32)));  ///< TX command buffer.
  uint8_t rx_buf_[RX_BUF_SIZE] __attribute__((aligned(32)));   ///< RX data buffer.
  uint16_t tx_pos_ = 0;       ///< Current write position in tx_buf_.
  uint8_t rx_expect_ = 0;     ///< Total RX bytes expected this cycle.

  // ── RX dispatch table ─────────────────────────────────────────────
  RxSlot rx_slots_[MAX_RX_SLOTS];  ///< Pending read destinations.
  uint8_t rx_slot_count_ = 0;     ///< Number of pending reads.

  // ── State ─────────────────────────────────────────────────────────
  volatile bool transfer_done_ = true;  ///< Set by ISR when transfer completes.
  bool bus_locked_ = false;             ///< True while holding the bus mutex.

#if defined(USE_FREERTOS)
  TaskHandle_t waiting_task_ = nullptr;  ///< Task to notify on DMA completion.
#endif

  /// Notify waiting task from ISR context (called by onTxComplete).
  void notifyTaskFromISR();

  // ── ISR routing ───────────────────────────────────────────────────
  static constexpr uint8_t MAX_INSTANCES = 3;  ///< One per Wire bus.
  static I2CDMABus *instances_[MAX_INSTANCES];  ///< Static instance table.
  uint8_t instance_idx_ = 0;                   ///< This instance's table index.

  /** @brief TX-complete ISR handler: marks transfer done (RX already finished). */
  void onTxComplete();

  // Static ISR trampolines (one per possible instance).
  static void txISR0() { if (instances_[0]) instances_[0]->onTxComplete(); }
  static void txISR1() { if (instances_[1]) instances_[1]->onTxComplete(); }
  static void txISR2() { if (instances_[2]) instances_[2]->onTxComplete(); }
};
