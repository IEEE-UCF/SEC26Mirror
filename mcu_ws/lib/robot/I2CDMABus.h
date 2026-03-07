/**
 * @file I2CDMABus.h
 * @brief Two-phase DMA engine for I2C buses on Teensy 4.1 (i.MX RT1062).
 *
 * Batches writes AND reads into a single DMA transfer cycle:
 *   1. TX phase: DMA pushes MTDR command words (START, addr, reg, data, STOP,
 *      RECEIVE commands for reads).  MDER = TDDE only.
 *   2. TX-complete ISR: disables TDDE, sets up RX DMA, enables RDDE.
 *   3. RX phase: DMA drains received bytes from MRDR.  MDER = RDDE only.
 *   4. RX-complete ISR: marks transfer done.
 *
 * One instance per Wire bus.  Thread-safe via I2CBusLock — fire() acquires
 * the bus mutex, dispatch() releases it.
 *
 * MTDR command encoding (LPI2C on RT1062):
 *   0x0000 | data     — Transmit byte
 *   0x0100 | (N-1)    — Receive N bytes
 *   0x0200           — STOP
 *   0x0400 | addr    — START + transmit address byte
 */

#pragma once

#include <DMAChannel.h>
#include <Wire.h>

#include "I2CBusLock.h"

class I2CDMABus {
 public:
  // ── Buffer sizing ──────────────────────────────────────────────────
  static constexpr uint16_t TX_BUF_SIZE = 300;  // MTDR command words
  static constexpr uint8_t RX_BUF_SIZE = 64;    // received bytes
  static constexpr uint8_t MAX_RX_SLOTS = 16;   // max pending reads per cycle

  // ── MTDR command bits (RT1062 LPI2C) ───────────────────────────────
  static constexpr uint16_t CMD_TX = 0x0000;
  static constexpr uint16_t CMD_RX = 0x0100;
  static constexpr uint16_t CMD_STOP = 0x0200;
  static constexpr uint16_t CMD_START = 0x0400;

  explicit I2CDMABus(TwoWire &wire, uint32_t clock_hz = 1000000);

  // ── Queue API (call between dispatch() and fire()) ─────────────────

  /// Queue a register write: START, addr+W, reg, data[0..len-1], STOP.
  void queueWrite(uint8_t addr, uint8_t reg, const uint8_t *data,
                  uint8_t len);

  /// Queue a single-byte write: START, addr+W, byte, STOP (e.g. mux select).
  void queueWriteByte(uint8_t addr, uint8_t byte);

  /// Queue a register read: START, addr+W, reg, RSTART, addr+R, RECEIVE(N), STOP.
  /// Received bytes will be copied to dest on dispatch().
  void queueRead(uint8_t addr, uint8_t reg, uint8_t *dest, uint8_t len);

  /// Reserve raw MTDR buffer space for custom command sequences (e.g. PCA9685).
  /// Returns pointer to first reserved slot. Caller writes MTDR commands directly.
  uint16_t *reserveTx(uint16_t count);

  /// Reclaim unused tail of a previous reserveTx() allocation.
  /// @param pos  absolute TX buffer position to rewind to.
  void rewindTx(uint16_t pos);

  /// Current TX write position (for rewindTx bookkeeping).
  uint16_t txPos() const { return tx_pos_; }

  // ── Transfer lifecycle ─────────────────────────────────────────────

  /// Flush caches, configure DMA, acquire bus lock, start TX phase.
  /// Returns false if previous transfer is still in progress.
  bool fire();

  /// True when both TX and RX phases are complete.
  bool isComplete() const { return transfer_done_; }

  /// Copy RX bytes to destinations, release bus lock, reset queue for next cycle.
  void dispatch();

  /// Emergency cleanup: disable DMA, release lock, reset state.
  void reset();

 private:
  // ── RX dispatch slot ──────────────────────────────────────────────
  struct RxSlot {
    uint8_t *dest;
    uint8_t len;
  };

  // ── Hardware ──────────────────────────────────────────────────────
  TwoWire &wire_;
  IMXRT_LPI2C_t *port_;
  uint8_t dma_mux_src_;
  uint32_t clock_hz_;

  // ── DMA channels ──────────────────────────────────────────────────
  DMAChannel tx_dma_;
  DMAChannel rx_dma_;

  // ── Buffers ───────────────────────────────────────────────────────
  uint16_t tx_buf_[TX_BUF_SIZE] __attribute__((aligned(32)));
  uint8_t rx_buf_[RX_BUF_SIZE] __attribute__((aligned(32)));
  uint16_t tx_pos_ = 0;
  uint8_t rx_expect_ = 0;  // total RX bytes expected this cycle

  // ── RX dispatch table ─────────────────────────────────────────────
  RxSlot rx_slots_[MAX_RX_SLOTS];
  uint8_t rx_slot_count_ = 0;

  // ── State ─────────────────────────────────────────────────────────
  volatile bool transfer_done_ = true;
  bool bus_locked_ = false;

  // ── ISR routing ───────────────────────────────────────────────────
  static constexpr uint8_t MAX_INSTANCES = 3;
  static I2CDMABus *instances_[MAX_INSTANCES];
  uint8_t instance_idx_ = 0;

  void onTxComplete();
  void onRxComplete();

  static void txISR0() { if (instances_[0]) instances_[0]->onTxComplete(); }
  static void txISR1() { if (instances_[1]) instances_[1]->onTxComplete(); }
  static void txISR2() { if (instances_[2]) instances_[2]->onTxComplete(); }

  static void rxISR0() { if (instances_[0]) instances_[0]->onRxComplete(); }
  static void rxISR1() { if (instances_[1]) instances_[1]->onRxComplete(); }
  static void rxISR2() { if (instances_[2]) instances_[2]->onRxComplete(); }
};
