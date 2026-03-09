/**
 * @file I2CDMABus.cpp
 * @brief Implementation of I2CDMABus — two-phase DMA engine for shared
 *        I2C buses on Teensy 4.1 (i.MX RT1062).
 * @see I2CDMABus.h
 */

#include "I2CDMABus.h"

// ── Static instance table for ISR routing ──────────────────────────────────
I2CDMABus *I2CDMABus::instances_[MAX_INSTANCES] = {nullptr, nullptr, nullptr};

// ── Constructor ────────────────────────────────────────────────────────────

I2CDMABus::I2CDMABus(TwoWire &wire, uint32_t clock_hz)
    : wire_(wire),
      port_(nullptr),
      dma_mux_src_(0),
      clock_hz_(clock_hz)
{
  // Map Wire instance to LPI2C peripheral and DMA mux source.
  // Teensy 4.1: Wire=LPI2C1, Wire1=LPI2C3, Wire2=LPI2C4.
  if (&wire == &Wire) {
    port_ = &IMXRT_LPI2C1;
    dma_mux_src_ = DMAMUX_SOURCE_LPI2C1;
    instance_idx_ = 0;
  } else if (&wire == &Wire1) {
    port_ = &IMXRT_LPI2C3;
    dma_mux_src_ = DMAMUX_SOURCE_LPI2C3;
    instance_idx_ = 1;
  } else if (&wire == &Wire2) {
    port_ = &IMXRT_LPI2C4;
    dma_mux_src_ = DMAMUX_SOURCE_LPI2C4;
    instance_idx_ = 2;
  }

  instances_[instance_idx_] = this;

  // Mark TX DMA as done so first fire() succeeds.
  tx_dma_.TCD->CSR |= DMA_TCD_CSR_DONE;
  rx_dma_.TCD->CSR |= DMA_TCD_CSR_DONE;

  // Configure bus clock speed once (all users on this bus share the same rate).
  wire_.begin();
  wire_.setClock(clock_hz_);
}

// ── Queue: register write ──────────────────────────────────────────────────

void I2CDMABus::queueWrite(uint8_t addr, uint8_t reg, const uint8_t *data,
                           uint8_t len)
{
  if (tx_pos_ + 3 + len + 1 > TX_BUF_SIZE) return;

  tx_buf_[tx_pos_++] = CMD_START | (addr << 1);       // START + addr+W
  tx_buf_[tx_pos_++] = CMD_TX | reg;                   // register byte
  for (uint8_t i = 0; i < len; i++)
    tx_buf_[tx_pos_++] = CMD_TX | data[i];             // data bytes
  tx_buf_[tx_pos_++] = CMD_STOP;                       // STOP
}

// ── Queue: single-byte write (mux select, etc.) ───────────────────────────

void I2CDMABus::queueWriteByte(uint8_t addr, uint8_t byte)
{
  if (tx_pos_ + 3 > TX_BUF_SIZE) return;

  tx_buf_[tx_pos_++] = CMD_START | (addr << 1);       // START + addr+W
  tx_buf_[tx_pos_++] = CMD_TX | byte;                  // data byte
  tx_buf_[tx_pos_++] = CMD_STOP;                       // STOP
}

// ── Queue: register read ───────────────────────────────────────────────────

void I2CDMABus::queueRead(uint8_t addr, uint8_t reg, uint8_t *dest,
                          uint8_t len)
{
  if (tx_pos_ + 5 > TX_BUF_SIZE) return;
  if (rx_slot_count_ >= MAX_RX_SLOTS) return;
  if (rx_expect_ + len > RX_BUF_SIZE) return;

  // Write phase: START, addr+W, register
  tx_buf_[tx_pos_++] = CMD_START | (addr << 1);       // START + addr+W
  tx_buf_[tx_pos_++] = CMD_TX | reg;                   // register byte

  // Read phase: repeated START, addr+R, receive N bytes, STOP
  tx_buf_[tx_pos_++] = CMD_START | (addr << 1) | 1;   // RSTART + addr+R
  tx_buf_[tx_pos_++] = CMD_RX | (len - 1);            // RECEIVE N bytes
  tx_buf_[tx_pos_++] = CMD_STOP;                       // STOP

  // Record dispatch slot
  rx_slots_[rx_slot_count_++] = {dest, len};
  rx_expect_ += len;
}

// ── Raw buffer access (for PCA9685Manager) ─────────────────────────────────

uint16_t *I2CDMABus::reserveTx(uint16_t count)
{
  if (tx_pos_ + count > TX_BUF_SIZE) return nullptr;
  uint16_t *ptr = &tx_buf_[tx_pos_];
  tx_pos_ += count;
  return ptr;
}

void I2CDMABus::rewindTx(uint16_t pos)
{
  if (pos <= tx_pos_)
    tx_pos_ = pos;
}

// ── Fire: start the two-phase DMA transfer ─────────────────────────────────

bool I2CDMABus::fire()
{
  if (!transfer_done_) return false;
  if (tx_pos_ == 0) return false;

  transfer_done_ = false;

  // Acquire bus lock — blocks other threads doing blocking I2C on this bus.
  I2CBus::mutexFor(wire_).lock();
  bus_locked_ = true;

  // Re-assert clock speed: driver inits (e.g. PCA9685) may have called
  // Wire.begin() which resets to 100 kHz defaults.
  wire_.setClock(clock_hz_);

#if defined(USE_FREERTOS)
  // Record caller so DMA ISR can notify on completion.
  waiting_task_ = xTaskGetCurrentTaskHandle();
#endif

  // Flush TX buffer from data cache if it lives in cached memory (OCRAM).
  if ((uint32_t)tx_buf_ >= 0x20200000u)
    arm_dcache_flush((void *)tx_buf_, tx_pos_ * sizeof(uint16_t));

  // ── RX DMA setup (must be ready BEFORE TX starts sending RECEIVE cmds) ──
  if (rx_expect_ > 0) {
    rx_dma_.clearComplete();
    rx_dma_.source(port_->MRDR);
    rx_dma_.destinationBuffer(rx_buf_, rx_expect_);
    rx_dma_.transferSize(1);               // 1-byte reads from LSB of MRDR
    rx_dma_.transferCount(rx_expect_);
    rx_dma_.disableOnCompletion();
    rx_dma_.triggerAtHardwareEvent(dma_mux_src_);
  }

  // ── TX DMA setup ───────────────────────────────────────────────────
  tx_dma_.clearComplete();
  tx_dma_.sourceBuffer(tx_buf_, tx_pos_ * sizeof(uint16_t));
  tx_dma_.destination(port_->MTDR);
  tx_dma_.transferSize(2);         // 16-bit MTDR writes
  tx_dma_.transferCount(tx_pos_);
  tx_dma_.disableOnCompletion();
  tx_dma_.triggerAtHardwareEvent(dma_mux_src_);

  // Attach TX-complete ISR.
  switch (instance_idx_) {
    case 0: tx_dma_.attachInterrupt(txISR0); break;
    case 1: tx_dma_.attachInterrupt(txISR1); break;
    case 2: tx_dma_.attachInterrupt(txISR2); break;
  }

  // Enable both TX and RX DMA requests simultaneously so the peripheral
  // can drain MRDR via DMA as soon as RECEIVE commands produce data.
  port_->MDER = LPI2C_MDER_TDDE | (rx_expect_ > 0 ? LPI2C_MDER_RDDE : 0);
  if (rx_expect_ > 0) rx_dma_.enable();
  tx_dma_.enable();

  return true;
}

// ── Notify waiting task from ISR ────────────────────────────────────────────

void I2CDMABus::notifyTaskFromISR()
{
#if defined(USE_FREERTOS)
  if (waiting_task_) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(waiting_task_, &xHigherPriorityTaskWoken);
    waiting_task_ = nullptr;
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
#endif
}

// ── TX complete ISR ────────────────────────────────────────────────────────

void I2CDMABus::onTxComplete()
{
  tx_dma_.clearInterrupt();
  tx_dma_.clearComplete();

  // RX DMA (if any) already completed during the TX phase.
  port_->MDER = 0;
  transfer_done_ = true;
  notifyTaskFromISR();
}


// ── Wait for completion (FreeRTOS) ──────────────────────────────────────────

#if defined(USE_FREERTOS)
bool I2CDMABus::waitComplete(uint32_t timeout_ms)
{
  if (transfer_done_) return true;

  // Block until ISR notifies us or timeout expires.
  TickType_t ticks = pdMS_TO_TICKS(timeout_ms);
  if (ticks == 0) ticks = 1;  // at least 1 tick
  ulTaskNotifyTake(pdTRUE, ticks);

  return transfer_done_;
}
#endif

// ── Dispatch: copy RX data to destinations, release lock, reset ────────────

void I2CDMABus::dispatch()
{
  // Invalidate RX buffer from data cache if in cached memory.
  if ((uint32_t)rx_buf_ >= 0x20200000u)
    arm_dcache_delete((void *)rx_buf_, RX_BUF_SIZE);

  // Copy received bytes to their destination buffers.
  uint8_t offset = 0;
  for (uint8_t i = 0; i < rx_slot_count_; i++) {
    memcpy(rx_slots_[i].dest, rx_buf_ + offset, rx_slots_[i].len);
    offset += rx_slots_[i].len;
  }

  // Release bus lock.
  if (bus_locked_) {
    I2CBus::mutexFor(wire_).unlock();
    bus_locked_ = false;
  }

  // Reset queue state for next cycle.
  tx_pos_ = 0;
  rx_expect_ = 0;
  rx_slot_count_ = 0;
}

// ── Reset: emergency cleanup ───────────────────────────────────────────────

void I2CDMABus::reset()
{
  // Kill DMA channels.
  tx_dma_.disable();
  rx_dma_.disable();
  tx_dma_.clearComplete();
  rx_dma_.clearComplete();

  // Disable DMA requests on peripheral.
  if (port_)
    port_->MDER = 0;

  // Release bus lock if held.
  if (bus_locked_) {
    I2CBus::mutexFor(wire_).unlock();
    bus_locked_ = false;
  }

  // Reset state.
  transfer_done_ = true;
  tx_pos_ = 0;
  rx_expect_ = 0;
  rx_slot_count_ = 0;
}
