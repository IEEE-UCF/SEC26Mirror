/**
 * @file PCA9685Manager.cpp
 * @brief Implementation of PCA9685DMAManager — DMA-accelerated multi-board
 *        PCA9685 write batching for Teensy 4.1.
 * @see PCA9685Manager.h
 */

#include "PCA9685Manager.h"

// ── Constructor ─────────────────────────────────────────────────────────────

PCA9685DMAManager::PCA9685DMAManager(TwoWire &wire, uint32_t clock_hz)
    : wire_(wire),
      port_(nullptr),
      dma_src_(0),
      clock_hz_(clock_hz),
      num_drivers_(0)
{
  // Map TwoWire instance to LPI2C peripheral and DMAMUX source.
  // Teensy 4.1 mapping: Wire=LPI2C1, Wire1=LPI2C3, Wire2=LPI2C4.
  if (&wire == &Wire)
  {
    port_ = &IMXRT_LPI2C1;
    dma_src_ = DMAMUX_SOURCE_LPI2C1;
  }
  else if (&wire == &Wire1)
  {
    port_ = &IMXRT_LPI2C3;
    dma_src_ = DMAMUX_SOURCE_LPI2C3;
  }
  else if (&wire == &Wire2)
  {
    port_ = &IMXRT_LPI2C4;
    dma_src_ = DMAMUX_SOURCE_LPI2C4;
  }

  // Mark standalone DMA as done so first update() succeeds.
  dma_.TCD->CSR |= DMA_TCD_CSR_DONE;
  memset(drivers_, 0, sizeof(drivers_));
}

// ── Driver registration ─────────────────────────────────────────────────────

bool PCA9685DMAManager::addDriver(Robot::PCA9685Driver *drv)
{
  if (num_drivers_ >= MAX_DRIVERS)
    return false;
  drivers_[num_drivers_++] = drv;
  return true;
}

// ── Per-driver MTDR command builder ─────────────────────────────────────────

uint16_t PCA9685DMAManager::buildForDriver(Robot::PCA9685Driver *drv, uint16_t *out)
{
  // Gather dirty bitmask from bool[16] and clear flags atomically.
  uint32_t dirty = 0;
  uint8_t *d = (uint8_t *)drv->buffer_dirty_;
  for (uint8_t i = 0; i < 16; i++)
  {
    dirty |= (uint32_t)d[i] << i;
    d[i] = 0;
  }

  if (dirty == 0)
    return 0;

  uint16_t *p = out;

  // Process consecutive runs of dirty channels for optimal bus utilisation.
  while (dirty)
  {
    uint32_t tz  = __builtin_ctz(dirty);          // first dirty channel
    uint32_t run = __builtin_ctz(~(dirty >> tz)); // consecutive run length

    // START + address byte (write mode)
    *p++ = LPI2C_MTDR_CMD_START | drv->setup_.i2c_addr_ << 1;
    // Register address: LED0_ON_L + 4 * channel_index
    *p++ = PCA9685_LED0_ON_L + (tz << 2);

    const uint16_t *src = &drv->buffer_[tz];
    uint32_t cnt = run;
    do
    {
      uint32_t dd = *src++;

      // Each channel: ON_L, ON_H, OFF_L, OFF_H (4 bytes).
      // Special cases for fully-off (duty==0) and fully-on (duty>=4095).
      if (__builtin_expect(dd == 0, 0))
      {
        *p++ = 0;
        *p++ = 0;
        *p++ = 0;
        *p++ = 0x0010u; // OFF_H bit4 = full-off
      }
      else if (__builtin_expect(dd >= 4095, 0))
      {
        *p++ = 0;
        *p++ = 0x0010u; // ON_H bit4 = full-on
        *p++ = 0;
        *p++ = 0;
      }
      else
      {
        *p++ = 0;           // ON_L  = 0
        *p++ = 0;           // ON_H  = 0
        *p++ = dd & 0xFFu;  // OFF_L
        *p++ = dd >> 8;     // OFF_H
      }
    } while (--cnt);

    // Clear the processed run from the bitmask.
    dirty ^= ((1u << run) - 1u) << tz;
  }

  *p++ = LPI2C_MTDR_CMD_STOP;

  return (uint16_t)(p - out);
}

// ── Shared DMA bus mode ─────────────────────────────────────────────────────

uint16_t PCA9685DMAManager::buildInto()
{
  if (!dma_bus_) return 0;

  // Reserve worst-case space; we'll rewind any unused tail after building.
  uint16_t needed = MAX_BUF_PER_DRIVER * num_drivers_;
  uint16_t save_pos = dma_bus_->txPos();

  uint16_t *slot = dma_bus_->reserveTx(needed);
  if (!slot) return 0;

  uint16_t total = 0;
  for (uint8_t d = 0; d < num_drivers_; d++)
    total += buildForDriver(drivers_[d], slot + total);

  if (total == 0)
  {
    // Nothing dirty — give back the entire reservation.
    dma_bus_->rewindTx(save_pos);
    return 0;
  }

  // Rewind unused tail of the reservation.
  dma_bus_->rewindTx(save_pos + total);
  return total;
}

// ── Legacy standalone mode ──────────────────────────────────────────────────

bool PCA9685DMAManager::update()
{
  if (!isComplete())
    return false;

  uint16_t pos = 0;
  for (uint8_t d = 0; d < num_drivers_; d++)
    pos += buildForDriver(drivers_[d], buf_ + pos);

  if (pos == 0)
    return false;

  // Configure DMA: source from buf_, destination to LPI2C MTDR register.
  dma_.clearComplete();
  dma_.sourceBuffer(buf_, pos);
  dma_.destination(port_->MTDR);
  dma_.transferSize(2);
  dma_.transferCount(pos);
  dma_.disableOnCompletion();
  dma_.triggerAtHardwareEvent(dma_src_);

  // Flush cache if buffer is in non-coherent memory (OCRAM2).
  if ((uint32_t)buf_ >= 0x20200000u)
    arm_dcache_flush((void *)buf_, pos * sizeof(uint16_t));

  // Enable TX DMA requests and start transfer.
  port_->MDER = LPI2C_MDER_TDDE;
  wire_.setClock(clock_hz_);
  dma_.enable();

  return true;
}

bool PCA9685DMAManager::isComplete()
{
  return dma_.complete();
}
