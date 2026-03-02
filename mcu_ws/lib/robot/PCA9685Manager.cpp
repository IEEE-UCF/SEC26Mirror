#include "PCA9685Manager.h"

PCA9685DMAManager::PCA9685DMAManager(TwoWire &wire, uint32_t clock_hz)
    : wire_(wire),
      port_(nullptr),
      dma_src_(0),
      clock_hz_(clock_hz),
      num_drivers_(0)
{
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

  dma_.TCD->CSR |= DMA_TCD_CSR_DONE;
  memset(drivers_, 0, sizeof(drivers_));
}

bool PCA9685DMAManager::addDriver(PCA9685Driver *drv)
{
  if (num_drivers_ >= MAX_DRIVERS)
    return false;
  drivers_[num_drivers_++] = drv;
  return true;
}

uint16_t PCA9685DMAManager::buildForDriver(PCA9685Driver *drv, uint16_t *out)
{
  // gather bitmask from bool[16] and zero them in one pass
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

  while (dirty)
  {
    uint32_t tz  = __builtin_ctz(dirty);          // first dirty channel
    uint32_t run = __builtin_ctz(~(dirty >> tz)); // consecutive run length

    *p++ = LPI2C_MTDR_CMD_START | drv->setup_.i2c_addr_ << 1;
    *p++ = PCA9685_LED0_ON_L + (tz << 2);

    const uint16_t *src = &drv->buffer_[tz];
    uint32_t cnt = run;
    do
    {
      uint32_t dd = *src++;

      if (__builtin_expect(dd == 0, 0))
      {
        *p++ = 0;
        *p++ = 0;
        *p++ = 0;
        *p++ = 0x0010u; // OFF_H bit4
      }
      else if (__builtin_expect(dd >= 4095, 0))
      {
        *p++ = 0;
        *p++ = 0x0010u; // ON_H bit4
        *p++ = 0;
        *p++ = 0;
      }
      else
      {
        *p++ = 0;
        *p++ = 0;
        *p++ = dd & 0xFFu;
        *p++ = dd >> 8;
      }
    } while (--cnt);

    dirty ^= ((1u << run) - 1u) << tz;
  }

  *p++ = LPI2C_MTDR_CMD_STOP;

  return (uint16_t)(p - out);
}

// ── Shared DMA bus mode ────────────────────────────────────────────────────

uint16_t PCA9685DMAManager::buildInto()
{
  if (!dma_bus_) return 0;

  // Calculate worst-case space needed
  uint16_t needed = MAX_BUF_PER_DRIVER * num_drivers_;
  uint16_t save_pos = dma_bus_->txPos();

  uint16_t *slot = dma_bus_->reserveTx(needed);
  if (!slot) return 0;

  uint16_t total = 0;
  for (uint8_t d = 0; d < num_drivers_; d++)
    total += buildForDriver(drivers_[d], slot + total);

  if (total == 0)
  {
    // Nothing dirty — give back the reserved space.
    dma_bus_->rewindTx(save_pos);
    return 0;
  }

  // Rewind unused tail of the reservation.
  dma_bus_->rewindTx(save_pos + total);
  return total;
}

// ── Legacy standalone mode ─────────────────────────────────────────────────

bool PCA9685DMAManager::update()
{
  if (!isComplete())
    return false;

  uint16_t pos = 0;
  for (uint8_t d = 0; d < num_drivers_; d++)
    pos += buildForDriver(drivers_[d], buf_ + pos);

  if (pos == 0)
    return false;

  dma_.clearComplete();
  dma_.sourceBuffer(buf_, pos);
  dma_.destination(port_->MTDR);
  dma_.transferSize(2);
  dma_.transferCount(pos);
  dma_.disableOnCompletion();
  dma_.triggerAtHardwareEvent(dma_src_);

  if ((uint32_t)buf_ >= 0x20200000u) // OCRAM2, not cache-coherent like DTCM
    arm_dcache_flush((void *)buf_, pos * sizeof(uint16_t));

  port_->MDER = LPI2C_MDER_TDDE;
  wire_.setClock(clock_hz_);
  dma_.enable();

  return true;
}

bool PCA9685DMAManager::isComplete()
{
  return dma_.complete();
}
