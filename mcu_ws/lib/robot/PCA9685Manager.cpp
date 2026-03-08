#include "PCA9685Manager.h"

#include "I2CDMABus.h"

using namespace Robot;

PCA9685Manager::PCA9685Manager(const PCA9685ManagerSetup& setup)
    : Classes::BaseDriver(setup) {}

PCA9685Manager::~PCA9685Manager() {
  for (auto d : drivers_) delete d;
  drivers_.clear();
}

bool PCA9685Manager::init() {
  bool ok = true;
  for (auto d : drivers_) {
    ok = d->init() && ok;
  }
  initSuccess_ = ok;
  return ok;
}

void PCA9685Manager::update() {
  for (auto d : drivers_) {
    d->applyBuffered();
  }
}

PCA9685Driver* PCA9685Manager::createDriver(const PCA9685DriverSetup& setup) {
  PCA9685Driver* d = new PCA9685Driver(setup);
  drivers_.push_back(d);
  return d;
}

// ── Shared DMA bus support ──────────────────────────────────────────────────

uint16_t PCA9685Manager::buildForDriver(PCA9685Driver* drv, uint16_t* out) {
  uint32_t dirty = 0;
  uint8_t* d = (uint8_t*)drv->buffer_dirty_;
  for (uint8_t i = 0; i < 16; i++) {
    dirty |= (uint32_t)d[i] << i;
    d[i] = 0;
  }

  if (dirty == 0) return 0;

  uint16_t* p = out;

  while (dirty) {
    uint32_t tz = __builtin_ctz(dirty);
    uint32_t run = __builtin_ctz(~(dirty >> tz));

    *p++ = LPI2C_MTDR_CMD_START | drv->setup_.i2c_addr_ << 1;
    *p++ = PCA9685_LED0_ON_L + (tz << 2);

    const uint16_t* src = &drv->buffer_[tz];
    uint32_t cnt = run;
    do {
      uint32_t dd = *src++;

      if (__builtin_expect(dd == 0, 0)) {
        *p++ = 0;
        *p++ = 0;
        *p++ = 0;
        *p++ = 0x0010u;
      } else if (__builtin_expect(dd >= 4095, 0)) {
        *p++ = 0;
        *p++ = 0x0010u;
        *p++ = 0;
        *p++ = 0;
      } else {
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

uint16_t PCA9685Manager::buildInto() {
  if (!dma_bus_) return 0;

  uint16_t needed = MAX_BUF_PER_DRIVER * drivers_.size();
  uint16_t save_pos = dma_bus_->txPos();

  uint16_t* slot = dma_bus_->reserveTx(needed);
  if (!slot) return 0;

  uint16_t total = 0;
  for (size_t i = 0; i < drivers_.size(); i++)
    total += buildForDriver(drivers_[i], slot + total);

  if (total == 0) {
    dma_bus_->rewindTx(save_pos);
    return 0;
  }

  dma_bus_->rewindTx(save_pos + total);
  return total;
}
