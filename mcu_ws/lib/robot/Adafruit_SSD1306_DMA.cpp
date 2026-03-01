#include "Adafruit_SSD1306_DMA.h"

#include <algorithm>

std::list<Adafruit_SSD1306_DMA*> Adafruit_SSD1306_DMA::display_list_;

// ── Constructor ──────────────────────────────────────────────────────────────

Adafruit_SSD1306_DMA::Adafruit_SSD1306_DMA(uint8_t w, uint8_t h,
                                           SPIClass* spi, int8_t dc_pin,
                                           int8_t rst_pin, int8_t cs_pin,
                                           uint32_t bitrate)
    : Adafruit_SSD1306(w, h, spi, dc_pin, rst_pin, cs_pin, bitrate) {
  initSpiHardware();
  display_list_.push_back(this);
}

void Adafruit_SSD1306_DMA::initSpiHardware() {
  spi_ = Adafruit_SSD1306::spi;

  if (spi_ == &SPI) {
    spi_struct_ = &IMXRT_LPSPI4_S;
    spi_hardware_ =
        (SPIClass::SPI_Hardware_t*)&spi_->spiclass_lpspi4_hardware;
  } else if (spi_ == &SPI1) {
    spi_struct_ = &IMXRT_LPSPI3_S;
    spi_hardware_ =
        (SPIClass::SPI_Hardware_t*)&spi_->spiclass_lpspi3_hardware;
  } else if (spi_ == &SPI2) {
    spi_struct_ = &IMXRT_LPSPI1_S;
    spi_hardware_ =
        (SPIClass::SPI_Hardware_t*)&spi_->spiclass_lpspi1_hardware;
  }
}

// ── DMA ISR ──────────────────────────────────────────────────────────────────

void Adafruit_SSD1306_DMA::isrDMACommon() {
  std::for_each(display_list_.begin(), display_list_.end(), [](auto* disp) {
    if (disp->dma_channel_start_.complete() && disp->running_) {
      disp->isrDMA();
    }
  });
}

void Adafruit_SSD1306_DMA::isrDMA() {
  if (command_) {
    // Command phase complete — switch to 32-bit data mode for framebuffer
    dma_channel_start_.clearInterrupt();
    command_ = false;
    while (spi_struct_->FSR & 0x1f)
      ;  // drain FIFO
    while (spi_struct_->SR & LPSPI_SR_MBF)
      ;  // wait for module idle
    digitalWriteFast(Adafruit_SSD1306::dcPin, HIGH);
    spi_struct_->TCR = (spi_struct_->TCR & 0xfffff000) |
                       LPSPI_TCR_FRAMESZ(31) | LPSPI_TCR_RXMSK |
                       LPSPI_TCR_BYSW;
    spi_struct_->DER = LPSPI_DER_TDDE;
    spi_struct_->SR = 0x3f00;
    dma_channel_.enable();
    return;
  }

  // Data phase complete — clean up
  dma_channel_.clearInterrupt();
  running_ = false;
  spi_struct_->TCR = spi_struct_TCR_;
  while (spi_struct_->FSR & 0x1f)
    ;
  while (spi_struct_->SR & LPSPI_SR_MBF)
    ;
  digitalWriteFast(Adafruit_SSD1306::csPin, HIGH);
  spi_->endTransaction();
#if defined(__IMXRT1062__)
  asm("DSB");
#endif
}

// ── Public API ───────────────────────────────────────────────────────────────

void Adafruit_SSD1306_DMA::waitForAsync() {
  while (running_) {
    ;
  }
}

void Adafruit_SSD1306_DMA::displayAsync() {
  if (!spi_) return;
  if (!dma_channel_.complete() && running_) return;  // previous still going

  dma_channel_start_.clearComplete();
  dma_channel_.clearComplete();

  // ── Data channel: framebuffer → SPI TDR (32-bit words) ───────────────────
  uint8_t* buf = Adafruit_SSD1306::getBuffer();
  uint32_t buf_bytes = (Adafruit_SSD1306::width() *
                        ((Adafruit_SSD1306::height() + 7) / 8));

  dma_channel_.sourceBuffer((uint32_t*)buf, buf_bytes);
  dma_channel_.destination(spi_struct_->TDR);
  dma_channel_.transferSize(4);
  dma_channel_.transferCount(buf_bytes / 4);
  dma_channel_.interruptAtCompletion();
  dma_channel_.disableOnCompletion();
  dma_channel_.triggerAtHardwareEvent(spi_hardware_->tx_dma_channel);
  dma_channel_.attachInterrupt(isrDMACommon);
  dma_channel_.clearComplete();

  // ── Begin SPI transaction ────────────────────────────────────────────────
  digitalWriteFast(Adafruit_SSD1306::csPin, LOW);
  spi_->beginTransaction(Adafruit_SSD1306::spiSettings);
  digitalWriteFast(Adafruit_SSD1306::dcPin, LOW);

  // ── Command channel: page/column address setup (16-bit) ──────────────────
  static uint8_t dlist1[] = {
      SSD1306_PAGEADDR,
      0,                                              // page start
      0xFF,                                           // page end
      SSD1306_COLUMNADDR,
      0,                                              // column start
      static_cast<uint8_t>(WIDTH - 1)                 // column end
  };
  if (WIDTH == 64) {
    dlist1[4] = 0x20;
    dlist1[5] = 0x20 + WIDTH - 1;
  }

  if ((uint32_t)dlist1 >= 0x20200000u)
    arm_dcache_flush((void*)dlist1, sizeof(dlist1));

  dma_channel_start_.sourceBuffer((uint16_t*)dlist1, sizeof(dlist1) / 2);
  dma_channel_start_.destination(spi_struct_->TDR);
  dma_channel_start_.transferSize(2);
  dma_channel_start_.transferCount(sizeof(dlist1) / 2);
  dma_channel_start_.interruptAtCompletion();
  dma_channel_start_.attachInterrupt(isrDMACommon);
  dma_channel_start_.disableOnCompletion();
  dma_channel_start_.triggerAtHardwareEvent(spi_hardware_->tx_dma_channel);
  command_ = true;

  // Start in 16-bit transfer mode
  spi_struct_TCR_ = spi_struct_->TCR;
  spi_struct_->TCR = (spi_struct_->TCR & 0xfffff000) |
                     LPSPI_TCR_FRAMESZ(15) | LPSPI_TCR_RXMSK |
                     LPSPI_TCR_BYSW;
  spi_struct_->DER = LPSPI_DER_TDDE;
  spi_struct_->SR = 0x3f00;

  if ((uint32_t)buf >= 0x20200000u)
    arm_dcache_flush((void*)buf, buf_bytes);

  dma_channel_start_.enable();
  running_ = true;
}
