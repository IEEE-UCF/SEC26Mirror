#pragma once

#include <Adafruit_SSD1306.h>
#include <DMAChannel.h>
#include <SPI.h>
#include <list>

/**
 * @brief DMA-accelerated SSD1306 driver for Teensy 4.x (IMXRT1062).
 *
 * Extends Adafruit_SSD1306 with displayAsync() which offloads the SPI
 * framebuffer transfer to DMA hardware, freeing the CPU during display
 * updates.  The two-phase transfer sends page/column address commands
 * (16-bit) then framebuffer data (32-bit) entirely via DMA interrupts.
 */
class Adafruit_SSD1306_DMA : public Adafruit_SSD1306 {
 public:
  Adafruit_SSD1306_DMA(uint8_t w, uint8_t h, SPIClass* spi, int8_t dc_pin,
                       int8_t rst_pin, int8_t cs_pin,
                       uint32_t bitrate = 8000000UL);

  /** Start an async DMA transfer of the framebuffer. Non-blocking. */
  void displayAsync();

  /** Block until an in-flight async transfer completes. */
  void waitForAsync();

  bool isAsyncRunning() const { return running_; }

 private:
  void initSpiHardware();

  SPIClass* spi_ = nullptr;
  IMXRT_LPSPI_t* spi_struct_ = nullptr;
  uint32_t spi_struct_TCR_;
  SPIClass::SPI_Hardware_t* spi_hardware_ = nullptr;

  static std::list<Adafruit_SSD1306_DMA*> display_list_;

  DMAChannel dma_channel_start_;
  DMAChannel dma_channel_;
  volatile bool running_ = false;
  volatile bool command_ = false;

  static void isrDMACommon();
  void isrDMA();
};
