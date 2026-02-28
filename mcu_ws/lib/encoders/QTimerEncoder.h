/**
 * @file QTimerEncoder.h
 * @brief IMXRT1062 QTimer-based FG pulse counter for 8 motor encoders.
 * @date 2026-02-27
 *
 * Uses hardware QTimers + XBAR crossbar for zero-CPU-overhead pulse counting
 * on 6 channels, and GPIO interrupt counting on 2 channels.
 *
 * Pin Routing:
 *   Pin 2 (EMC_04) -> XBAR In 6  -> Out 86 -> QTimer1 Timer 0  [HW]
 *   Pin 3 (EMC_05) -> XBAR In 7  -> Out 87 -> QTimer1 Timer 1  [HW]
 *   Pin 4 (EMC_06) -> XBAR In 8  -> Out 88 -> QTimer1 Timer 2  [HW]
 *   Pin 5 (EMC_08) -> XBAR In 17 -> Out 89 -> QTimer1 Timer 3  [HW]
 *   Pin 6 (B0_10)  -> GPIO interrupt         (software counter) [SW]
 *   Pin 7 (B1_01)  -> XBAR In 15 -> Out 90 -> QTimer2 Timer 0  [HW]
 *   Pin 8 (B1_00)  -> XBAR In 14 -> Out 91 -> QTimer2 Timer 1  [HW]
 *   Pin 9 (B0_11)  -> GPIO interrupt         (software counter) [SW]
 *
 * Note: Pins 6 and 9 cannot use QTimer4 — the Teensy core uses all four
 * TMR4 channels for internal timing (CTRL=0x300B: 150 MHz bus clock counter
 * with output toggle, ENBL=0x000F). Writing any TMR4 CTRL register kills USB.
 * GPIO interrupt counting is used for these two pins instead.
 */

#pragma once

#include <stdint.h>

namespace Encoders {

static constexpr uint8_t NUM_ENCODER_CHANNELS = 8;
static constexpr uint8_t PIN6_CHANNEL = 4;
static constexpr uint8_t PIN9_CHANNEL = 7;

struct EncoderChannel {
  volatile uint16_t* cntr_reg;
  uint16_t last_count;
  int32_t accumulated_ticks;
};

class QTimerEncoder {
 public:
  QTimerEncoder() = default;

  /**
   * @brief Initialize all 8 encoder channels.
   * Configures clock gates, IOMUX, XBAR routing, QTimer registers (ch 0-3,5-6),
   * and GPIO interrupts for channels 4,7 (pins 6,9).
   * Must be called once during setup().
   */
  bool init();

  /**
   * @brief Read the raw 16-bit counter value for a channel.
   * @param channel 0-7.
   */
  uint16_t readRaw(uint8_t channel) const;

  /**
   * @brief Capture deltas for all 8 channels.
   * Reads all counters, computes unsigned deltas (handles 16-bit wrap),
   * applies direction sign, and accumulates into signed tick counts.
   * @param directions Array of 8 bools: true = forward, false = reverse.
   */
  void captureAll(const bool directions[NUM_ENCODER_CHANNELS]);

  /**
   * @brief Get accumulated signed ticks for a channel since last reset.
   * @param channel 0-7.
   */
  int32_t getTicks(uint8_t channel) const;

  /**
   * @brief Reset accumulated ticks for a channel to zero.
   * @param channel 0-7.
   */
  void resetTicks(uint8_t channel);

  /** @brief Reset all channels' accumulated ticks to zero. */
  void resetAll();

 private:
  EncoderChannel channels_[NUM_ENCODER_CHANNELS] = {};

  // Software counters for pins 6 and 9 (GPIO interrupt)
  volatile uint16_t sw_counter_pin6_ = 0;
  volatile uint16_t sw_counter_pin9_ = 0;

  // ISR support
  static QTimerEncoder* s_instance_;
  static void pin6ISR();
  static void pin9ISR();

  void enableClockGates();
  void configureIOMUX();
  void configureXBAR();
  void configureQTimers();
  void configureGPIOInterrupts();
};

}  // namespace Encoders
