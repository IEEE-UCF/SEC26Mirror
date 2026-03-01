/**
 * @file QTimerEncoder.h
 * @brief IMXRT1062 QTimer-based FG pulse counter for 8 motor encoders.
 * @date 2026-02-27
 *
 * Uses hardware QTimers + XBAR crossbar for zero-CPU-overhead pulse counting.
 * No software interrupts — counters are read atomically from 16-bit CNTR regs.
 *
 * Pin Routing:
 *   Pin 2 (EMC_04) -> XBAR In 6  -> Out 86 -> QTimer1 Timer 0
 *   Pin 3 (EMC_05) -> XBAR In 7  -> Out 87 -> QTimer1 Timer 1
 *   Pin 4 (EMC_06) -> XBAR In 8  -> Out 88 -> QTimer1 Timer 2
 *   Pin 5 (EMC_08) -> XBAR In 17 -> Out 89 -> QTimer1 Timer 3
 *   Pin 6 (B0_10)  -> Direct TMR4 ch1       -> QTimer4 Timer 1
 *   Pin 7 (B1_01)  -> XBAR In 15 -> Out 90 -> QTimer2 Timer 0
 *   Pin 8 (B1_00)  -> XBAR In 14 -> Out 91 -> QTimer2 Timer 1
 *   Pin 9 (B0_11)  -> Direct TMR4 ch2       -> QTimer4 Timer 2
 */
 #pragma once
 #include <stdint.h>
 namespace Encoders {
 static constexpr uint8_t NUM_ENCODER_CHANNELS = 8;
 struct EncoderChannel {
  volatile uint16_t* cntr_reg;
  volatile uint16_t* dir_reg;
};
 class QTimerEncoder {
 public:
  QTimerEncoder() = default;
   /**
   * @brief Initialize all 8 QTimer encoder channels.
   * Configures clock gates, IOMUX, XBAR routing, and QTimer registers.
   * Must be called once during setup().
   */
  bool init();
   /**
   * @brief Read the raw 16-bit counter value for a channel.
   * @param channel 0-7.
   */
  int16_t readRaw(uint8_t channel) const;
   /**
   * @brief Get delta for a channel and reset
   * @param channel 0-7.
   */
  int16_t getDelta(uint8_t channel) const;
   /**
   * @brief Change count direction for a channel.
   * @param channel 0-7.
   * @param dir true = count down (negative), false = count up (positive).
   */
  void changeDir(uint8_t channel, bool dir);

  /**
   * @brief Capture deltas from all channels, accumulate signed ticks,
   *        and update hardware count direction for the next period.
   * @param dirs Array of 8 booleans from MotorManager::getIntendedDirection().
   *             true = forward (count up), false = reverse (count down).
   */
  void captureAll(const bool* dirs);

  /**
   * @brief Get accumulated signed ticks for a channel.
   * @param channel 0-7.
   */
  int32_t getTicks(uint8_t channel) const;

  /**
   * @brief Reset all accumulators and hardware counters to zero.
   */
  void resetAll();

 private:
  EncoderChannel channels_[NUM_ENCODER_CHANNELS] = {};
  int32_t ticks_[NUM_ENCODER_CHANNELS] = {};
  void enableClockGates();
  void configureIOMUX();
  void configureXBAR();
  void configureQTimers();
};
 }  // namespace Encoders