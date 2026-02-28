/**
 * @file QTimerEncoder.cpp
 * @brief IMXRT1062 QTimer + XBAR register configuration for 8 FG encoders.
 * @date 2026-02-27
 *
 * Channel-to-register mapping:
 *   [0] Pin 2 -> QTimer1 Timer 0  (XBAR routed)
 *   [1] Pin 3 -> QTimer1 Timer 1  (XBAR routed)
 *   [2] Pin 4 -> QTimer1 Timer 2  (XBAR routed)
 *   [3] Pin 5 -> QTimer1 Timer 3  (XBAR routed)
 *   [4] Pin 6 -> QTimer4 Timer 1  (direct pad connection)
 *   [5] Pin 7 -> QTimer2 Timer 0  (XBAR routed)
 *   [6] Pin 8 -> QTimer2 Timer 1  (XBAR routed)
 *   [7] Pin 9 -> QTimer4 Timer 2  (direct pad connection)
 */

#include "QTimerEncoder.h"

#include <Arduino.h>

namespace Encoders {

// Pad control: hysteresis, 100k pulldown, fast slew — suitable for
// open-collector / push-pull FG signals.
static constexpr uint32_t FG_PAD_CTL = 0x10B0;

// Small input filter period for debouncing FG edges (~3 IPbus clocks).
// At 150 MHz IPbus this is ~20 ns — far below any legitimate FG period.
static constexpr uint16_t FG_FILT_PERIOD = 3;

bool QTimerEncoder::init() {
  enableClockGates();
  configureIOMUX();
  configureXBAR();
  configureQTimers();

  // Snapshot initial counter values
  for (uint8_t i = 0; i < NUM_ENCODER_CHANNELS; i++) {
    channels_[i].last_count = *channels_[i].cntr_reg;
    channels_[i].accumulated_ticks = 0;
  }

  return true;
}

// ═══════════════════════════════════════════════════════════════════════════
//  Clock gates
// ═══════════════════════════════════════════════════════════════════════════

void QTimerEncoder::enableClockGates() {
  CCM_CCGR2 |= CCM_CCGR2_XBAR1(CCM_CCGR_ON);
  CCM_CCGR6 |= CCM_CCGR6_QTIMER1(CCM_CCGR_ON) |
                CCM_CCGR6_QTIMER2(CCM_CCGR_ON) |
                CCM_CCGR6_QTIMER4(CCM_CCGR_ON);
}

// ═══════════════════════════════════════════════════════════════════════════
//  IOMUX — pin muxing and daisy-chain select
// ═══════════════════════════════════════════════════════════════════════════

void QTimerEncoder::configureIOMUX() {
  // ── Pins 2-5: ALT3 = XBAR1_INOUTxx ──────────────────────────────────
  IOMUXC_SW_MUX_CTL_PAD_GPIO_EMC_04 = 3;  // Pin 2 -> XBAR1_INOUT06
  IOMUXC_SW_PAD_CTL_PAD_GPIO_EMC_04 = FG_PAD_CTL;

  IOMUXC_SW_MUX_CTL_PAD_GPIO_EMC_05 = 3;  // Pin 3 -> XBAR1_INOUT07
  IOMUXC_SW_PAD_CTL_PAD_GPIO_EMC_05 = FG_PAD_CTL;

  IOMUXC_SW_MUX_CTL_PAD_GPIO_EMC_06 = 3;  // Pin 4 -> XBAR1_INOUT08
  IOMUXC_SW_PAD_CTL_PAD_GPIO_EMC_06 = FG_PAD_CTL;

  IOMUXC_SW_MUX_CTL_PAD_GPIO_EMC_08 = 3;  // Pin 5 -> XBAR1_INOUT17
  IOMUXC_SW_PAD_CTL_PAD_GPIO_EMC_08 = FG_PAD_CTL;

  // ── Pin 6: ALT1 = direct QTIMER4_TIMER1 ─────────────────────────────
  IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_10 = 1;
  IOMUXC_SW_PAD_CTL_PAD_GPIO_B0_10 = FG_PAD_CTL;

  // ── Pins 7-8: ALT1 = XBAR1_INOUTxx ──────────────────────────────────
  IOMUXC_SW_MUX_CTL_PAD_GPIO_B1_01 = 1;  // Pin 7 -> XBAR1_INOUT15
  IOMUXC_SW_PAD_CTL_PAD_GPIO_B1_01 = FG_PAD_CTL;

  IOMUXC_SW_MUX_CTL_PAD_GPIO_B1_00 = 1;  // Pin 8 -> XBAR1_INOUT14
  IOMUXC_SW_PAD_CTL_PAD_GPIO_B1_00 = FG_PAD_CTL;

  // ── Pin 9: ALT1 = direct QTIMER4_TIMER2 ─────────────────────────────
  IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_11 = 1;
  IOMUXC_SW_PAD_CTL_PAD_GPIO_B0_11 = FG_PAD_CTL;

  // ── XBAR input daisy-chain select ────────────────────────────────────
  // EMC pads: value 0 selects the default source pad.
  IOMUXC_XBAR1_IN06_SELECT_INPUT = 0;  // GPIO_EMC_04
  IOMUXC_XBAR1_IN07_SELECT_INPUT = 0;  // GPIO_EMC_05
  IOMUXC_XBAR1_IN08_SELECT_INPUT = 0;  // GPIO_EMC_06
  IOMUXC_XBAR1_IN17_SELECT_INPUT = 0;  // GPIO_EMC_08

  // B-port pads: value 1 selects GPIO_B1_xx for the XBAR input.
  IOMUXC_XBAR1_IN14_SELECT_INPUT = 1;  // GPIO_B1_00
  IOMUXC_XBAR1_IN15_SELECT_INPUT = 1;  // GPIO_B1_01
}

// ═══════════════════════════════════════════════════════════════════════════
//  XBAR — crossbar input-to-output routing
// ═══════════════════════════════════════════════════════════════════════════

void QTimerEncoder::configureXBAR() {
  // Each SEL register is 16 bits: [6:0] = even output, [14:8] = odd output.
  //
  // SEL43 -> Out 86 (QTimer1 Timer0), Out 87 (QTimer1 Timer1)
  // SEL44 -> Out 88 (QTimer1 Timer2), Out 89 (QTimer1 Timer3)
  // SEL45 -> Out 90 (QTimer2 Timer0), Out 91 (QTimer2 Timer1)

  XBARA1_SEL43 = (6) | (7 << 8);    // In6 (Pin2) -> TMR1_0, In7 (Pin3) -> TMR1_1
  XBARA1_SEL44 = (8) | (17 << 8);   // In8 (Pin4) -> TMR1_2, In17 (Pin5) -> TMR1_3
  XBARA1_SEL45 = (15) | (14 << 8);  // In15 (Pin7) -> TMR2_0, In14 (Pin8) -> TMR2_1
}

// ═══════════════════════════════════════════════════════════════════════════
//  QTimer — configure each channel to count rising edges on external input
// ═══════════════════════════════════════════════════════════════════════════

void QTimerEncoder::configureQTimers() {
  // CTRL register:
  //   CM  [15:13] = 001  -> count rising edges of primary source
  //   PCS [12:9]  = N    -> primary count source = counter N input pin
  //                         (either XBAR-routed or direct pad)
  //   All other bits = 0 -> count up, free-running, no compare stop

  // ── QTimer1: channels 0-3 (pins 2,3,4,5 via XBAR) ───────────────────
  // Channel 0 (Pin 2)
  TMR1_CTRL0 = 0;
  TMR1_CNTR0 = 0;
  TMR1_LOAD0 = 0;
  TMR1_COMP10 = 0xFFFF;
  TMR1_SCTRL0 = 0;
  TMR1_FILT0 = FG_FILT_PERIOD;
  TMR1_CTRL0 = TMR_CTRL_CM(1) | TMR_CTRL_PCS(0);

  // Channel 1 (Pin 3)
  TMR1_CTRL1 = 0;
  TMR1_CNTR1 = 0;
  TMR1_LOAD1 = 0;
  TMR1_COMP11 = 0xFFFF;
  TMR1_SCTRL1 = 0;
  TMR1_FILT1 = FG_FILT_PERIOD;
  TMR1_CTRL1 = TMR_CTRL_CM(1) | TMR_CTRL_PCS(1);

  // Channel 2 (Pin 4)
  TMR1_CTRL2 = 0;
  TMR1_CNTR2 = 0;
  TMR1_LOAD2 = 0;
  TMR1_COMP12 = 0xFFFF;
  TMR1_SCTRL2 = 0;
  TMR1_FILT2 = FG_FILT_PERIOD;
  TMR1_CTRL2 = TMR_CTRL_CM(1) | TMR_CTRL_PCS(2);

  // Channel 3 (Pin 5)
  TMR1_CTRL3 = 0;
  TMR1_CNTR3 = 0;
  TMR1_LOAD3 = 0;
  TMR1_COMP13 = 0xFFFF;
  TMR1_SCTRL3 = 0;
  TMR1_FILT3 = FG_FILT_PERIOD;
  TMR1_CTRL3 = TMR_CTRL_CM(1) | TMR_CTRL_PCS(3);

  // ── QTimer4 Channel 1 (Pin 6, direct) ────────────────────────────────
  TMR4_CTRL1 = 0;
  TMR4_CNTR1 = 0;
  TMR4_LOAD1 = 0;
  TMR4_COMP11 = 0xFFFF;
  TMR4_SCTRL1 = 0;
  TMR4_FILT1 = FG_FILT_PERIOD;
  TMR4_CTRL1 = TMR_CTRL_CM(1) | TMR_CTRL_PCS(1);

  // ── QTimer2 Channel 0 (Pin 7, via XBAR) ──────────────────────────────
  TMR2_CTRL0 = 0;
  TMR2_CNTR0 = 0;
  TMR2_LOAD0 = 0;
  TMR2_COMP10 = 0xFFFF;
  TMR2_SCTRL0 = 0;
  TMR2_FILT0 = FG_FILT_PERIOD;
  TMR2_CTRL0 = TMR_CTRL_CM(1) | TMR_CTRL_PCS(0);

  // ── QTimer2 Channel 1 (Pin 8, via XBAR) ──────────────────────────────
  TMR2_CTRL1 = 0;
  TMR2_CNTR1 = 0;
  TMR2_LOAD1 = 0;
  TMR2_COMP11 = 0xFFFF;
  TMR2_SCTRL1 = 0;
  TMR2_FILT1 = FG_FILT_PERIOD;
  TMR2_CTRL1 = TMR_CTRL_CM(1) | TMR_CTRL_PCS(1);

  // ── QTimer4 Channel 2 (Pin 9, direct) ────────────────────────────────
  TMR4_CTRL2 = 0;
  TMR4_CNTR2 = 0;
  TMR4_LOAD2 = 0;
  TMR4_COMP12 = 0xFFFF;
  TMR4_SCTRL2 = 0;
  TMR4_FILT2 = FG_FILT_PERIOD;
  TMR4_CTRL2 = TMR_CTRL_CM(1) | TMR_CTRL_PCS(2);

  // ── Set CNTR register pointers ────────────────────────────────────────
  channels_[0].cntr_reg = &IMXRT_TMR1.CH[0].CNTR;
  channels_[1].cntr_reg = &IMXRT_TMR1.CH[1].CNTR;
  channels_[2].cntr_reg = &IMXRT_TMR1.CH[2].CNTR;
  channels_[3].cntr_reg = &IMXRT_TMR1.CH[3].CNTR;
  channels_[4].cntr_reg = &IMXRT_TMR4.CH[1].CNTR;
  channels_[5].cntr_reg = &IMXRT_TMR2.CH[0].CNTR;
  channels_[6].cntr_reg = &IMXRT_TMR2.CH[1].CNTR;
  channels_[7].cntr_reg = &IMXRT_TMR4.CH[2].CNTR;

  // ── Enable timer channels ────────────────────────────────────────────
  TMR1_ENBL |= 0x0F;  // QTimer1 channels 0-3
  TMR2_ENBL |= 0x03;  // QTimer2 channels 0-1
  TMR4_ENBL |= 0x06;  // QTimer4 channels 1-2
}

// ═══════════════════════════════════════════════════════════════════════════
//  Public API
// ═══════════════════════════════════════════════════════════════════════════

uint16_t QTimerEncoder::readRaw(uint8_t channel) const {
  if (channel >= NUM_ENCODER_CHANNELS) return 0;
  return *channels_[channel].cntr_reg;
}

void QTimerEncoder::captureAll(const bool directions[NUM_ENCODER_CHANNELS]) {
  // Read all 8 counters as close together as possible to minimize skew
  uint16_t counts[NUM_ENCODER_CHANNELS];
  for (uint8_t i = 0; i < NUM_ENCODER_CHANNELS; i++) {
    counts[i] = *channels_[i].cntr_reg;
  }

  // Compute deltas and accumulate with direction sign.
  // Unsigned subtraction handles 16-bit rollover correctly.
  for (uint8_t i = 0; i < NUM_ENCODER_CHANNELS; i++) {
    uint16_t delta = counts[i] - channels_[i].last_count;
    channels_[i].last_count = counts[i];

    if (directions[i]) {
      channels_[i].accumulated_ticks += (int32_t)delta;
    } else {
      channels_[i].accumulated_ticks -= (int32_t)delta;
    }
  }
}

int32_t QTimerEncoder::getTicks(uint8_t channel) const {
  if (channel >= NUM_ENCODER_CHANNELS) return 0;
  return channels_[channel].accumulated_ticks;
}

void QTimerEncoder::resetTicks(uint8_t channel) {
  if (channel >= NUM_ENCODER_CHANNELS) return;
  channels_[channel].accumulated_ticks = 0;
  channels_[channel].last_count = *channels_[channel].cntr_reg;
}

void QTimerEncoder::resetAll() {
  for (uint8_t i = 0; i < NUM_ENCODER_CHANNELS; i++) {
    resetTicks(i);
  }
}

}  // namespace Encoders
