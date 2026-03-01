#include <cstdint>
#include "imxrt.h"
#include "core_pins.h"
/**
 * @file QTimerEncoder.cpp
 * @brief IMXRT1062 QTimer + XBAR register configuration for 8 FG encoders.
 * @date 2026-02-28
 *
 * Channel-to-register mapping:
 *   [0] Pin 2 -> QTimer3 Timer 0  (XBAR routed)
 *   [1] Pin 3 -> QTimer3 Timer 1  (XBAR routed)
 *   [2] Pin 4 -> QTimer3 Timer 2  (XBAR routed)
 *   [3] Pin 5 -> QTimer3 Timer 3  (XBAR routed)
 *   [4] Pin 6 -> QTimer4 Timer 0  (direct pad connection)
 *   [5] Pin 7 -> QTimer4 Timer 1  (XBAR routed)
 *   [6] Pin 8 -> QTimer4 Timer 2  (XBAR routed)
 *   [7] Pin 9 -> QTimer4 Timer 3  (direct pad connection)
 */
#include "QTimerEncoder.h"
#include <Arduino.h>
namespace Encoders {
extern "C" void xbar_connect(unsigned int input, unsigned int output);  // in pwm.c
                                                                        // Pad control: hysteresis, 100k pulldown, fast slew — suitable for
// open-collector / push-pull FG signals.
// static constexpr uint32_t FG_PAD_CTL = 0x10B0;
static constexpr uint32_t FG_PAD_CTL = IOMUXC_PAD_DSE(7) | IOMUXC_PAD_PKE | IOMUXC_PAD_PUE | IOMUXC_PAD_PUS(3) | IOMUXC_PAD_HYS;
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
    *channels_[i].cntr_reg = UINT16_C(UINT16_MAX/2);
  }
  return true;
}
// ═══════════════════════════════════════════════════════════════════════════
//  Clock gates
// ═══════════════════════════════════════════════════════════════════════════
void QTimerEncoder::enableClockGates() {
  CCM_CCGR2 |= CCM_CCGR2_XBAR1(CCM_CCGR_ON);
  CCM_CCGR6 |= CCM_CCGR6_QTIMER3(CCM_CCGR_ON) | CCM_CCGR6_QTIMER4(CCM_CCGR_ON);
}
// ═══════════════════════════════════════════════════════════════════════════
//  IOMUX — pin muxing and daisy-chain select
// ═══════════════════════════════════════════════════════════════════════════
void QTimerEncoder::configureIOMUX() {
  // ── Pins 2-5: ALT3 = XBAR1_INOUTxx ──────────────────────────────────
  CORE_PIN2_CONFIG = 3;  // Pin 2 -> XBAR1_INOUT06
  CORE_PIN2_PADCONFIG = FG_PAD_CTL;
  CORE_PIN3_CONFIG = 3;  // Pin 3 -> XBAR1_INOUT07
  CORE_PIN3_PADCONFIG = FG_PAD_CTL;
  CORE_PIN4_CONFIG = 3;  // Pin 4 -> XBAR1_INOUT08
  CORE_PIN4_PADCONFIG = FG_PAD_CTL;
  CORE_PIN5_CONFIG = 3;  // Pin 5 -> XBAR1_INOUT17
  CORE_PIN5_PADCONFIG = FG_PAD_CTL;
  // ── Pin 6: ALT1 = direct QTIMER4_TIMER1 ─────────────────────────────
  CORE_PIN6_CONFIG = 1;
  CORE_PIN6_PADCONFIG = FG_PAD_CTL;
  // ── Pins 7-8: ALT1 = XBAR1_INOUTxx ──────────────────────────────────
  CORE_PIN7_CONFIG = 1;  // Pin 7 -> XBAR1_INOUT15
  CORE_PIN7_PADCONFIG = FG_PAD_CTL;
  CORE_PIN8_CONFIG = 1;  // Pin 8 -> XBAR1_INOUT14
  CORE_PIN8_PADCONFIG = FG_PAD_CTL;
  // ── Pin 9: ALT1 = direct QTIMER4_TIMER2 ─────────────────────────────
  CORE_PIN9_CONFIG = 1;
  CORE_PIN9_PADCONFIG = FG_PAD_CTL;
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
  xbar_connect(XBARA1_IN_IOMUX_XBAR_INOUT06, XBARA1_OUT_QTIMER3_TIMER0);
  xbar_connect(XBARA1_IN_IOMUX_XBAR_INOUT07, XBARA1_OUT_QTIMER3_TIMER1);
  xbar_connect(XBARA1_IN_IOMUX_XBAR_INOUT08, XBARA1_OUT_QTIMER3_TIMER2);
  xbar_connect(XBARA1_IN_IOMUX_XBAR_INOUT17, XBARA1_OUT_QTIMER3_TIMER3);
  xbar_connect(XBARA1_IN_IOMUX_XBAR_INOUT15, XBARA1_OUT_QTIMER4_TIMER0);
  xbar_connect(XBARA1_IN_IOMUX_XBAR_INOUT14, XBARA1_OUT_QTIMER4_TIMER3);
  IOMUXC_GPR_GPR6 &= ~(IOMUXC_GPR_GPR6_IOMUXC_XBAR_DIR_SEL_6 | IOMUXC_GPR_GPR6_IOMUXC_XBAR_DIR_SEL_7 |
                     IOMUXC_GPR_GPR6_IOMUXC_XBAR_DIR_SEL_8 | IOMUXC_GPR_GPR6_IOMUXC_XBAR_DIR_SEL_17 |
                     IOMUXC_GPR_GPR6_IOMUXC_XBAR_DIR_SEL_15 | IOMUXC_GPR_GPR6_IOMUXC_XBAR_DIR_SEL_14);
  IOMUXC_GPR_GPR6 |= IOMUXC_GPR_GPR6_QTIMER3_TRM0_INPUT_SEL | IOMUXC_GPR_GPR6_QTIMER3_TRM1_INPUT_SEL | 
                     IOMUXC_GPR_GPR6_QTIMER3_TRM2_INPUT_SEL | IOMUXC_GPR_GPR6_QTIMER3_TRM3_INPUT_SEL |
                     IOMUXC_GPR_GPR6_QTIMER4_TRM0_INPUT_SEL | IOMUXC_GPR_GPR6_QTIMER4_TRM3_INPUT_SEL;
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
  // ── QTimer3: channels 0-3 (pins 2,3,4,5 via XBAR) ───────────────────
  // Channel 0 (Pin 2)
  TMR3_CNTR0 = 0;
  TMR3_SCTRL0 = TMR_SCTRL_IPS;
  TMR3_FILT0 = TMR_FILT_FILT_PER(FG_FILT_PERIOD);
  TMR3_CTRL0 = TMR_CTRL_CM(1) | TMR_CTRL_PCS(0);
  // Channel 1 (Pin 3)
  TMR3_CNTR1 = 0;
  TMR3_SCTRL1 = TMR_SCTRL_IPS;
  TMR3_FILT1 = TMR_FILT_FILT_PER(FG_FILT_PERIOD);
  TMR3_CTRL1 = TMR_CTRL_CM(1) | TMR_CTRL_PCS(1);
  // Channel 2 (Pin 4)
  TMR3_CNTR2 = 0;
  TMR3_SCTRL2 = TMR_SCTRL_IPS;
  TMR3_FILT2 = TMR_FILT_FILT_PER(FG_FILT_PERIOD);
  TMR3_CTRL2 = TMR_CTRL_CM(1) | TMR_CTRL_PCS(2);
  // Channel 3 (Pin 5)
  TMR3_CNTR3 = 0;
  TMR3_SCTRL3 = TMR_SCTRL_IPS;
  TMR3_FILT3 = TMR_FILT_FILT_PER(FG_FILT_PERIOD);
  TMR3_CTRL3 = TMR_CTRL_CM(1) | TMR_CTRL_PCS(3);
  // ── QTimer4 Channel 0 (Pin 6, direct) ────────────────────────────────
  TMR4_CNTR0 = 0;
  TMR4_SCTRL0 = TMR_SCTRL_IPS;
  TMR4_FILT0 = TMR_FILT_FILT_PER(FG_FILT_PERIOD);
  TMR4_CTRL0 = TMR_CTRL_CM(1) | TMR_CTRL_PCS(1);
  // ── QTimer4 Channel 1 (Pin 7, via XBAR) ──────────────────────────────
  TMR4_CNTR1 = 0;
  TMR4_SCTRL1 = TMR_SCTRL_IPS;
  TMR4_FILT1 = TMR_FILT_FILT_PER(FG_FILT_PERIOD);
  TMR4_CTRL1 = TMR_CTRL_CM(1) | TMR_CTRL_PCS(0);
  // ── QTimer4 Channel 2 (Pin 8, via XBAR) ──────────────────────────────
  TMR4_CNTR2 = 0;
  TMR4_SCTRL2 = TMR_SCTRL_IPS;
  TMR4_FILT2 = TMR_FILT_FILT_PER(FG_FILT_PERIOD);
  TMR4_CTRL2 = TMR_CTRL_CM(1) | TMR_CTRL_PCS(3);
  // ── QTimer4 Channel 3 (Pin 9, direct) ────────────────────────────────
  TMR4_CNTR3 = 0;
  TMR4_SCTRL3 = TMR_SCTRL_IPS;
  TMR4_FILT3 = TMR_FILT_FILT_PER(FG_FILT_PERIOD);
  TMR4_CTRL3 = TMR_CTRL_CM(1) | TMR_CTRL_PCS(2);
  // ── Set CNTR register pointers ────────────────────────────────────────
  channels_[0].cntr_reg = &TMR3_CNTR0;
  channels_[1].cntr_reg = &TMR3_CNTR1;
  channels_[2].cntr_reg = &TMR3_CNTR2;
  channels_[3].cntr_reg = &TMR3_CNTR3;
  channels_[4].cntr_reg = &TMR4_CNTR0;
  channels_[5].cntr_reg = &TMR4_CNTR1;
  channels_[6].cntr_reg = &TMR4_CNTR2;
  channels_[7].cntr_reg = &TMR4_CNTR3;
  channels_[0].dir_reg = &TMR3_CTRL0;
  channels_[1].dir_reg = &TMR3_CTRL1;
  channels_[2].dir_reg = &TMR3_CTRL2;
  channels_[3].dir_reg = &TMR3_CTRL3;
  channels_[4].dir_reg = &TMR4_CTRL0;
  channels_[5].dir_reg = &TMR4_CTRL1;
  channels_[6].dir_reg = &TMR4_CTRL2;
  channels_[7].dir_reg = &TMR4_CTRL3;
  // ── Enable timer channels ────────────────────────────────────────────
  TMR3_ENBL |= 0b1111;  // QTimer3 channels 0-3
  TMR4_ENBL |= 0b1111;  // QTimer4 channels 0-3
}
// ═══════════════════════════════════════════════════════════════════════════
//  Public API
// ═══════════════════════════════════════════════════════════════════════════
int16_t QTimerEncoder::readRaw(uint8_t channel) const {
  if (channel >= NUM_ENCODER_CHANNELS) return 0;
  return (int16_t)*channels_[channel].cntr_reg - UINT16_C(UINT16_MAX/2);
}

int16_t QTimerEncoder::getDelta(uint8_t channel) const {
  if (channel >= NUM_ENCODER_CHANNELS) return 0;
  int16_t delta = (int16_t)*channels_[channel].cntr_reg - UINT16_C(UINT16_MAX/2);
  *channels_[channel].cntr_reg = UINT16_C(UINT16_MAX/2);
  return delta;
}

void QTimerEncoder::changeDir(uint8_t channel, bool dir) {
  if (channel >= NUM_ENCODER_CHANNELS) return;
  if(dir) *channels_[channel].dir_reg |= TMR_CTRL_DIR;
  else *channels_[channel].dir_reg &= ~TMR_CTRL_DIR;
}
}  // namespace Encoders