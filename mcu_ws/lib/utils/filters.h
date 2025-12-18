#pragma once
/**
 * @file filters.h
 * @author Rafeed Khan
 * @brief Signal filtering utilities for noisy sensor data.
 *
 * Includes:
 *  - median3: Spike rejection for sensors with occasional outliers (ToF, IR)
 *  - MovingAverage<N>: O(1) circular buffer average, good for steady smoothing
 *  - LowPass1P: Single-pole IIR filter, configurable by alpha, tau, or cutoff freq
 *
 * All filters are statically allocated and suitable for real-time MCU use!!! so please use em!!
 */

#include <cstddef>
#include <cstdint>
#include <cmath>

#include "math.h"  // clamp(), kPi, etc

namespace secbot::utils {

// ------------------------------------------------------------
// Median-of-3 (tiny spike killer)
// Useful for ToF / IR sensors that occasionally jump
// ------------------------------------------------------------
inline float median3(float a, float b, float c) {
  // returns middle value without sorting arrays
  if (a > b) { float t=a; a=b; b=t; }
  if (b > c) { float t=b; b=c; c=t; }
  if (a > b) { float t=a; a=b; b=t; }
  return b;
}

// ------------------------------------------------------------
// Moving Average (Running Average) - O(1) update
// Circular buffer + running sum (no shifting arrays)
// ------------------------------------------------------------
template <std::size_t N>
class MovingAverage {
  static_assert(N > 0, "MovingAverage<N>: N must be > 0");

public:
  MovingAverage() { reset(0.0f); }

  // Fill buffer with a known value so the initial output is stable
  inline void reset(float fill_value = 0.0f) {
    sum_ = 0.0f;
    idx_ = 0;
    count_ = 0;
    for (std::size_t i = 0; i < N; ++i) {
      buf_[i] = fill_value;
    }
    // If you want it to start "fully settled" immediately:
    // uncomment next 3 lines
    // sum_ = fill_value * float(N);
    // count_ = N;
    // idx_ = 0;
    // but that shouldnt be an issue ;)
  }

  // Push a new sample, returns current average
  inline float update(float x) {
    if (count_ < N) {
      // still filling
      buf_[idx_] = x;
      sum_ += x;
      ++count_;
      idx_ = (idx_ + 1) % N;
      return sum_ / float(count_);
    } else {
      // steady-state
      const float old = buf_[idx_];
      buf_[idx_] = x;
      sum_ += (x - old);
      idx_ = (idx_ + 1) % N;
      return sum_ / float(N);
    }
  }

  inline float value() const {
    return (count_ == 0) ? 0.0f : (sum_ / float((count_ < N) ? count_ : N));
  }

  inline bool filled() const { return count_ >= N; }
  inline std::size_t count() const { return count_; }

private:
  float buf_[N];
  float sum_;
  std::size_t idx_;
  std::size_t count_;
};

// ------------------------------------------------------------
// First-order low-pass (single-pole IIR / exponential smoothing)
// y = y + alpha * (x - y)
// alpha in (0,1]: smaller alpha = smoother but slower response
//
// Two ways to pick alpha:
//  1) "Fast" RC discretization: alpha = dt / (tau + dt)
//  2) "Exact" time-constant mapping: alpha = 1 - exp(-dt/tau)
// ------------------------------------------------------------
class LowPass1P {
public:
  LowPass1P() : y_(0.0f), alpha_(1.0f), initialized_(false) {}

  inline void reset(float y0 = 0.0f, bool initialized = true) {
    y_ = y0;
    initialized_ = initialized;
  }

  // Set alpha directly (0..1]. alpha=1 means no filtering
  inline void setAlpha(float alpha) {
    alpha_ = clamp(alpha, 0.0f, 1.0f);
    if (alpha_ == 0.0f) alpha_ = 1.0f; // avoid "stuck" filter
  }

  // Configure via time constant tau (seconds) and dt (seconds), FAST (no expf)
  inline void configureTauDtFast(float tau_s, float dt_s) {
    if (tau_s <= 0.0f || dt_s <= 0.0f) { setAlpha(1.0f); return; }
    const float a = dt_s / (tau_s + dt_s);
    setAlpha(a);
  }

  // Configure via time constant tau (seconds) and dt (seconds), EXACT (uses expf)
  inline void configureTauDtExact(float tau_s, float dt_s) {
    if (tau_s <= 0.0f || dt_s <= 0.0f) { setAlpha(1.0f); return; }
    const float a = 1.0f - std::exp(-dt_s / tau_s);
    setAlpha(a);
  }

  // Configure via cutoff frequency (Hz) and dt (seconds)
  // tau = 1 / (2*pi*fc)
  inline void configureCutoffHzFast(float cutoff_hz, float dt_s) {
    if (cutoff_hz <= 0.0f) { setAlpha(1.0f); return; }
    const float tau = 1.0f / (2.0f * kPi * cutoff_hz);
    configureTauDtFast(tau, dt_s);
  }

  inline void configureCutoffHzExact(float cutoff_hz, float dt_s) {
    if (cutoff_hz <= 0.0f) { setAlpha(1.0f); return; }
    const float tau = 1.0f / (2.0f * kPi * cutoff_hz);
    configureTauDtExact(tau, dt_s);
  }

  // Update filter with one sample
  // If uninitialized, snaps output to first sample (prevents startup ramp weirdness)
  inline float update(float x) {
    if (!initialized_) {
      y_ = x;
      initialized_ = true;
      return y_;
    }
    y_ = y_ + alpha_ * (x - y_);
    return y_;
  }

  inline float value() const { return y_; }
  inline float alpha() const { return alpha_; }
  inline bool initialized() const { return initialized_; }

private:
  float y_;
  float alpha_;
  bool  initialized_;
};

} // namespace secbot::utils
