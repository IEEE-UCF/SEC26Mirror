#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H
#include <cmath>
#include <cstdint>
/**
 * @file pid_controller.h
 * @author Rafeed Khan
 * @brief Generic PID controller for MCU-based control loops.
 * No dynamic allocation, deterministic O(1) update time.
 */

class PIDController {
 public:
  struct Gains {
    float kp = 0.0f;
    float ki = 0.0f;
    float kd = 0.0f;
  };

  struct Limits {
    // Output clamp (command saturation)
    float out_min = -1e9f;
    float out_max = 1e9f;

    // Integral clamp (anti windup)
    float i_min = -1e9f;
    float i_max = 1e9f;
  };

  enum class DerivativeMode : uint8_t {
    OnError,       // d(error)/dt (can kick when setpoint steps)
    OnMeasurement  // -d(measurement)/dt (this is more stable for setpoint
                   // steps)
  };

  struct Config {
    Gains gains{};
    Limits limits{};
    DerivativeMode dmode = DerivativeMode::OnMeasurement;

    // If true, only integrate when output is not saturated, or when integration
    // would help unsaturate
    bool conditional_integration = true;

    // Optional low-pass filter for derivative term (0.0-0.9, higher = more
    // smoothing)
    float d_filter_alpha = 0.0f;

    // Set to true if we want to treat dt that is too small as invalid
    float min_dt = 1e-6f;
    float max_dt = 1.0f;  // sanity guard for stalls, i can set it larger later
                          // if we want to run slowly
  };

  PIDController() = default;
  explicit PIDController(const Config& cfg) { configure(cfg); }

  void configure(const Config& cfg) {
    cfg_ = cfg;
    // Ensuring sane limit ordering
    if (cfg_.limits.out_min > cfg_.limits.out_max) {
      const float tmp = cfg_.limits.out_min;
      cfg_.limits.out_min = cfg_.limits.out_max;
      cfg_.limits.out_max = tmp;
    }
    if (cfg_.limits.i_min > cfg_.limits.i_max) {
      const float tmp = cfg_.limits.i_min;
      cfg_.limits.i_min = cfg_.limits.i_max;
      cfg_.limits.i_max = tmp;
    }
    reset();
  }

  const Config& config() const { return cfg_; }

  void setGains(float kp, float ki, float kd) {
    cfg_.gains.kp = kp;
    cfg_.gains.ki = ki;
    cfg_.gains.kd = kd;
  }

  void setOutputLimits(float out_min, float out_max) {
    cfg_.limits.out_min = out_min;
    cfg_.limits.out_max = out_max;
    if (cfg_.limits.out_min > cfg_.limits.out_max) {
      const float tmp = cfg_.limits.out_min;
      cfg_.limits.out_min = cfg_.limits.out_max;
      cfg_.limits.out_max = tmp;
    }
  }

  void setIntegralLimits(float i_min, float i_max) {
    cfg_.limits.i_min = i_min;
    cfg_.limits.i_max = i_max;
    if (cfg_.limits.i_min > cfg_.limits.i_max) {
      const float tmp = cfg_.limits.i_min;
      cfg_.limits.i_min = cfg_.limits.i_max;
      cfg_.limits.i_max = tmp;
    }
    // Clamping current integrator into new bounds
    i_term_ = clamp(i_term_, cfg_.limits.i_min, cfg_.limits.i_max);
  }

  void setDerivativeMode(DerivativeMode m) { cfg_.dmode = m; }
  void setDerivativeFilterAlpha(float alpha) {
    cfg_.d_filter_alpha = clamp(alpha, 0.0f, 0.999999f);
  }

  void reset() {
    initialized_ = false;
    prev_error_ = 0.0f;
    prev_meas_ = 0.0f;
    i_term_ = 0.0f;
    d_term_ = 0.0f;
    last_out_ = 0.0f;
  }

  // Main update
  // setpoint: desired value
  // measurement: current value
  // dt: time since last call, seconds
  // feedforward: optional term added directly to output (default 0)
  float update(float setpoint, float measurement, float dt,
               float feedforward = 0.0f) {
    // Rejecting bad dt
    if (!(dt >= cfg_.min_dt) || !(dt <= cfg_.max_dt) || !isFinite(dt)) {
      // Do nothing if dt is invalid
      return last_out_;
    }

    const float error = setpoint - measurement;

    // Initialize historical values on first valid call to avoid spikes
    if (!initialized_) {
      initialized_ = true;
      prev_error_ = error;
      prev_meas_ = measurement;
      d_term_ = 0.0f;
      // im keeping i_term_ intentionally as is (0 by reset, or we can pre load)
    }

    // Proportional
    const float p = cfg_.gains.kp * error;

    // Derivative
    float deriv = 0.0f;
    if (cfg_.gains.kd != 0.0f) {
      if (cfg_.dmode == DerivativeMode::OnError) {
        deriv = (error - prev_error_) / dt;
      } else {
        // Derivative on measurement avoids kick when setpoint steps
        deriv = -(measurement - prev_meas_) / dt;
      }
    }
    float d_unfiltered = cfg_.gains.kd * deriv;

    // Optional derivative low-pass filter
    if (cfg_.d_filter_alpha > 0.0f) {
      // d_term_ = alpha * d_term_ + (1-alpha) * d_unfiltered
      d_term_ = cfg_.d_filter_alpha * d_term_ +
                (1.0f - cfg_.d_filter_alpha) * d_unfiltered;
    } else {
      d_term_ = d_unfiltered;
    }

    // Candidate integral update
    float i_next = i_term_ + (cfg_.gains.ki * error * dt);

    // Clamp integrator
    i_next = clamp(i_next, cfg_.limits.i_min, cfg_.limits.i_max);

    // Compute unclamped output with candidate integrator
    float out_unclamped = p + i_next + d_term_ + feedforward;

    // Clamp output
    float out = clamp(out_unclamped, cfg_.limits.out_min, cfg_.limits.out_max);

    // Anti windup: conditional integration
    if (cfg_.conditional_integration) {
      // only updating if it reduces saturation
      const bool saturated = (out != out_unclamped);
      if (saturated) {
        // skipping integration here cuz positive error would overshoot the max
        if ((out >= cfg_.limits.out_max) && (error > 0.0f)) {
          // keeping old i_term_
        }
        // not integrating here because a negative error would push us further
        // below the minimum
        else if ((out <= cfg_.limits.out_min) && (error < 0.0f)) {
          // keep old i_term_
        } else {
          i_term_ = i_next;
        }
      } else {
        i_term_ = i_next;
      }
    } else {
      i_term_ = i_next;
    }

    // Now we update the history!
    prev_error_ = error;
    prev_meas_ = measurement;
    last_out_ = out;

    return out;
  }

  // Debug slop...
  float lastOutput() const { return last_out_; }
  float integralTerm() const { return i_term_; }
  float derivativeTerm() const { return d_term_; }
  bool initialized() const { return initialized_; }

 private:
  static float clamp(float x, float lo, float hi) {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
  }

  static bool isFinite(float x) {
    return (x == x) && (x > -3.4e38f) && (x < 3.4e38f);  // filters NaN/Inf
  }

  Config cfg_{};

  bool initialized_ = false;

  float prev_error_ = 0.0f;
  float prev_meas_ = 0.0f;

  float i_term_ = 0.0f;
  float d_term_ = 0.0f;

  float last_out_ = 0.0f;
};

#endif  // PID_CONTROLLER_H
