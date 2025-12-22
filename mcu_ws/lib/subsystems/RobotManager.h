/**
 * @file RobotManager.h
 * @date 12/16/25
 * @author Aldem Pido
 * @brief Main logic for the SEC26 robot.
 */

#include <Arduino.h>
#include <BaseDriver.h>
#include <BaseSubsystem.h>
#include <elapsedMillis.h>

#include <vector>

// ==========================================================
// ## 1. Instantiate All Driver and Manager Objects ##
// ==========================================================

#define DELAY_MS_1000 1000000
#define DELAY_MS_200 200000
#define DELAY_MS_50 50000
#define DELAY_MS_20 20000
#define DELAY_MS_5 5000
#define DELAY_MS_2 2000
#define DELAY_NS_500 500
#define DELAY_NS_200 200

namespace Subsystem {
// Forward declaration within namespace to keep header lightweight
class MCUSubsystem;

enum TimerConfig {
  MS_1000,  // 1 Hz
  MS_200,   // 5 Hz
  MS_50,    // 20 Hz
  MS_20,    // 50 Hz
  MS_5,     // 200 Hz
  MS_2,     // 500 Hz
  NS_500,   // 2000 Hz
  NS_200,   // 5000 Hz
  REALTIME  // Unlimited (Caution!)
};

class RobotObject {
 public:
  ~RobotObject() = default;
  RobotObject() = delete;
  // Generic base (init-only)
  RobotObject(const Classes::BaseClass& base, TimerConfig timer_config)
      : base_(base), timer_config_(timer_config), update_fn_(nullptr), reset_fn_(nullptr) {}

  // Driver (has update)
  RobotObject(Classes::BaseDriver& driver, TimerConfig timer_config)
      : base_(driver), timer_config_(timer_config) {
    update_fn_ = [](Classes::BaseClass* b) {
      static_cast<Classes::BaseDriver*>(b)->update();
    };
    reset_fn_ = nullptr;
  }

  // Subsystem (has update + reset)
  RobotObject(Classes::BaseSubsystem& subsystem, TimerConfig timer_config)
      : base_(subsystem), timer_config_(timer_config) {
    update_fn_ = [](Classes::BaseClass* b) {
      static_cast<Classes::BaseSubsystem*>(b)->update();
    };
    reset_fn_ = [](Classes::BaseClass* b) {
      static_cast<Classes::BaseSubsystem*>(b)->reset();
    };
  }
  const Classes::BaseClass& base_;
  TimerConfig timer_config_;
  void (*update_fn_)(Classes::BaseClass*) = nullptr;
  void (*reset_fn_)(Classes::BaseClass*) = nullptr;
};

class RobotManagerSetup : public Classes::BaseSetup {
 public:
  RobotManagerSetup(const char* _id, const std::vector<RobotObject*>& objects)
      : Classes::BaseSetup(_id), objects_(objects) {}
  const std::vector<RobotObject*> objects_;
};

class RobotManagerTimekeeper {
 public:
  RobotManagerTimekeeper(const TimerConfig& config, const unsigned long delay)
      : config_(config), delay_(delay), time_(0) {}
  bool checkTimer() {
    if (config_ == REALTIME) {
      return true;
    } else {
      if (time_ > delay_) {
        time_ = 0;
        return true;
      }
    }
    return false;
  }
  TimerConfig getConfig() const { return config_; }

 private:
  TimerConfig config_;
  const unsigned long delay_;
  elapsedMicros time_;
};

class RobotManager : public Classes::BaseSubsystem {
 public:
  ~RobotManager() override = default;
  RobotManager() = delete;
  explicit RobotManager(const RobotManagerSetup& setup)
      : Classes::BaseSubsystem(setup), setup_(setup) {}

  bool init() override;
  void begin() override {}
  void update() override;
  void pause() override {}
  void reset() override;
  const char* getInfo() override { static const char info[] = "RobotManager"; return info; }

 private:
  const RobotManagerSetup& setup_;
  std::vector<RobotManagerTimekeeper> timekeepers_;
};
};  // namespace Subsystem
// --- Drivers ---

// --- Subsystems ---
