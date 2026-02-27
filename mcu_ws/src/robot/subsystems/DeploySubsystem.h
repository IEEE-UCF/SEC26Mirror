/**
 * @file DeploySubsystem.h
 * @brief Button-triggered deployment with LED status feedback.
 * @date 2026-02-26
 *
 * Monitors button 4 (TCA9555 port 1, bit 3) for 1-second hold to trigger
 * deployment.  DIP switch 8 (TCA9555 port 0, bit 7) selects the target
 * firmware: ON = "robot", OFF = "teensy-test-all-subsystems".
 * DIP switch 7 (bit 6) enables force reflash (bypasses firmware hash check).
 *
 * State machine: IDLE -> HOLD_DETECT -> DEPLOYING -> CANCEL_HOLD -> IDLE
 *
 * LED indicator scheme (5 LEDs):
 *   LED 0 = Git pull, LED 1 = MCU flash, LED 2 = ESP32 OTA,
 *   LED 3 = Docker restart, LED 4 = Colcon build
 *   Blue = active, Green = done, Red = failed, Yellow = cancelled
 *
 * -- ROS2 interface (defaults) ------------------------------------------------
 *   /mcu_robot/deploy/trigger   publisher     std_msgs/msg/String
 *   /mcu_robot/deploy/status    subscription  std_msgs/msg/String
 */

#pragma once

#include <BaseSubsystem.h>
#include <microros_manager_robot.h>
#include <std_msgs/msg/string.h>

#include "robot/subsystems/ButtonSubsystem.h"
#include "robot/subsystems/DipSwitchSubsystem.h"
#include "robot/subsystems/LEDSubsystem.h"
#include "robot/subsystems/OLEDSubsystem.h"

#ifdef USE_TEENSYTHREADS
#include <TeensyThreads.h>
#endif

namespace Subsystem {

class DeploySubsystemSetup : public Classes::BaseSetup {
 public:
  DeploySubsystemSetup(const char* _id, ButtonSubsystem* btn,
                       DipSwitchSubsystem* dip, LEDSubsystem* led,
                       OLEDSubsystem* oled = nullptr,
                       const char* trigger_topic = "/mcu_robot/deploy/trigger",
                       const char* status_topic = "/mcu_robot/deploy/status")
      : Classes::BaseSetup(_id),
        btn_(btn),
        dip_(dip),
        led_(led),
        oled_(oled),
        trigger_topic_(trigger_topic),
        status_topic_(status_topic) {}
  ButtonSubsystem* btn_;
  DipSwitchSubsystem* dip_;
  LEDSubsystem* led_;
  OLEDSubsystem* oled_;
  const char* trigger_topic_;
  const char* status_topic_;
};

class DeploySubsystem : public IMicroRosParticipant,
                        public Classes::BaseSubsystem {
 public:
  enum class State { IDLE, HOLD_DETECT, DEPLOYING, CANCEL_HOLD };

  enum class Phase {
    NONE = -1,
    GIT_PULL = 0,
    MCU_FLASH = 1,
    ESP32_OTA = 2,
    DOCKER = 3,
    COLCON_BUILD = 4,
    DONE = 5,
    FAILED = 6,
    CANCELLED = 7
  };

  static constexpr uint8_t BUTTON_IDX = 3;          // Button 4 (0-indexed)
  static constexpr uint8_t DIP_TARGET_IDX = 7;      // DIP 8: target select
  static constexpr uint8_t DIP_FORCE_IDX = 6;       // DIP 7: force reflash
  static constexpr uint32_t HOLD_MS = 1000;
  static constexpr uint8_t NUM_PHASE_LEDS = 5;

  explicit DeploySubsystem(const DeploySubsystemSetup& setup)
      : Classes::BaseSubsystem(setup), setup_(setup) {}

  bool init() override { return setup_.btn_ && setup_.dip_ && setup_.led_; }
  void begin() override {}

  void update() override {
    uint32_t now = millis();
    bool btn_pressed = setup_.btn_->isPressed(BUTTON_IDX);

    switch (state_) {
      case State::IDLE:
        if (btn_pressed && !prev_btn_) {
          hold_start_ms_ = now;
          state_ = State::HOLD_DETECT;
          showDeployConfig();
          oledPrint("Hold 1s to deploy...");
        }
        break;

      case State::HOLD_DETECT:
        if (!btn_pressed) {
          state_ = State::IDLE;
          oledPrint("Deploy: released");
        } else if (now - hold_start_ms_ >= HOLD_MS) {
          publishTrigger(false);
          state_ = State::DEPLOYING;
          deploying_ = true;
          current_phase_ = Phase::NONE;
          setAllLeds(0, 0, 32);  // All blue = starting
          oledPrint("Deploy: TRIGGERED");
        }
        break;

      case State::DEPLOYING:
        if (btn_pressed && !prev_btn_) {
          hold_start_ms_ = now;
          state_ = State::CANCEL_HOLD;
        }
        break;

      case State::CANCEL_HOLD:
        if (!btn_pressed) {
          state_ = State::DEPLOYING;
        } else if (now - hold_start_ms_ >= HOLD_MS) {
          publishTrigger(true);
          setAllLeds(32, 32, 0);  // Yellow = cancelled
          deploying_ = false;
          state_ = State::IDLE;
          oledPrint("Deploy: CANCELLED");
        }
        break;
    }
    prev_btn_ = btn_pressed;
  }

  void pause() override {}
  void reset() override {
    state_ = State::IDLE;
    deploying_ = false;
  }
  const char* getInfo() override {
    static const char info[] = "DeploySubsystem";
    return info;
  }

  bool onCreate(rcl_node_t* node, rclc_executor_t* executor) override {
    node_ = node;

    // Pre-allocate status message buffer
    status_msg_.data.data = status_buf_;
    status_msg_.data.capacity = sizeof(status_buf_);
    status_msg_.data.size = 0;

    if (rclc_publisher_init_best_effort(
            &pub_, node_,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
            setup_.trigger_topic_) != RCL_RET_OK) {
      return false;
    }

    if (rclc_subscription_init_best_effort(
            &sub_, node_,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
            setup_.status_topic_) != RCL_RET_OK) {
      return false;
    }

    if (rclc_executor_add_subscription_with_context(
            executor, &sub_, &status_msg_, &DeploySubsystem::statusCallback,
            this, ON_NEW_DATA) != RCL_RET_OK) {
      return false;
    }

    return true;
  }

  void onDestroy() override {
    pub_ = rcl_get_zero_initialized_publisher();
    sub_ = rcl_get_zero_initialized_subscription();
    node_ = nullptr;
  }

  State getState() const { return state_; }

#ifdef USE_TEENSYTHREADS
  void beginThreaded(uint32_t stackSize, int /*priority*/ = 1,
                     uint32_t updateRateMs = 20) {
    task_delay_ms_ = updateRateMs;
    threads.addThread(taskFunction, this, stackSize);
  }

 private:
  static void taskFunction(void* pv) {
    auto* self = static_cast<DeploySubsystem*>(pv);
    self->begin();
    while (true) {
      self->update();
      threads.delay(self->task_delay_ms_);
    }
  }
  uint32_t task_delay_ms_ = 20;
#endif

 private:
  void oledPrint(const char* text) {
    if (setup_.oled_) {
      setup_.oled_->appendText(text);
    }
  }

  bool readDipTarget() {
    return (setup_.dip_->getState() >> DIP_TARGET_IDX) & 0x01;
  }

  bool readDipForce() {
    return (setup_.dip_->getState() >> DIP_FORCE_IDX) & 0x01;
  }

  void showDeployConfig() {
    bool dip_target = readDipTarget();
    bool dip_force = readDipForce();
    const char* target_str =
        dip_target ? "robot" : "teensy-test-all";

    char line[48];
    snprintf(line, sizeof(line), "Target: %s", target_str);
    oledPrint(line);
    snprintf(line, sizeof(line), "Force:  %s",
             dip_force ? "YES" : "no");
    oledPrint(line);
  }

  void publishTrigger(bool cancel) {
    if (!pub_.impl) return;

    char buf[64];
    if (cancel) {
      snprintf(buf, sizeof(buf), "cancel");
    } else {
      bool dip_target = readDipTarget();
      bool dip_force = readDipForce();
      const char* target =
          dip_target ? "start:robot" : "start:teensy-test-all-subsystems";
      if (dip_force) {
        snprintf(buf, sizeof(buf), "%s:force", target);
      } else {
        snprintf(buf, sizeof(buf), "%s", target);
      }
    }

    trigger_msg_.data.data = buf;
    trigger_msg_.data.size = strlen(buf);
    trigger_msg_.data.capacity = sizeof(buf);

#ifdef USE_TEENSYTHREADS
    {
      Threads::Scope guard(g_microros_mutex);
      (void)rcl_publish(&pub_, &trigger_msg_, NULL);
    }
#else
    (void)rcl_publish(&pub_, &trigger_msg_, NULL);
#endif
  }

  static void statusCallback(const void* msg, void* ctx) {
    auto* self = static_cast<DeploySubsystem*>(ctx);
    auto* str = static_cast<const std_msgs__msg__String*>(msg);

    if (!str->data.data) return;

    // Ignore status updates when not actively deploying
    if (!self->deploying_) return;

    const char* data = str->data.data;

    if (strncmp(data, "phase:", 6) == 0) {
      const char* phase_str = data + 6;
      self->handlePhase(phase_str);
    } else if (strncmp(data, "msg:", 4) == 0) {
      const char* message = data + 4;
      self->handleMessage(message);
    }
  }

  void handlePhase(const char* phase_str) {
    Phase new_phase = Phase::NONE;
    if (strcmp(phase_str, "git_pull") == 0)
      new_phase = Phase::GIT_PULL;
    else if (strcmp(phase_str, "mcu_flash") == 0)
      new_phase = Phase::MCU_FLASH;
    else if (strcmp(phase_str, "esp32_ota") == 0)
      new_phase = Phase::ESP32_OTA;
    else if (strcmp(phase_str, "docker") == 0)
      new_phase = Phase::DOCKER;
    else if (strcmp(phase_str, "colcon_build") == 0)
      new_phase = Phase::COLCON_BUILD;
    else if (strcmp(phase_str, "done") == 0)
      new_phase = Phase::DONE;
    else if (strcmp(phase_str, "failed") == 0)
      new_phase = Phase::FAILED;
    else if (strcmp(phase_str, "cancelled") == 0)
      new_phase = Phase::CANCELLED;

    // Debug output to OLED
    char dbg[32];
    snprintf(dbg, sizeof(dbg), "Phase: %s", phase_str);
    oledPrint(dbg);

    // Reset sub-phase tracking when entering a new phase
    if (new_phase != current_phase_) {
      ota_ok_ = 0;
      ota_skip_ = 0;
      ota_fail_ = 0;
    }

    if (new_phase == Phase::DONE) {
      for (uint8_t i = 0; i < NUM_PHASE_LEDS; i++) {
        setup_.led_->setPixel(i, 0, 32, 0);
      }
      deploying_ = false;
      state_ = State::IDLE;
    } else if (new_phase == Phase::FAILED) {
      int idx = static_cast<int>(current_phase_);
      if (idx >= 0 && idx < NUM_PHASE_LEDS) {
        setup_.led_->setPixel(idx, 32, 0, 0);
      }
      deploying_ = false;
      state_ = State::IDLE;
    } else if (new_phase == Phase::CANCELLED) {
      setAllLeds(32, 32, 0);
      deploying_ = false;
      state_ = State::IDLE;
    } else if (new_phase != Phase::NONE) {
      int new_idx = static_cast<int>(new_phase);
      for (uint8_t i = 0; i < NUM_PHASE_LEDS; i++) {
        if (i < new_idx)
          setup_.led_->setPixel(i, 0, 32, 0);   // Green = done
        else if (i == new_idx)
          setup_.led_->setPixel(i, 0, 0, 32);   // Blue = active
        else
          setup_.led_->setPixel(i, 0, 0, 0);    // Off = pending
      }
      current_phase_ = new_phase;
    }
  }

  /**
   * Handle sub-phase message updates for per-device LED feedback.
   *
   * During esp32_ota, messages follow the pattern "device: STATUS" where
   * STATUS is OK, FAIL, or SKIP (with optional reason). The active phase
   * LED color reflects the aggregate result:
   *   blue = in progress, green = all OK, yellow = some skipped, red = any failed
   */
  void handleMessage(const char* message) {
    int phase_idx = static_cast<int>(current_phase_);
    if (phase_idx < 0 || phase_idx >= NUM_PHASE_LEDS) return;

    // Parse "device: STATUS" pattern for per-device result tracking
    const char* colon = strchr(message, ':');
    if (!colon) return;

    // Extract status after ": "
    const char* status = colon + 1;
    while (*status == ' ') status++;

    if (strncmp(status, "OK", 2) == 0) {
      ota_ok_++;
    } else if (strncmp(status, "FAIL", 4) == 0) {
      ota_fail_++;
    } else if (strncmp(status, "SKIP", 4) == 0) {
      ota_skip_++;
    } else if (strncmp(status, "Flashing", 8) == 0 ||
               strncmp(status, "Starting", 8) == 0 ||
               strncmp(status, "Building", 8) == 0) {
      // In-progress messages — keep LED blue (already set by handlePhase)
      return;
    } else {
      return;
    }

    // Update LED color based on aggregate results
    if (ota_fail_ > 0) {
      setup_.led_->setPixel(phase_idx, 32, 0, 0);    // Red = any failed
    } else if (ota_skip_ > 0 && ota_ok_ > 0) {
      setup_.led_->setPixel(phase_idx, 32, 24, 0);   // Yellow = mixed
    } else if (ota_ok_ > 0) {
      setup_.led_->setPixel(phase_idx, 0, 32, 0);    // Green = all OK so far
    } else if (ota_skip_ > 0) {
      setup_.led_->setPixel(phase_idx, 32, 24, 0);   // Yellow = all skipped
    }
  }

  void setAllLeds(uint8_t r, uint8_t g, uint8_t b) {
    for (uint8_t i = 0; i < NUM_PHASE_LEDS; i++) {
      setup_.led_->setPixel(i, r, g, b);
    }
  }

  const DeploySubsystemSetup setup_;
  State state_ = State::IDLE;
  Phase current_phase_ = Phase::NONE;
  uint32_t hold_start_ms_ = 0;
  bool prev_btn_ = false;
  bool deploying_ = false;

  // Sub-phase device result counters (reset on phase change)
  uint8_t ota_ok_ = 0;
  uint8_t ota_skip_ = 0;
  uint8_t ota_fail_ = 0;

  rcl_publisher_t pub_{};
  rcl_subscription_t sub_{};
  std_msgs__msg__String trigger_msg_{};
  std_msgs__msg__String status_msg_{};
  char status_buf_[128] = {};
  rcl_node_t* node_ = nullptr;
};

}  // namespace Subsystem
