/**
 * @file EncoderSubsystem.h
 * @brief Motor FG encoder subsystem using QTimer hardware pulse counting.
 * @date 2026-02-27
 *
 * Reads 8 motor FG (frequency generator) speed signals using IMXRT1062
 * QTimer hardware counters routed via XBAR crossbar.  Publishes signed
 * tick rates (ticks/sec) to micro-ROS.  Completely decoupled from the
 * drive base — provides raw encoder data for any consumer.
 *
 * Direction is read from MotorManagerSubsystem's intended direction
 * (immune to NFPShop reverse pulses that don't represent real movement).
 *
 * -- ROS2 interface (defaults) ---------------------------------------------
 *   /mcu_robot/encoders   topic   std_msgs/msg/Float32MultiArray (8 floats)
 *     [0..7] = signed ticks/sec for motors 1-8 (pins 2-9)
 */

#pragma once

#include <BaseSubsystem.h>
#include <QTimerEncoder.h>
#include <microros_manager_robot.h>
#include <std_msgs/msg/float32_multi_array.h>

#ifdef USE_TEENSYTHREADS
#include <TeensyThreads.h>
#endif

#include "robot/subsystems/MotorManagerSubsystem.h"

namespace Subsystem {

class EncoderSubsystemSetup : public Classes::BaseSetup {
 public:
  /**
   * @param _id           Subsystem identifier.
   * @param encoder       QTimerEncoder hardware driver instance.
   * @param motorManager  MotorManagerSubsystem for reading intended directions.
   * @param motorMap      Array mapping encoder channel -> motor manager index.
   *                      nullptr = identity mapping (channel N -> motor N).
   * @param topic         ROS2 topic name for encoder data.
   */
  EncoderSubsystemSetup(const char* _id, Encoders::QTimerEncoder* encoder,
                         MotorManagerSubsystem* motorManager,
                         const uint8_t* motorMap = nullptr,
                         const char* topic = "/mcu_robot/encoders")
      : Classes::BaseSetup(_id),
        encoder_(encoder),
        motorManager_(motorManager),
        topic_(topic) {
    if (motorMap) {
      for (uint8_t i = 0; i < Encoders::NUM_ENCODER_CHANNELS; i++)
        motorMap_[i] = motorMap[i];
    } else {
      for (uint8_t i = 0; i < Encoders::NUM_ENCODER_CHANNELS; i++)
        motorMap_[i] = i;
    }
  }

  Encoders::QTimerEncoder* encoder_ = nullptr;
  MotorManagerSubsystem* motorManager_ = nullptr;
  const char* topic_ = "/mcu_robot/encoders";
  uint8_t motorMap_[Encoders::NUM_ENCODER_CHANNELS] = {};
};

class EncoderSubsystem : public IMicroRosParticipant,
                          public Classes::BaseSubsystem {
 public:
  static constexpr uint8_t NUM_CHANNELS = Encoders::NUM_ENCODER_CHANNELS;

  explicit EncoderSubsystem(const EncoderSubsystemSetup& setup)
      : Classes::BaseSubsystem(setup), setup_(setup) {}

  bool init() override {
    if (!setup_.encoder_) return false;
    setup_.encoder_->init();
    last_capture_us_ = micros();
    return true;
  }

  void begin() override {}

  void update() override {
    if (!setup_.encoder_ || !setup_.motorManager_) return;

    // Read intended motor directions (ignores NFPShop reverse pulses)
    bool dirs[NUM_CHANNELS];
    for (uint8_t i = 0; i < NUM_CHANNELS; i++) {
      dirs[i] = setup_.motorManager_->getIntendedDirection(setup_.motorMap_[i]);
    }

    // Capture all encoder counters and apply direction signs
    setup_.encoder_->captureAll(dirs);

    // Compute ticks/sec from accumulated ticks and elapsed time
    uint32_t now_us = micros();
    uint32_t dt_us = now_us - last_capture_us_;
    last_capture_us_ = now_us;
    if (dt_us == 0) dt_us = 1;

    float dt_inv = 1000000.0f / (float)dt_us;
    for (uint8_t i = 0; i < NUM_CHANNELS; i++) {
      int32_t ticks = setup_.encoder_->getTicks(i);
      tick_rates_[i] = (float)ticks * dt_inv;
    }

    // Reset accumulators for next interval
    setup_.encoder_->resetAll();

    // Publish
    if (!pub_.impl) return;
    publishData();
  }

  void pause() override {}

  void reset() override {
    if (setup_.encoder_) setup_.encoder_->resetAll();
    for (uint8_t i = 0; i < NUM_CHANNELS; i++) tick_rates_[i] = 0.0f;
  }

  const char* getInfo() override {
    static const char info[] = "EncoderSubsystem";
    return info;
  }

  // ── IMicroRosParticipant ──────────────────────────────────────────────
  bool onCreate(rcl_node_t* node, rclc_executor_t* executor) override {
    (void)executor;  // Publisher only — 0 executor handles consumed
    node_ = node;

    pub_msg_.data.data = pub_data_;
    pub_msg_.data.size = NUM_CHANNELS;
    pub_msg_.data.capacity = NUM_CHANNELS;

    if (rclc_publisher_init_best_effort(
            &pub_, node_,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
            setup_.topic_) != RCL_RET_OK) {
      return false;
    }

    return true;
  }

  void onDestroy() override {
    pub_ = rcl_get_zero_initialized_publisher();
    node_ = nullptr;
  }

  /** @brief Get tick rate for a channel (signed ticks/sec). */
  float getTickRate(uint8_t channel) const {
    return (channel < NUM_CHANNELS) ? tick_rates_[channel] : 0.0f;
  }

#ifdef USE_TEENSYTHREADS
  void beginThreaded(uint32_t stackSize, int /*priority*/ = 1,
                     uint32_t updateRateMs = 20) {
    task_delay_ms_ = updateRateMs;
    threads.addThread(taskFunction, this, stackSize);
  }

 private:
  static void taskFunction(void* pv) {
    auto* self = static_cast<EncoderSubsystem*>(pv);
    self->begin();
    while (true) {
      self->update();
      threads.delay(self->task_delay_ms_);
    }
  }
  uint32_t task_delay_ms_ = 20;
#endif

 private:
  void publishData() {
    for (uint8_t i = 0; i < NUM_CHANNELS; i++) pub_data_[i] = tick_rates_[i];
    pub_msg_.data.size = NUM_CHANNELS;
#ifdef USE_TEENSYTHREADS
    {
      Threads::Scope guard(g_microros_mutex);
      (void)rcl_publish(&pub_, &pub_msg_, NULL);
    }
#else
    (void)rcl_publish(&pub_, &pub_msg_, NULL);
#endif
  }

  const EncoderSubsystemSetup setup_;
  float tick_rates_[NUM_CHANNELS] = {};
  uint32_t last_capture_us_ = 0;

  rcl_publisher_t pub_{};
  std_msgs__msg__Float32MultiArray pub_msg_{};
  float pub_data_[NUM_CHANNELS] = {};
  rcl_node_t* node_ = nullptr;
};

}  // namespace Subsystem
