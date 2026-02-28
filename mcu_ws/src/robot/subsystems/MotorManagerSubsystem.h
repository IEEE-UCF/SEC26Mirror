/**
 * @file MotorManagerSubsystem.h
 * @brief Manages brushless motors (NFPShop) on a PCA9685 with micro-ROS.
 * @date 2026-02-26
 *
 * Each motor uses two consecutive PCA9685 channels:
 *   Motors 0-3: even channel = PWM, odd channel = DIR
 *   Motors 4-7: even channel = DIR, odd channel = PWM (swapped)
 * Motor 0 -> ch 0(PWM),1(DIR);  ...  Motor 4 -> ch 8(DIR),9(PWM); ...
 *
 * NFPShop brushless motor quirk: a brief reverse pulse (~3ms) is applied
 * every ~103ms to prevent the integrated controller from entering a fault
 * state.  This is handled transparently in update().
 *
 * The motor PCA9685 should be configured at a higher frequency than the
 * servo board (e.g. 1000 Hz via MOTOR_PCA9685_FREQ in RobotConstants.h).
 *
 * -- ROS2 interface (defaults) ---------------------------------------------
 *   /mcu_robot/motor/state     topic    std_msgs/msg/Float32MultiArray (5 Hz)
 *   /mcu_robot/motor/set       service  mcu_msgs/srv/SetMotor
 */

#pragma once

#include <BaseSubsystem.h>
#include <PCA9685Driver.h>
#include "DebugLog.h"
#include <mcu_msgs/srv/set_motor.h>
#include <microros_manager_robot.h>
#include <std_msgs/msg/float32_multi_array.h>

#ifdef USE_TEENSYTHREADS
#include <TeensyThreads.h>
#endif

namespace Subsystem {

class MotorManagerSubsystemSetup : public Classes::BaseSetup {
 public:
  /**
   * @param _id          Subsystem identifier.
   * @param driver       PCA9685 driver for the motor board.
   * @param oePin        Output-Enable GPIO pin (active LOW), 255 = unused.
   * @param numMotors    Number of motors (max 8, uses 2 channels each).
   * @param stateTopic   ROS2 topic for publishing current speeds.
   * @param serviceName  ROS2 service for setting a motor speed.
   */
  MotorManagerSubsystemSetup(const char* _id, Robot::PCA9685Driver* driver,
                             uint8_t oePin = 255, uint8_t numMotors = 8,
                             const char* stateTopic = "/mcu_robot/motor/state",
                             const char* serviceName = "/mcu_robot/motor/set")
      : Classes::BaseSetup(_id),
        driver_(driver),
        oePin_(oePin),
        numMotors_(numMotors),
        stateTopic_(stateTopic),
        serviceName_(serviceName) {}

  Robot::PCA9685Driver* driver_ = nullptr;
  uint8_t oePin_ = 255;
  uint8_t numMotors_ = 8;
  const char* stateTopic_ = "/mcu_robot/motor/state";
  const char* serviceName_ = "/mcu_robot/motor/set";
};

class MotorManagerSubsystem : public IMicroRosParticipant,
                              public Classes::BaseSubsystem {
 public:
  static constexpr uint8_t MAX_MOTORS = 8;
  static constexpr uint16_t MAX_PWM = 4095;

  // NFPShop reverse-pulse timing (microseconds)
  static constexpr uint32_t NFPSHOP_CYCLE_US = 103000;
  static constexpr uint32_t NFPSHOP_REVERSE_START_US = 100000;
  // PWM duty used during the brief reverse pulse — kept low so the
  // motor inertia absorbs it and it's not visible as a jerk.
  static constexpr uint16_t NFPSHOP_REVERSE_DUTY = 5 * (MAX_PWM / 255);

  explicit MotorManagerSubsystem(const MotorManagerSubsystemSetup& setup)
      : Classes::BaseSubsystem(setup), setup_(setup) {}

  bool init() override {
    if (setup_.oePin_ != 255) {
      pinMode(setup_.oePin_, OUTPUT);
      digitalWrite(setup_.oePin_, LOW);
    }
    // Set all motor PWM channels to stopped (inverted: MAX_PWM = stopped).
    // The PCA9685Driver::init() sets channels to OFF (duty 0) which would
    // mean full speed with inverted logic, so we must override immediately.
    for (uint8_t i = 0; i < setup_.numMotors_; i++) {
      speeds_[i] = 0.0f;
      dirs_[i] = true;
      if (setup_.driver_) {
        setup_.driver_->bufferPWM(pwmChannel(i), MAX_PWM);
        setup_.driver_->bufferDigital(dirChannel(i), true);
      }
    }
    // Flush immediately so motors don't twitch on startup
    if (setup_.driver_) setup_.driver_->applyBuffered();
    DEBUG_PRINTF("[MOTOR] init OK (%d motors, OE=%d)\n", setup_.numMotors_,
                 setup_.oePin_);
    return true;
  }

  void begin() override {}

  void update() override {
    // NFPShop reverse-pulse: briefly flip direction for ~3ms every ~103ms
    uint32_t now_us = micros();
    for (uint8_t i = 0; i < setup_.numMotors_; i++) {
      uint32_t elapsed = now_us - reverse_timer_us_[i];
      if (elapsed >= NFPSHOP_CYCLE_US) {
        // Cycle complete — reset timer, restore normal output
        reverse_timer_us_[i] = now_us;
        applyMotorOutput(i);
      } else if (elapsed >= NFPSHOP_REVERSE_START_US && !in_reverse_[i]) {
        // Enter reverse pulse window
        in_reverse_[i] = true;
        if (setup_.driver_) {
          setup_.driver_->bufferDigital(dirChannel(i), !dirs_[i]);
          setup_.driver_->bufferPWM(pwmChannel(i),
                                    MAX_PWM - NFPSHOP_REVERSE_DUTY);
        }
      }
    }

    if (!pub_.impl) return;
    uint32_t now = millis();
    if (now - last_publish_ms_ >= PUBLISH_INTERVAL_MS) {
      last_publish_ms_ = now;
      publishState();
    }
  }

  void pause() override {
    for (uint8_t i = 0; i < setup_.numMotors_; i++) setSpeed(i, 0.0f);
    if (setup_.oePin_ != 255) digitalWrite(setup_.oePin_, HIGH);
  }

  void reset() override { pause(); }

  const char* getInfo() override {
    static const char info[] = "MotorManagerSubsystem";
    return info;
  }

  bool onCreate(rcl_node_t* node, rclc_executor_t* executor) override {
    node_ = node;

    pub_msg_.data.data = pub_data_;
    pub_msg_.data.size = setup_.numMotors_;
    pub_msg_.data.capacity = MAX_MOTORS;

    if (rclc_publisher_init_best_effort(
            &pub_, node_,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
            setup_.stateTopic_) != RCL_RET_OK) {
      return false;
    }

    if (rclc_service_init_default(
            &srv_, node_, ROSIDL_GET_SRV_TYPE_SUPPORT(mcu_msgs, srv, SetMotor),
            setup_.serviceName_) != RCL_RET_OK) {
      return false;
    }

    if (rclc_executor_add_service_with_context(
            executor, &srv_, &srv_req_, &srv_res_,
            &MotorManagerSubsystem::srvCallback, this) != RCL_RET_OK) {
      DEBUG_PRINTLN("[MOTOR] onCreate FAIL: service executor");
      return false;
    }

    DEBUG_PRINTLN("[MOTOR] onCreate OK");
    return true;
  }

  void onDestroy() override {
    pub_ = rcl_get_zero_initialized_publisher();
    srv_ = rcl_get_zero_initialized_service();
    node_ = nullptr;
  }

  /**
   * @brief Set motor speed.
   * @param motor Motor index (0-7).
   * @param speed Speed from -1.0 (full reverse) to +1.0 (full forward).
   *              PWM is inverted: 0 speed → MAX_PWM duty, full speed → 0 duty.
   */
  void setSpeed(uint8_t motor, float speed) {
    if (motor >= setup_.numMotors_ || !setup_.driver_) return;
    speed = constrain(speed, -1.0f, 1.0f);
    speeds_[motor] = speed;
    dirs_[motor] = (speed >= 0.0f);

    applyMotorOutput(motor);
  }

  float getSpeed(uint8_t motor) const {
    return motor < setup_.numMotors_ ? speeds_[motor] : 0.0f;
  }

  void enable() {
    if (setup_.oePin_ != 255) digitalWrite(setup_.oePin_, LOW);
  }

  void disable() {
    if (setup_.oePin_ != 255) digitalWrite(setup_.oePin_, HIGH);
  }

#ifdef USE_TEENSYTHREADS
  void beginThreaded(uint32_t stackSize, int /*priority*/ = 1,
                     uint32_t updateRateMs = 50) {
    task_delay_ms_ = updateRateMs;
    threads.addThread(taskFunction, this, stackSize);
  }

 private:
  static void taskFunction(void* pv) {
    auto* self = static_cast<MotorManagerSubsystem*>(pv);
    self->begin();
    while (true) {
      self->update();
      threads.delay(self->task_delay_ms_);
    }
  }
  uint32_t task_delay_ms_ = 50;
#endif

 private:
  /** Get the PWM channel for a motor (swapped for motors 4-7). */
  static uint8_t pwmChannel(uint8_t motor) {
    return (motor < 4) ? (motor * 2) : (motor * 2 + 1);
  }

  /** Get the DIR channel for a motor (swapped for motors 4-7). */
  static uint8_t dirChannel(uint8_t motor) {
    return (motor < 4) ? (motor * 2 + 1) : (motor * 2);
  }

  /** Write the normal (non-reverse-pulse) output for a motor. */
  void applyMotorOutput(uint8_t motor) {
    // Inverted PWM: MAX_PWM = stopped, 0 = full speed
    uint16_t duty = MAX_PWM - (uint16_t)(fabsf(speeds_[motor]) * MAX_PWM);

    setup_.driver_->bufferDigital(dirChannel(motor), dirs_[motor]);
    setup_.driver_->bufferPWM(pwmChannel(motor), duty);
    in_reverse_[motor] = false;
  }

  void publishState() {
    for (uint8_t i = 0; i < setup_.numMotors_; i++) pub_data_[i] = speeds_[i];
    pub_msg_.data.size = setup_.numMotors_;
#ifdef USE_TEENSYTHREADS
    {
      Threads::Scope guard(g_microros_mutex);
      (void)rcl_publish(&pub_, &pub_msg_, NULL);
    }
#else
    (void)rcl_publish(&pub_, &pub_msg_, NULL);
#endif
  }

  static void srvCallback(const void* req, void* res, void* ctx) {
    auto* self = static_cast<MotorManagerSubsystem*>(ctx);
    auto* r = static_cast<const mcu_msgs__srv__SetMotor_Request*>(req);
    auto* rsp = static_cast<mcu_msgs__srv__SetMotor_Response*>(res);
    if (r->index < self->setup_.numMotors_) {
      self->setSpeed(r->index, r->speed);
      rsp->success = true;
      DEBUG_PRINTF("[MOTOR] set m%d = %.2f\n", r->index, r->speed);
    } else {
      rsp->success = false;
      DEBUG_PRINTF("[MOTOR] set FAIL: m%d out of range\n", r->index);
    }
  }

  static constexpr uint32_t PUBLISH_INTERVAL_MS = 200;

  const MotorManagerSubsystemSetup setup_;
  float speeds_[MAX_MOTORS] = {};
  bool dirs_[MAX_MOTORS] = {};                  // cached direction per motor
  uint32_t reverse_timer_us_[MAX_MOTORS] = {};  // NFPShop cycle timer
  bool in_reverse_[MAX_MOTORS] = {};  // currently in reverse pulse window

  rcl_publisher_t pub_{};
  std_msgs__msg__Float32MultiArray pub_msg_{};
  float pub_data_[MAX_MOTORS] = {};

  rcl_service_t srv_{};
  mcu_msgs__srv__SetMotor_Request srv_req_{};
  mcu_msgs__srv__SetMotor_Response srv_res_{};

  rcl_node_t* node_ = nullptr;
  uint32_t last_publish_ms_ = 0;
};

}  // namespace Subsystem
