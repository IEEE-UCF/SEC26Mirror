/**
 * @file ServoSubsystem.h
 * @brief Manages servos on a PCA9685 with micro-ROS service control.
 * @date 2026-02-24
 *
 * Each servo occupies one PCA9685 channel.  Angles are 0-180 degrees,
 * mapped to standard servo pulse widths (~500-2500 us).
 *
 * -- ROS2 interface (defaults) ---------------------------------------------
 *   /mcu_robot/servo/state     topic    std_msgs/msg/Float32MultiArray (5 Hz)
 *   /mcu_robot/servo/set       service  mcu_msgs/srv/SetServo
 */

#pragma once

#include <BaseSubsystem.h>
#include <PCA9685Driver.h>
#include <mcu_msgs/srv/set_servo.h>
#include <microros_manager_robot.h>
#include <std_msgs/msg/float32_multi_array.h>

#ifdef USE_TEENSYTHREADS
#include <TeensyThreads.h>
#endif

namespace Subsystem {

class ServoSubsystemSetup : public Classes::BaseSetup {
 public:
  /**
   * @param _id          Subsystem identifier.
   * @param driver       PCA9685 driver for the servo board.
   * @param oePin        Output-Enable GPIO pin (active LOW), 255 = unused.
   * @param numServos    Number of servos (max 8).
   * @param stateTopic   ROS2 topic for publishing current angles.
   * @param serviceName  ROS2 service for setting a servo angle.
   */
  ServoSubsystemSetup(const char* _id, Robot::PCA9685Driver* driver,
                      uint8_t oePin = 255, uint8_t numServos = 8,
                      const char* stateTopic = "/mcu_robot/servo/state",
                      const char* serviceName = "/mcu_robot/servo/set")
      : Classes::BaseSetup(_id),
        driver_(driver),
        oePin_(oePin),
        numServos_(numServos),
        stateTopic_(stateTopic),
        serviceName_(serviceName) {}

  Robot::PCA9685Driver* driver_ = nullptr;
  uint8_t oePin_ = 255;
  uint8_t numServos_ = 8;
  const char* stateTopic_ = "/mcu_robot/servo/state";
  const char* serviceName_ = "/mcu_robot/servo/set";
};

class ServoSubsystem : public IMicroRosParticipant,
                       public Classes::BaseSubsystem {
 public:
  static constexpr uint8_t MAX_SERVOS = 8;
  static constexpr uint16_t PWM_MIN = 102;  // ~500 us at 50 Hz
  static constexpr uint16_t PWM_MAX = 512;  // ~2500 us at 50 Hz

  explicit ServoSubsystem(const ServoSubsystemSetup& setup)
      : Classes::BaseSubsystem(setup), setup_(setup) {
    for (uint8_t i = 0; i < MAX_SERVOS; i++) angles_[i] = 90.0f;
  }

  bool init() override {
    if (setup_.oePin_ != 255) {
      pinMode(setup_.oePin_, OUTPUT);
      digitalWrite(setup_.oePin_, LOW);
    }
    return true;
  }

  void begin() override {}

  void update() override {
    if (!pub_.impl) return;
    uint32_t now = millis();
    if (now - last_publish_ms_ >= PUBLISH_INTERVAL_MS) {
      last_publish_ms_ = now;
      publishState();
    }
  }

  void pause() override {
    if (setup_.oePin_ != 255) digitalWrite(setup_.oePin_, HIGH);
  }

  void reset() override { pause(); }

  const char* getInfo() override {
    static const char info[] = "ServoSubsystem";
    return info;
  }

  bool onCreate(rcl_node_t* node, rclc_executor_t* executor) override {
    node_ = node;

    pub_msg_.data.data = pub_data_;
    pub_msg_.data.size = setup_.numServos_;
    pub_msg_.data.capacity = MAX_SERVOS;

    if (rclc_publisher_init_best_effort(
            &pub_, node_,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
            setup_.stateTopic_) != RCL_RET_OK) {
      return false;
    }

    if (rclc_service_init_default(
            &srv_, node_, ROSIDL_GET_SRV_TYPE_SUPPORT(mcu_msgs, srv, SetServo),
            setup_.serviceName_) != RCL_RET_OK) {
      return false;
    }

    if (rclc_executor_add_service_with_context(
            executor, &srv_, &srv_req_, &srv_res_, &ServoSubsystem::srvCallback,
            this) != RCL_RET_OK) {
      return false;
    }

    return true;
  }

  void onDestroy() override {
    pub_ = rcl_get_zero_initialized_publisher();
    srv_ = rcl_get_zero_initialized_service();
    node_ = nullptr;
  }

  void setAngle(uint8_t servo, float degrees) {
    if (servo >= setup_.numServos_ || !setup_.driver_) return;
    degrees = constrain(degrees, 0.0f, 180.0f);
    angles_[servo] = degrees;
    uint16_t pwm =
        PWM_MIN + (uint16_t)((degrees / 180.0f) * (PWM_MAX - PWM_MIN));
    setup_.driver_->bufferPWM(servo, pwm);
  }

  float getAngle(uint8_t servo) const {
    return servo < setup_.numServos_ ? angles_[servo] : 0.0f;
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
    auto* self = static_cast<ServoSubsystem*>(pv);
    self->begin();
    while (true) {
      self->update();
      threads.delay(self->task_delay_ms_);
    }
  }
  uint32_t task_delay_ms_ = 50;
#endif

 private:
  void publishState() {
    for (uint8_t i = 0; i < setup_.numServos_; i++) pub_data_[i] = angles_[i];
    pub_msg_.data.size = setup_.numServos_;
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
    auto* self = static_cast<ServoSubsystem*>(ctx);
    auto* r = static_cast<const mcu_msgs__srv__SetServo_Request*>(req);
    auto* rsp = static_cast<mcu_msgs__srv__SetServo_Response*>(res);
    if (r->index < self->setup_.numServos_) {
      self->setAngle(r->index, r->angle);
      rsp->success = true;
    } else {
      rsp->success = false;
    }
  }

  static constexpr uint32_t PUBLISH_INTERVAL_MS = 200;

  const ServoSubsystemSetup setup_;
  float angles_[MAX_SERVOS] = {};

  rcl_publisher_t pub_{};
  std_msgs__msg__Float32MultiArray pub_msg_{};
  float pub_data_[MAX_SERVOS] = {};

  rcl_service_t srv_{};
  mcu_msgs__srv__SetServo_Request srv_req_{};
  mcu_msgs__srv__SetServo_Response srv_res_{};

  rcl_node_t* node_ = nullptr;
  uint32_t last_publish_ms_ = 0;
};

}  // namespace Subsystem
