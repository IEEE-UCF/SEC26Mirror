#include "DroneStateSubsystem.h"

#include <rosidl_runtime_c/string_functions.h>
#include <math_utils.h>

using secbot::utils::clamp;
using secbot::utils::deg2rad;

namespace Drone {

DroneStateSubsystem* DroneStateSubsystem::s_instance_ = nullptr;

bool DroneStateSubsystem::onCreate(rcl_node_t* node,
                                   rclc_executor_t* executor) {
  s_instance_ = this;
  if (!data_mutex_) data_mutex_ = xSemaphoreCreateMutex();

  DRONE_PRINTLN("[State] onCreate: state_pub...");
  // State publisher (best-effort)
  if (rclc_publisher_init_best_effort(
          &state_pub_, node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(mcu_msgs, msg, DroneState),
          "/mcu_drone/state") != RCL_RET_OK) {
    DRONE_PRINTLN("[State] FAIL: state_pub");
    return false;
  }

  DRONE_PRINTLN("[State] onCreate: cmd_vel_sub...");
  // cmd_vel subscriber
  if (rclc_subscription_init_default(
          &cmd_vel_sub_, node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
          "/mcu_drone/cmd_vel") != RCL_RET_OK) {
    DRONE_PRINTLN("[State] FAIL: cmd_vel_sub init");
    return false;
  }
  if (rclc_executor_add_subscription(executor, &cmd_vel_sub_, &cmd_vel_msg_,
                                     [](const void* msg) {
                                       if (!s_instance_) return;
                                       auto* twist =
                                           (const geometry_msgs__msg__Twist*)msg;
                                       s_instance_->cmd_pitch_ =
                                           twist->linear.x *
                                           Config::CMD_VEL_PITCH_SCALE;
                                       s_instance_->cmd_roll_ =
                                           twist->linear.y *
                                           Config::CMD_VEL_ROLL_SCALE;
                                       s_instance_->cmd_alt_rate_ =
                                           twist->linear.z *
                                           Config::CMD_VEL_ALT_SCALE;
                                       s_instance_->cmd_yaw_rate_ =
                                           twist->angular.z *
                                           Config::CMD_VEL_YAW_SCALE;
                                       s_instance_->last_cmd_vel_ms_ = millis();
                                     },
                                     ON_NEW_DATA) != RCL_RET_OK) {
    DRONE_PRINTLN("[State] FAIL: cmd_vel_sub executor");
    return false;
  }

  DRONE_PRINTLN("[State] onCreate: arm_srv...");
  // Arm service
  if (rclc_service_init_default(
          &arm_srv_, node,
          ROSIDL_GET_SRV_TYPE_SUPPORT(mcu_msgs, srv, DroneArm),
          "/mcu_drone/arm") != RCL_RET_OK) {
    DRONE_PRINTLN("[State] FAIL: arm_srv init");
    return false;
  }
  if (rclc_executor_add_service(executor, &arm_srv_, &arm_req_, &arm_res_,
                                armCallback) != RCL_RET_OK) {
    DRONE_PRINTLN("[State] FAIL: arm_srv executor");
    return false;
  }

  DRONE_PRINTLN("[State] onCreate: takeoff_srv...");
  // Takeoff service
  {
    rcl_ret_t rc = rclc_service_init_default(
            &takeoff_srv_, node,
            ROSIDL_GET_SRV_TYPE_SUPPORT(mcu_msgs, srv, DroneTakeoff),
            "/mcu_drone/takeoff");
    if (rc != RCL_RET_OK) {
      DRONE_PRINTF("[State] FAIL: takeoff_srv init rc=%d heap=%u\n",
                   (int)rc, ESP.getFreeHeap());
      return false;
    }
  }
  if (rclc_executor_add_service(executor, &takeoff_srv_, &takeoff_req_,
                                &takeoff_res_, takeoffCallback) != RCL_RET_OK) {
    DRONE_PRINTLN("[State] FAIL: takeoff_srv executor");
    return false;
  }

  DRONE_PRINTLN("[State] onCreate: land_srv...");
  // Land service
  if (rclc_service_init_default(
          &land_srv_, node,
          ROSIDL_GET_SRV_TYPE_SUPPORT(mcu_msgs, srv, DroneLand),
          "/mcu_drone/land") != RCL_RET_OK) {
    DRONE_PRINTLN("[State] FAIL: land_srv init");
    return false;
  }
  if (rclc_executor_add_service(executor, &land_srv_, &land_req_, &land_res_,
                                landCallback) != RCL_RET_OK) {
    DRONE_PRINTLN("[State] FAIL: land_srv executor");
    return false;
  }

  DRONE_PRINTLN("[State] onCreate: set_anchors_srv...");
  // SetAnchors service
  if (rclc_service_init_default(
          &set_anchors_srv_, node,
          ROSIDL_GET_SRV_TYPE_SUPPORT(mcu_msgs, srv, DroneSetAnchors),
          "/mcu_drone/set_anchors") != RCL_RET_OK) {
    DRONE_PRINTLN("[State] FAIL: set_anchors_srv init");
    return false;
  }
  if (rclc_executor_add_service(executor, &set_anchors_srv_, &anchors_req_,
                                &anchors_res_, setAnchorsCallback) !=
      RCL_RET_OK) {
    DRONE_PRINTLN("[State] FAIL: set_anchors_srv executor");
    return false;
  }

  DRONE_PRINTLN("[State] onCreate: set_motors_srv...");
  // SetMotors service
  if (rclc_service_init_default(
          &set_motors_srv_, node,
          ROSIDL_GET_SRV_TYPE_SUPPORT(mcu_msgs, srv, DroneSetMotors),
          "/mcu_drone/set_motors") != RCL_RET_OK) {
    DRONE_PRINTLN("[State] FAIL: set_motors_srv init");
    return false;
  }
  if (rclc_executor_add_service(executor, &set_motors_srv_, &motors_req_,
                                &motors_res_, setMotorsCallback) != RCL_RET_OK) {
    DRONE_PRINTLN("[State] FAIL: set_motors_srv executor");
    return false;
  }

  DRONE_PRINTLN("[State] onCreate: tare_srv...");
  // Tare service (reset yaw reference)
  if (rclc_service_init_default(
          &tare_srv_, node,
          ROSIDL_GET_SRV_TYPE_SUPPORT(mcu_msgs, srv, DroneTare),
          "/mcu_drone/tare") != RCL_RET_OK) {
    DRONE_PRINTLN("[State] FAIL: tare_srv init");
    return false;
  }
  if (rclc_executor_add_service(executor, &tare_srv_, &tare_req_, &tare_res_,
                                tareCallback) != RCL_RET_OK) {
    DRONE_PRINTLN("[State] FAIL: tare_srv executor");
    return false;
  }

  DRONE_PRINTLN("[State] onCreate: set_param_srv...");
  // SetParam service (runtime PID/flight parameter tuning)
  if (rclc_service_init_default(
          &set_param_srv_, node,
          ROSIDL_GET_SRV_TYPE_SUPPORT(mcu_msgs, srv, DroneSetParam),
          "/mcu_drone/set_param") != RCL_RET_OK) {
    DRONE_PRINTLN("[State] FAIL: set_param_srv init");
    return false;
  }
  if (rclc_executor_add_service(executor, &set_param_srv_, &set_param_req_,
                                &set_param_res_, setParamCallback) !=
      RCL_RET_OK) {
    DRONE_PRINTLN("[State] FAIL: set_param_srv executor");
    return false;
  }

  DRONE_PRINTLN("[State] onCreate: save_config_srv...");
  // Save config service (reuses DroneTare type)
  if (rclc_service_init_default(
          &save_config_srv_, node,
          ROSIDL_GET_SRV_TYPE_SUPPORT(mcu_msgs, srv, DroneTare),
          "/mcu_drone/save_config") != RCL_RET_OK) {
    DRONE_PRINTLN("[State] FAIL: save_config_srv init");
    return false;
  }
  if (rclc_executor_add_service(executor, &save_config_srv_, &save_config_req_,
                                &save_config_res_, saveConfigCallback) !=
      RCL_RET_OK) {
    DRONE_PRINTLN("[State] FAIL: save_config_srv executor");
    return false;
  }

  DRONE_PRINTLN("[State] onCreate: ready_srv...");
  // Ready for takeoff service (reuses DroneTare type)
  if (rclc_service_init_default(
          &ready_srv_, node,
          ROSIDL_GET_SRV_TYPE_SUPPORT(mcu_msgs, srv, DroneTare),
          "/mcu_drone/ready_for_takeoff") != RCL_RET_OK) {
    DRONE_PRINTLN("[State] FAIL: ready_srv init");
    return false;
  }
  if (rclc_executor_add_service(executor, &ready_srv_, &ready_req_,
                                &ready_res_, readyCallback) != RCL_RET_OK) {
    DRONE_PRINTLN("[State] FAIL: ready_srv executor");
    return false;
  }

  DRONE_PRINTLN("[State] onCreate: get_config_srv...");
  // Get config service — publishes CONFIG: via debug topic on demand
  if (rclc_service_init_default(
          &get_config_srv_, node,
          ROSIDL_GET_SRV_TYPE_SUPPORT(mcu_msgs, srv, DroneTare),
          "/mcu_drone/get_config") != RCL_RET_OK) {
    DRONE_PRINTLN("[State] FAIL: get_config_srv init");
    return false;
  }
  if (rclc_executor_add_service(executor, &get_config_srv_, &get_config_req_,
                                &get_config_res_, getConfigCallback) !=
      RCL_RET_OK) {
    DRONE_PRINTLN("[State] FAIL: get_config_srv executor");
    return false;
  }

  // Initialize string fields so rosidl_runtime_c__String__assign
  // can safely realloc on each service call without leaking.
  rosidl_runtime_c__String__init(&arm_res_.message);
  rosidl_runtime_c__String__init(&takeoff_res_.message);
  rosidl_runtime_c__String__init(&land_res_.message);
  rosidl_runtime_c__String__init(&motors_res_.message);
  rosidl_runtime_c__String__init(&tare_res_.message);
  rosidl_runtime_c__String__init(&save_config_res_.message);
  rosidl_runtime_c__String__init(&ready_res_.message);
  rosidl_runtime_c__String__init(&get_config_res_.message);
  // Pre-allocate param_name buffer so the XRCE-DDS deserializer has
  // somewhere to write the incoming string (init alone gives capacity=0).
  set_param_req_.param_name.data = (char*)malloc(64);
  set_param_req_.param_name.data[0] = '\0';
  set_param_req_.param_name.size = 0;
  set_param_req_.param_name.capacity = 64;
  rosidl_runtime_c__String__init(&set_param_res_.message);

  return true;
}

void DroneStateSubsystem::onDestroy() {
  state_pub_ = rcl_get_zero_initialized_publisher();
  cmd_vel_sub_ = rcl_get_zero_initialized_subscription();
  arm_srv_ = rcl_get_zero_initialized_service();
  takeoff_srv_ = rcl_get_zero_initialized_service();
  land_srv_ = rcl_get_zero_initialized_service();
  set_anchors_srv_ = rcl_get_zero_initialized_service();
  set_motors_srv_ = rcl_get_zero_initialized_service();
  tare_srv_ = rcl_get_zero_initialized_service();
  set_param_srv_ = rcl_get_zero_initialized_service();
  save_config_srv_ = rcl_get_zero_initialized_service();
  ready_srv_ = rcl_get_zero_initialized_service();
  get_config_srv_ = rcl_get_zero_initialized_service();

  // Free string buffers allocated by rosidl_runtime_c__String__assign
  rosidl_runtime_c__String__fini(&arm_res_.message);
  rosidl_runtime_c__String__fini(&takeoff_res_.message);
  rosidl_runtime_c__String__fini(&land_res_.message);
  rosidl_runtime_c__String__fini(&motors_res_.message);
  rosidl_runtime_c__String__fini(&tare_res_.message);
  rosidl_runtime_c__String__fini(&save_config_res_.message);
  rosidl_runtime_c__String__fini(&ready_res_.message);
  rosidl_runtime_c__String__fini(&get_config_res_.message);
  rosidl_runtime_c__String__fini(&set_param_req_.param_name);
  rosidl_runtime_c__String__fini(&set_param_res_.message);
}

void DroneStateSubsystem::update() {
  uint32_t now = millis();
  float dt = (last_update_ms_ > 0) ? (now - last_update_ms_) * 0.001f : 0.0f;
  last_update_ms_ = now;

#if DRONE_ENABLE_HEIGHT
  float alt = height_.getAltitudeM();
#else
  float alt = 0.0f;
#endif

  switch (state_) {
    case DroneState::INIT:
      if (all_sensors_ready_) {
        flight_.disarm();  // Ensure clean state on first transition
        target_altitude_ = 0.0f;
        hold_altitude_ = 0.0f;
        emergency_start_ms_ = 0;
        last_cmd_vel_ms_ = 0;
        cmd_roll_ = cmd_pitch_ = cmd_yaw_rate_ = cmd_alt_rate_ = 0.0f;
        state_ = DroneState::UNARMED;
      }
      break;

    case DroneState::UNARMED:
      // Defensive: ensure flight controller is disarmed in case of state desync
      // (e.g. micro-ROS reconnect resets state but not flight controller)
      if (flight_.isArmed()) {
        flight_.disarm();
        DRONE_PRINTLN("[State] Defensive disarm — flight was still armed");
      }
      break;

    case DroneState::ARMED:
      // Waiting for takeoff or motor override
      break;

    case DroneState::LAUNCHING: {
      // Ramp altitude target gradually to avoid throttle step change.
      // hold_altitude_ starts at 0 (set in takeoffCallback) and ramps
      // up to target_altitude_ at LAUNCH_CLIMB_RATE m/s.
      hold_altitude_ += Config::LAUNCH_CLIMB_RATE * dt;
      if (hold_altitude_ > target_altitude_) {
        hold_altitude_ = target_altitude_;
      }

      FlightSetpoint sp;
      sp.altitude_hold = true;
      sp.altitude_des = hold_altitude_;
      sp.roll_des = 0.0f;
      sp.pitch_des = 0.0f;
      sp.yaw_rate_des = 0.0f;
      flight_.setSetpoint(sp);

      if (alt >= target_altitude_ * 0.95f) {
        hold_altitude_ = target_altitude_;
        state_ = DroneState::VELOCITY_CONTROL;
      }

      // Timeout: if stuck in LAUNCHING too long, emergency land
      if (now - launch_start_ms_ > Config::LAUNCH_TIMEOUT_MS) {
        state_ = DroneState::EMERGENCY_LAND;
      }
      break;
    }

    case DroneState::VELOCITY_CONTROL: {
      // cmd_vel timeout → hover
      updateCmdVel(dt);
      break;
    }

    case DroneState::LANDING: {
      // Descend at fixed rate
      hold_altitude_ -= Config::LANDING_DESCENT_RATE * dt;
      if (hold_altitude_ < 0.0f) hold_altitude_ = 0.0f;

      FlightSetpoint sp;
      sp.altitude_hold = true;
      sp.altitude_des = hold_altitude_;
      sp.roll_des = 0.0f;
      sp.pitch_des = 0.0f;
      sp.yaw_rate_des = 0.0f;
      flight_.setSetpoint(sp);

      if (alt < Config::LANDED_ALT_M) {
        flight_.disarm();
        state_ = DroneState::UNARMED;
      }

      // Timeout: if stuck in LANDING (sensor failed), force disarm
      if (now - landing_start_ms_ > Config::LANDING_TIMEOUT_MS) {
        flight_.disarm();
        state_ = DroneState::UNARMED;
      }
      break;
    }

    case DroneState::READY_FOR_TAKEOFF: {
      FlightSetpoint sp;
      sp.altitude_hold = false;
      sp.throttle = flight_.ready_throttle;
      sp.roll_des = 0.0f;
      sp.pitch_des = 0.0f;
      sp.yaw_rate_des = 0.0f;
      flight_.setSetpoint(sp);
      break;
    }

    case DroneState::EMERGENCY_LAND: {
      FlightSetpoint sp;
      sp.roll_des = 0.0f;
      sp.pitch_des = 0.0f;
      sp.yaw_rate_des = 0.0f;

#if DRONE_ENABLE_HEIGHT
      if (height_.isValid()) {
        // Controlled descent with height sensor
        hold_altitude_ -= Config::LANDING_DESCENT_RATE * dt;
        if (hold_altitude_ < 0.0f) hold_altitude_ = 0.0f;
        sp.altitude_hold = true;
        sp.altitude_des = hold_altitude_;
      } else {
        // Open-loop descent
        sp.altitude_hold = false;
        sp.throttle = Config::EMERGENCY_THROTTLE;
      }
#else
      sp.altitude_hold = false;
      sp.throttle = Config::EMERGENCY_THROTTLE;
#endif
      flight_.setSetpoint(sp);

      // Timeout: disarm after EMERGENCY_DISARM_MS
      if (emergency_start_ms_ == 0) emergency_start_ms_ = now;
      if (now - emergency_start_ms_ > Config::EMERGENCY_DISARM_MS) {
        flight_.disarm();
        emergency_start_ms_ = 0;
        state_ = DroneState::UNARMED;
      }

#if DRONE_ENABLE_HEIGHT
      if (height_.isValid() && alt < Config::LANDED_ALT_M) {
        flight_.disarm();
        emergency_start_ms_ = 0;
        state_ = DroneState::UNARMED;
      }
#endif
      break;
    }
  }

  // Populate state message at configured rate (deferred publishing)
  if (now - last_state_pub_ms_ >=
      (1000 / Config::STATE_PUB_RATE_HZ)) {
    populateStateMsg();
    last_state_pub_ms_ = now;
  }
}

void DroneStateSubsystem::updateCmdVel(float dt) {
  uint32_t now = millis();
  bool timed_out =
      (now - last_cmd_vel_ms_ > Config::CMD_VEL_TIMEOUT_MS) &&
      last_cmd_vel_ms_ > 0;

  FlightSetpoint sp;
  sp.altitude_hold = true;

  if (timed_out) {
    // Auto-hover: zero attitude, hold altitude
    sp.roll_des = 0.0f;
    sp.pitch_des = 0.0f;
    sp.yaw_rate_des = 0.0f;
    sp.altitude_des = hold_altitude_;
  } else {
    sp.roll_des = cmd_roll_;
    sp.pitch_des = cmd_pitch_;
    sp.yaw_rate_des = cmd_yaw_rate_;
    // Adjust hold altitude by cmd_vel z rate
    hold_altitude_ += cmd_alt_rate_ * dt;
    hold_altitude_ = clamp(hold_altitude_, 0.0f, Config::ALTITUDE_CEILING_M);
    sp.altitude_des = hold_altitude_;
  }

  flight_.setSetpoint(sp);
}

void DroneStateSubsystem::populateStateMsg() {
  if (!state_pub_.impl) return;

  EKFState ekf = ekf_.getState();
  IMUData imu = gyro_.getData();

  if (data_mutex_) xSemaphoreTake(data_mutex_, portMAX_DELAY);
  state_msg_.state = static_cast<uint8_t>(state_);
#if DRONE_ENABLE_HEIGHT
  state_msg_.altitude = height_.getAltitudeM();
#else
  state_msg_.altitude = 0.0f;
#endif
  state_msg_.pos_x = ekf.x;
  state_msg_.pos_y = ekf.y;
  state_msg_.roll = imu.roll;
  state_msg_.pitch = imu.pitch;
  state_msg_.yaw = imu.yaw;
  data_ready_ = true;
  if (data_mutex_) xSemaphoreGive(data_mutex_);
}

void DroneStateSubsystem::publishAll() {
  if (!data_mutex_) return;
  xSemaphoreTake(data_mutex_, portMAX_DELAY);
  if (data_ready_ && state_pub_.impl) {
    (void)rcl_publish(&state_pub_, &state_msg_, NULL);
    data_ready_ = false;
  }
  xSemaphoreGive(data_mutex_);
}

// ── Service callbacks ────────────────────────────────────────

void DroneStateSubsystem::armCallback(const void* req_raw, void* res_raw) {
  auto* req = (const mcu_msgs__srv__DroneArm_Request*)req_raw;
  auto* res = (mcu_msgs__srv__DroneArm_Response*)res_raw;
  auto* self = s_instance_;
  if (!self) {
    res->success = false;
    return;
  }

  if (req->arm) {
    if (self->state_ != DroneState::UNARMED) {
      res->success = false;
      rosidl_runtime_c__String__assign(&res->message, "Not in UNARMED state");
      return;
    }
    if (!self->gyro_.isInitialized()) {
      res->success = false;
      rosidl_runtime_c__String__assign(&res->message, "IMU not ready");
      return;
    }
#if DRONE_ENABLE_HEIGHT
    if (!self->height_.isInitialized()) {
      res->success = false;
      rosidl_runtime_c__String__assign(&res->message, "Height sensor not ready");
      return;
    }
#endif
    self->flight_.arm();
    self->state_ = DroneState::ARMED;
    res->success = true;
    rosidl_runtime_c__String__assign(&res->message, "Armed");
  } else {
    // Disarm — allowed from any state as emergency safety
    if (self->state_ == DroneState::UNARMED) {
      res->success = false;
      rosidl_runtime_c__String__assign(&res->message, "Already disarmed");
    } else {
      bool was_flying = self->isFlying();
      self->flight_.disarm();
      self->emergency_start_ms_ = 0;
      self->state_ = DroneState::UNARMED;
      res->success = true;
      if (was_flying) {
        rosidl_runtime_c__String__assign(&res->message, "Emergency disarmed");
      } else {
        rosidl_runtime_c__String__assign(&res->message, "Disarmed");
      }
    }
  }
}

void DroneStateSubsystem::takeoffCallback(const void* req_raw,
                                           void* res_raw) {
  auto* req = (const mcu_msgs__srv__DroneTakeoff_Request*)req_raw;
  auto* res = (mcu_msgs__srv__DroneTakeoff_Response*)res_raw;
  auto* self = s_instance_;
  if (!self) {
    res->success = false;
    return;
  }

  if (self->state_ != DroneState::ARMED &&
      self->state_ != DroneState::READY_FOR_TAKEOFF) {
    res->success = false;
    rosidl_runtime_c__String__assign(&res->message, "Not in ARMED or READY state");
    return;
  }

  float alt = clamp(req->target_altitude, 0.3f, Config::ALTITUDE_CEILING_M);
  self->target_altitude_ = alt;
  self->hold_altitude_ = 0.0f;
  self->launch_start_ms_ = millis();
  self->flight_.clearOverride();
  self->state_ = DroneState::LAUNCHING;
  res->success = true;
  rosidl_runtime_c__String__assign(&res->message, "Launching");
}

void DroneStateSubsystem::landCallback(const void* /*req_raw*/,
                                        void* res_raw) {
  auto* res = (mcu_msgs__srv__DroneLand_Response*)res_raw;
  auto* self = s_instance_;
  if (!self) {
    res->success = false;
    return;
  }

  if (!self->isFlying()) {
    res->success = false;
    rosidl_runtime_c__String__assign(&res->message, "Not flying");
    return;
  }

  self->landing_start_ms_ = millis();
#if DRONE_ENABLE_HEIGHT
  self->hold_altitude_ = self->height_.getAltitudeM();
#else
  self->hold_altitude_ = self->target_altitude_;
#endif
  self->state_ = DroneState::LANDING;
  res->success = true;
  rosidl_runtime_c__String__assign(&res->message, "Landing");
}

void DroneStateSubsystem::setAnchorsCallback(const void* req_raw,
                                              void* res_raw) {
  auto* req = (const mcu_msgs__srv__DroneSetAnchors_Request*)req_raw;
  auto* res = (mcu_msgs__srv__DroneSetAnchors_Response*)res_raw;
  auto* self = s_instance_;
  if (!self) {
    res->success = false;
    return;
  }

  uint8_t count = (req->num_anchors > 4) ? 4 : req->num_anchors;
  AnchorInfo anchors[4];
  for (uint8_t i = 0; i < count; i++) {
    anchors[i].id = req->anchor_ids[i];
    anchors[i].x = req->anchor_x[i];
    anchors[i].y = req->anchor_y[i];
    anchors[i].valid = true;
  }
  self->ekf_.setAnchors(anchors, count);
  res->success = true;
}

void DroneStateSubsystem::setMotorsCallback(const void* req_raw,
                                             void* res_raw) {
  auto* req = (const mcu_msgs__srv__DroneSetMotors_Request*)req_raw;
  auto* res = (mcu_msgs__srv__DroneSetMotors_Response*)res_raw;
  auto* self = s_instance_;
  if (!self) {
    res->success = false;
    return;
  }

  if (self->state_ != DroneState::ARMED) {
    res->success = false;
    rosidl_runtime_c__String__assign(
        &res->message, "Motor override only in ARMED (not flying)");
    return;
  }

  self->flight_.setMotorsOverride(req->motor_speeds);
  res->success = true;
  rosidl_runtime_c__String__assign(&res->message, "Motors set");
}

void DroneStateSubsystem::tareCallback(const void* /*req_raw*/,
                                        void* res_raw) {
  auto* res = (mcu_msgs__srv__DroneTare_Response*)res_raw;
  auto* self = s_instance_;
  if (!self) {
    res->success = false;
    return;
  }

  if (!self->gyro_.isInitialized()) {
    res->success = false;
    rosidl_runtime_c__String__assign(&res->message, "IMU not ready");
    return;
  }

  self->gyro_.tare();
  res->success = true;
  rosidl_runtime_c__String__assign(&res->message, "Tared");
}

void DroneStateSubsystem::setParamCallback(const void* req_raw,
                                            void* res_raw) {
  auto* req = (const mcu_msgs__srv__DroneSetParam_Request*)req_raw;
  auto* res = (mcu_msgs__srv__DroneSetParam_Response*)res_raw;
  auto* self = s_instance_;
  if (!self) {
    res->success = false;
    return;
  }

  const char* name = req->param_name.data;
  float value = req->value;

  if (!name || name[0] == '\0') {
    res->success = false;
    rosidl_runtime_c__String__assign(&res->message, "Empty param name");
    return;
  }

  if (self->flight_.setParam(name, value)) {
    res->success = true;
    char buf[64];
    snprintf(buf, sizeof(buf), "Set %s = %.4f", name, value);
    rosidl_runtime_c__String__assign(&res->message, buf);
    DRONE_PRINTF("[State] Param: %s = %.4f\n", name, value);
  } else {
    res->success = false;
    char buf[64];
    snprintf(buf, sizeof(buf), "Unknown param: %s", name);
    rosidl_runtime_c__String__assign(&res->message, buf);
  }
}

void DroneStateSubsystem::saveConfigCallback(const void* /*req_raw*/,
                                              void* res_raw) {
  auto* res = (mcu_msgs__srv__DroneTare_Response*)res_raw;
  auto* self = s_instance_;
  if (!self) {
    res->success = false;
    return;
  }

  size_t count = self->config_store_.save(self->flight_);
  if (count > 0) {
    res->success = true;
    char buf[48];
    snprintf(buf, sizeof(buf), "Saved %u params to NVS", (unsigned)count);
    rosidl_runtime_c__String__assign(&res->message, buf);
    DRONE_PRINTF("[State] Config saved: %u params\n", (unsigned)count);
  } else {
    res->success = false;
    rosidl_runtime_c__String__assign(&res->message, "NVS save failed");
  }
}

void DroneStateSubsystem::readyCallback(const void* /*req_raw*/,
                                         void* res_raw) {
  auto* res = (mcu_msgs__srv__DroneTare_Response*)res_raw;
  auto* self = s_instance_;
  if (!self) {
    res->success = false;
    return;
  }

  if (self->state_ != DroneState::ARMED) {
    res->success = false;
    rosidl_runtime_c__String__assign(&res->message, "Not in ARMED state");
    return;
  }

  self->state_ = DroneState::READY_FOR_TAKEOFF;
  res->success = true;
  rosidl_runtime_c__String__assign(&res->message, "READY_FOR_TAKEOFF");
}

void DroneStateSubsystem::getConfigCallback(const void* /*req_raw*/,
                                             void* res_raw) {
  auto* res = (mcu_msgs__srv__DroneTare_Response*)res_raw;
  auto* self = s_instance_;
  if (!self) {
    res->success = false;
    return;
  }

  size_t chunks = self->config_store_.formatConfigString(self->flight_,
      [](const char* msg) {
        if (s_instance_) s_instance_->mr_.debugLog(msg);
      });

  res->success = true;
  char buf[48];
  snprintf(buf, sizeof(buf), "Published %u config chunks", (unsigned)chunks);
  rosidl_runtime_c__String__assign(&res->message, buf);
}

}  // namespace Drone
