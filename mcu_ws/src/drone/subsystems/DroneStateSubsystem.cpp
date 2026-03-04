#include "DroneStateSubsystem.h"

#include <micro_ros_utilities/string_utilities.h>
#include <math_utils.h>

using secbot::utils::clamp;
using secbot::utils::deg2rad;

namespace Drone {

DroneStateSubsystem* DroneStateSubsystem::s_instance_ = nullptr;

bool DroneStateSubsystem::onCreate(rcl_node_t* node,
                                   rclc_executor_t* executor) {
  s_instance_ = this;

  // State publisher (best-effort)
  if (rclc_publisher_init_best_effort(
          &state_pub_, node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(mcu_msgs, msg, DroneState),
          "/mcu_drone/state") != RCL_RET_OK)
    return false;

  // cmd_vel subscriber
  if (rclc_subscription_init_default(
          &cmd_vel_sub_, node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
          "/mcu_drone/cmd_vel") != RCL_RET_OK)
    return false;
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
                                     ON_NEW_DATA) != RCL_RET_OK)
    return false;

  // Arm service
  if (rclc_service_init_default(
          &arm_srv_, node,
          ROSIDL_GET_SRV_TYPE_SUPPORT(mcu_msgs, srv, DroneArm),
          "/mcu_drone/arm") != RCL_RET_OK)
    return false;
  if (rclc_executor_add_service(executor, &arm_srv_, &arm_req_, &arm_res_,
                                armCallback) != RCL_RET_OK)
    return false;

  // Takeoff service
  if (rclc_service_init_default(
          &takeoff_srv_, node,
          ROSIDL_GET_SRV_TYPE_SUPPORT(mcu_msgs, srv, DroneTakeoff),
          "/mcu_drone/takeoff") != RCL_RET_OK)
    return false;
  if (rclc_executor_add_service(executor, &takeoff_srv_, &takeoff_req_,
                                &takeoff_res_, takeoffCallback) != RCL_RET_OK)
    return false;

  // Land service
  if (rclc_service_init_default(
          &land_srv_, node,
          ROSIDL_GET_SRV_TYPE_SUPPORT(mcu_msgs, srv, DroneLand),
          "/mcu_drone/land") != RCL_RET_OK)
    return false;
  if (rclc_executor_add_service(executor, &land_srv_, &land_req_, &land_res_,
                                landCallback) != RCL_RET_OK)
    return false;

  // SetAnchors service
  if (rclc_service_init_default(
          &set_anchors_srv_, node,
          ROSIDL_GET_SRV_TYPE_SUPPORT(mcu_msgs, srv, DroneSetAnchors),
          "/mcu_drone/set_anchors") != RCL_RET_OK)
    return false;
  if (rclc_executor_add_service(executor, &set_anchors_srv_, &anchors_req_,
                                &anchors_res_, setAnchorsCallback) !=
      RCL_RET_OK)
    return false;

  // SetMotors service
  if (rclc_service_init_default(
          &set_motors_srv_, node,
          ROSIDL_GET_SRV_TYPE_SUPPORT(mcu_msgs, srv, DroneSetMotors),
          "/mcu_drone/set_motors") != RCL_RET_OK)
    return false;
  if (rclc_executor_add_service(executor, &set_motors_srv_, &motors_req_,
                                &motors_res_, setMotorsCallback) != RCL_RET_OK)
    return false;

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
}

void DroneStateSubsystem::update() {
  uint32_t now = millis();

#if DRONE_ENABLE_HEIGHT
  float alt = height_.getAltitudeM();
#else
  float alt = 0.0f;
#endif

  switch (state_) {
    case DroneState::INIT:
      if (all_sensors_ready_) {
        state_ = DroneState::UNARMED;
      }
      break;

    case DroneState::UNARMED:
      // Waiting for arm service
      break;

    case DroneState::ARMED:
      // Waiting for takeoff or motor override
      break;

    case DroneState::LAUNCHING: {
      // Ascending to target altitude
      FlightSetpoint sp;
      sp.altitude_hold = true;
      sp.altitude_des = target_altitude_;
      sp.roll_des = 0.0f;
      sp.pitch_des = 0.0f;
      sp.yaw_rate_des = 0.0f;
      flight_.setSetpoint(sp);

      if (alt >= target_altitude_ * 0.95f) {
        hold_altitude_ = target_altitude_;
        state_ = DroneState::VELOCITY_CONTROL;
      }
      break;
    }

    case DroneState::VELOCITY_CONTROL: {
      // cmd_vel timeout → hover
      updateCmdVel();
      break;
    }

    case DroneState::LANDING: {
      // Descend at fixed rate
      hold_altitude_ -= Config::LANDING_DESCENT_RATE * 0.1f;  // ~10Hz update
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
        hold_altitude_ -= Config::LANDING_DESCENT_RATE * 0.1f;
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

  // Publish state at configured rate
  if (now - last_state_pub_ms_ >=
      (1000 / Config::STATE_PUB_RATE_HZ)) {
    publishState();
    last_state_pub_ms_ = now;
  }
}

void DroneStateSubsystem::updateCmdVel() {
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
    hold_altitude_ += cmd_alt_rate_ * 0.1f;  // ~10Hz
    hold_altitude_ = clamp(hold_altitude_, 0.0f, Config::ALTITUDE_CEILING_M);
    sp.altitude_des = hold_altitude_;
  }

  flight_.setSetpoint(sp);
}

void DroneStateSubsystem::publishState() {
  if (!state_pub_.impl) return;

  EKFState ekf = ekf_.getState();
  IMUData imu = gyro_.getData();

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

  rcl_publish(&state_pub_, &state_msg_, NULL);
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
      res->message =
          micro_ros_string_utilities_set(res->message, "Not in UNARMED state");
      return;
    }
    if (!self->gyro_.isInitialized()) {
      res->success = false;
      res->message =
          micro_ros_string_utilities_set(res->message, "IMU not ready");
      return;
    }
#if DRONE_ENABLE_HEIGHT
    if (!self->height_.isInitialized()) {
      res->success = false;
      res->message =
          micro_ros_string_utilities_set(res->message, "Height sensor not ready");
      return;
    }
#endif
    self->flight_.arm();
    self->state_ = DroneState::ARMED;
    res->success = true;
    res->message = micro_ros_string_utilities_set(res->message, "Armed");
  } else {
    // Disarm - only from ARMED (not flying)
    if (self->state_ == DroneState::ARMED) {
      self->flight_.disarm();
      self->state_ = DroneState::UNARMED;
      res->success = true;
      res->message = micro_ros_string_utilities_set(res->message, "Disarmed");
    } else if (self->isFlying()) {
      res->success = false;
      res->message = micro_ros_string_utilities_set(
          res->message, "Cannot disarm while flying, use land");
    } else {
      res->success = false;
      res->message =
          micro_ros_string_utilities_set(res->message, "Already disarmed");
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

  if (self->state_ != DroneState::ARMED) {
    res->success = false;
    res->message =
        micro_ros_string_utilities_set(res->message, "Not in ARMED state");
    return;
  }

  float alt = clamp(req->target_altitude, 0.3f, Config::ALTITUDE_CEILING_M);
  self->target_altitude_ = alt;
  self->hold_altitude_ = 0.0f;
  self->flight_.clearOverride();
  self->state_ = DroneState::LAUNCHING;
  res->success = true;
  res->message = micro_ros_string_utilities_set(res->message, "Launching");
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
    res->message =
        micro_ros_string_utilities_set(res->message, "Not flying");
    return;
  }

#if DRONE_ENABLE_HEIGHT
  self->hold_altitude_ = self->height_.getAltitudeM();
#else
  self->hold_altitude_ = self->target_altitude_;
#endif
  self->state_ = DroneState::LANDING;
  res->success = true;
  res->message = micro_ros_string_utilities_set(res->message, "Landing");
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
    res->message = micro_ros_string_utilities_set(
        res->message, "Motor override only in ARMED (not flying)");
    return;
  }

  self->flight_.setMotorsOverride(req->motor_speeds);
  res->success = true;
  res->message =
      micro_ros_string_utilities_set(res->message, "Motors set");
}

}  // namespace Drone
