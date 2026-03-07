#pragma once
/**
 * @file DroneStateSubsystem.h
 * @brief Drone state machine + ROS2 publisher/services/cmd_vel subscriber.
 *
 * States: INIT → UNARMED → ARMED → LAUNCHING → VELOCITY_CONTROL → LANDING
 *         Any flying → EMERGENCY_LAND → UNARMED
 */

#include <Arduino.h>
#include <microros_manager_robot.h>

#include <geometry_msgs/msg/twist.h>
#include <mcu_msgs/msg/drone_state.h>
#include <mcu_msgs/srv/drone_arm.h>
#include <mcu_msgs/srv/drone_land.h>
#include <mcu_msgs/srv/drone_set_anchors.h>
#include <mcu_msgs/srv/drone_set_motors.h>
#include <mcu_msgs/srv/drone_takeoff.h>

#include "../DroneConfig.h"
#include "DroneEKFSubsystem.h"
#include "DroneFlightSubsystem.h"
#include "GyroSubsystem.h"
#include "HeightSubsystem.h"

namespace Drone {

enum class DroneState : uint8_t {
  INIT = 0,
  UNARMED = 1,
  ARMED = 2,
  LAUNCHING = 3,
  VELOCITY_CONTROL = 4,
  LANDING = 5,
  EMERGENCY_LAND = 6
};

class DroneStateSubsystem : public Subsystem::IMicroRosParticipant {
 public:
  DroneStateSubsystem(DroneFlightSubsystem& flight, GyroSubsystem& gyro,
                      DroneEKFSubsystem& ekf
#if DRONE_ENABLE_HEIGHT
                      ,
                      HeightSubsystem& height
#endif
                      )
      : flight_(flight),
        gyro_(gyro),
        ekf_(ekf)
#if DRONE_ENABLE_HEIGHT
        ,
        height_(height)
#endif
  {
  }

  // State machine update (called from main loop)
  void update();

  // micro-ROS participant interface
  bool onCreate(rcl_node_t* node, rclc_executor_t* executor) override;
  void onDestroy() override;

  // State accessors
  DroneState getState() const { return state_; }
  void triggerEmergencyLand() { state_ = DroneState::EMERGENCY_LAND; }
  bool isFlying() const {
    return state_ == DroneState::LAUNCHING ||
           state_ == DroneState::VELOCITY_CONTROL ||
           state_ == DroneState::LANDING;
  }

  // Sensor readiness (called by safety or init)
  void setAllSensorsReady(bool ready) { all_sensors_ready_ = ready; }

  uint32_t lastCmdVelMs() const { return last_cmd_vel_ms_; }

 private:
  void publishState();
  void updateCmdVel(float dt);

  // Service callbacks
  static void armCallback(const void* req, void* res);
  static void takeoffCallback(const void* req, void* res);
  static void landCallback(const void* req, void* res);
  static void setAnchorsCallback(const void* req, void* res);
  static void setMotorsCallback(const void* req, void* res);

  DroneFlightSubsystem& flight_;
  GyroSubsystem& gyro_;
  DroneEKFSubsystem& ekf_;
#if DRONE_ENABLE_HEIGHT
  HeightSubsystem& height_;
#endif

  DroneState state_ = DroneState::INIT;
  bool all_sensors_ready_ = false;

  // Takeoff/landing targets
  float target_altitude_ = 0.0f;
  float hold_altitude_ = 0.0f;
  uint32_t emergency_start_ms_ = 0;

  // cmd_vel
  float cmd_roll_ = 0.0f;
  float cmd_pitch_ = 0.0f;
  float cmd_yaw_rate_ = 0.0f;
  float cmd_alt_rate_ = 0.0f;
  uint32_t last_cmd_vel_ms_ = 0;

  // micro-ROS entities
  rcl_publisher_t state_pub_{};
  mcu_msgs__msg__DroneState state_msg_{};
  rcl_subscription_t cmd_vel_sub_{};
  geometry_msgs__msg__Twist cmd_vel_msg_{};
  rcl_service_t arm_srv_{};
  rcl_service_t takeoff_srv_{};
  rcl_service_t land_srv_{};
  rcl_service_t set_anchors_srv_{};
  rcl_service_t set_motors_srv_{};

  // Service request/response buffers
  mcu_msgs__srv__DroneArm_Request arm_req_{};
  mcu_msgs__srv__DroneArm_Response arm_res_{};
  mcu_msgs__srv__DroneTakeoff_Request takeoff_req_{};
  mcu_msgs__srv__DroneTakeoff_Response takeoff_res_{};
  mcu_msgs__srv__DroneLand_Request land_req_{};
  mcu_msgs__srv__DroneLand_Response land_res_{};
  mcu_msgs__srv__DroneSetAnchors_Request anchors_req_{};
  mcu_msgs__srv__DroneSetAnchors_Response anchors_res_{};
  mcu_msgs__srv__DroneSetMotors_Request motors_req_{};
  mcu_msgs__srv__DroneSetMotors_Response motors_res_{};

  uint32_t last_state_pub_ms_ = 0;
  uint32_t last_update_ms_ = 0;

  // Static instance for service callbacks
  static DroneStateSubsystem* s_instance_;
};

}  // namespace Drone
