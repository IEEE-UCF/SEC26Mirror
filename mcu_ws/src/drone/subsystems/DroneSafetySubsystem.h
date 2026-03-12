#pragma once
/**
 * @file DroneSafetySubsystem.h
 * @brief Failure detection and emergency land trigger.
 *        Runs as a FreeRTOS task at 10Hz, priority 4.
 */

#include <Arduino.h>
#include <RTOSSubsystem.h>
#include <microros_manager_robot.h>

#include "../DroneConfig.h"
#include "DroneFlightSubsystem.h"
#include "DroneStateSubsystem.h"
#include "GyroSubsystem.h"

#if DRONE_ENABLE_HEIGHT
#include "HeightSubsystem.h"
#endif

namespace Drone {

class DroneSafetySubsystem : public Subsystem::RTOSSubsystem {
 public:
  DroneSafetySubsystem(const char* name,
                       DroneStateSubsystem& state, DroneFlightSubsystem& flight,
                       GyroSubsystem& gyro,
                       Subsystem::MicrorosManager& mr
#if DRONE_ENABLE_HEIGHT
                       ,
                       HeightSubsystem& height
#endif
                       )
      : RTOSSubsystem(setup_),
        setup_(name),
        state_(state),
        flight_(flight),
        gyro_(gyro),
        mr_(mr)
#if DRONE_ENABLE_HEIGHT
        ,
        height_(height)
#endif
  {
  }

  // RTOSSubsystem lifecycle
  bool init() override { return true; }
  void begin() override {}
  void pause() override {}
  void reset() override {}
  const char* getInfo() override { return setup_.getId(); }

  // Check all failure conditions (called by RTOSSubsystem task loop).
  void update() override {
    if (!state_.isFlying() &&
        state_.getState() != DroneState::EMERGENCY_LAND) {
      return;
    }

    uint32_t now = millis();

    // IMU failure: immediate disarm
    if (!gyro_.isInitialized()) {
      flight_.disarm();
      state_.triggerEmergencyLand();
      return;
    }

#if DRONE_ENABLE_HEIGHT
    // Height sensor timeout — only enforce after launch is complete.
    // During LAUNCHING the drone is ramping up near the ground and
    // the VL53L0X may not have valid readings yet (out of range,
    // blocked, or stale from before arm).
    if (state_.getState() != DroneState::LAUNCHING &&
        now - height_.lastValidMs() > Config::HEIGHT_TIMEOUT_MS &&
        height_.lastValidMs() > 0) {
      state_.triggerEmergencyLand();
      return;
    }

    // Altitude ceiling
    if (height_.isValid() &&
        height_.getAltitudeM() > Config::ALTITUDE_CEILING_M) {
      // Don't emergency land, just let the altitude PID handle it
      // by reducing the hold altitude in state machine
    }
#endif

    // Crash detection
    {
      IMUData imu = gyro_.getData();
      float abs_roll = fabsf(imu.roll);
      float abs_pitch = fabsf(imu.pitch);

      // Instant kill: extreme tilt (>60°) or upside down — no debounce
      if (abs_roll > 60.0f || abs_pitch > 60.0f) {
        flight_.disarm();
        state_.triggerEmergencyLand();
        return;
      }

      // Instant kill: extreme angular rate (>400 deg/s) — tumbling
      float abs_gx = fabsf(imu.gyro_x);
      float abs_gy = fabsf(imu.gyro_y);
      if (abs_gx > 400.0f || abs_gy > 400.0f) {
        flight_.disarm();
        state_.triggerEmergencyLand();
        return;
      }

      // Stall detection: motor commanded but no angular response
      // If any motor > 30% throttle but total angular rate < 5 deg/s
      // for 500ms, a motor is likely stalled (stuck in netting, etc.)
      float max_motor = 0.0f;
      for (int i = 0; i < 4; i++) {
        float m = flight_.getMotor(i);
        if (m > max_motor) max_motor = m;
      }
      float total_rate = abs_gx + abs_gy + fabsf(imu.gyro_z);
      bool possibly_stalled = max_motor > 0.3f && total_rate < 5.0f &&
                              abs_roll < 5.0f && abs_pitch < 5.0f;
      if (possibly_stalled) {
        if (stall_start_ms_ == 0) stall_start_ms_ = now;
        if (now - stall_start_ms_ > 500) {
          flight_.disarm();
          state_.triggerEmergencyLand();
          stall_start_ms_ = 0;
          return;
        }
      } else {
        stall_start_ms_ = 0;
      }
    }

    // micro-ROS disconnect
    if (!mr_.isConnected()) {
      if (disconnect_start_ms_ == 0) disconnect_start_ms_ = now;
      if (now - disconnect_start_ms_ > Config::MICROROS_TIMEOUT_MS) {
        state_.triggerEmergencyLand();
        disconnect_start_ms_ = 0;
        return;
      }
    } else {
      disconnect_start_ms_ = 0;
    }
  }

  // Called during init to check if all sensors are ready
  bool checkSensorsReady() const {
    if (!gyro_.isInitialized()) return false;
#if DRONE_ENABLE_HEIGHT
    if (!height_.isInitialized()) return false;
#endif
    return true;
  }

 private:
  Classes::BaseSetup setup_;
  DroneStateSubsystem& state_;
  DroneFlightSubsystem& flight_;
  GyroSubsystem& gyro_;
  Subsystem::MicrorosManager& mr_;
#if DRONE_ENABLE_HEIGHT
  HeightSubsystem& height_;
#endif
  uint32_t disconnect_start_ms_ = 0;
  uint32_t stall_start_ms_ = 0;
};

}  // namespace Drone
