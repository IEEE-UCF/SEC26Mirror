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
    // Height sensor timeout
    if (now - height_.lastValidMs() > Config::HEIGHT_TIMEOUT_MS &&
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
};

}  // namespace Drone
