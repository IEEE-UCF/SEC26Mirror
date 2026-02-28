/**
 * @file UWBSubsystem.h
 * @brief Robot-specific UWB configuration (DW3000 tag, ID 13)
 *
 * Wraps the shared UWB subsystem from lib/subsystems/ with robot-specific
 * pin assignments, device IDs, and topic namespace.
 *
 * UWB SPI0 wiring (see RobotPins.h):
 *   CS=10, MOSI=11, MISO=12, CLK=13
 *
 * Publishes: /mcu_robot/uwb/ranging  (mcu_msgs/UWBRanging, 10 Hz)
 */
#pragma once

#include <UWBSubsystem.h>  // lib/subsystems

#include "../RobotPins.h"

// Robot UWB tag ID (reservation: 10-12 = beacons, 13 = robot, 14 = minibot)
inline constexpr uint8_t ROBOT_UWB_TAG_ID = 13;

// Beacon anchor IDs the robot ranges to
inline constexpr uint8_t ROBOT_UWB_ANCHOR_IDS[] = {10, 11, 12};
inline constexpr uint8_t ROBOT_UWB_NUM_ANCHORS = 3;

// Robot UWB topic (under /mcu_robot/ namespace)
inline constexpr const char* ROBOT_UWB_TOPIC = "/mcu_robot/uwb/ranging";
