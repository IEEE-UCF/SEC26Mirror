/**
 * @file RobotLogic.h
 * @date 12/16/25
 * @author Aldem Pido
 * @brief Main logic for the SEC26 robot.
 */

#pragma once

#include <Arduino.h>
#include <microros_manager_robot.h>

#include "../RobotConstants.h"
#include "I2CPowerDriver.h"
#include "PCA9685Manager.h"
#include "TOF.h"
#include "robot/machines/HeartbeatSubsystem.h"
#include "robot/machines/McuSubsystem.h"
#include "robot/machines/RobotManager.h"
#include "robot/subsystems/ArmSubsystem.h"
#include "robot/subsystems/BatterySubsystem.h"
#include "robot/subsystems/RCSubsystem.h"
#include "robot/subsystems/SensorSubsystem.h"

using namespace Subsystem;

// --- micro-ROS manager ---
static MicrorosManagerSetup g_mr_setup("microros_manager");
static MicrorosManager g_mr(g_mr_setup);

// --- Heartbeat subsystem ---
static HeartbeatSubsystemSetup g_hb_setup("heartbeat_subsystem");
static HeartbeatSubsystem g_hb(g_hb_setup);

// --- Battery subsystem ---
static Drivers::I2CPowerDriverSetup g_power_driver_setup("power_driver", 0x40,
                                                         10.0f, 0.015f);
static Drivers::I2CPowerDriver g_power_driver(g_power_driver_setup);
static BatterySubsystemSetup g_battery_setup("battery_subsystem",
                                             &g_power_driver);
static BatterySubsystem g_battery(g_battery_setup);

// --- Sensor subsystem (TOF) ---
static Drivers::TOFDriverSetup g_tof_setup("tof_sensor", 500, 0);
static Drivers::TOFDriver g_tof_driver(g_tof_setup);
static std::vector<Drivers::TOFDriver*> g_tof_drivers = {&g_tof_driver};
static SensorSubsystemSetup g_sensor_setup("sensor_subsystem", g_tof_drivers);
static SensorSubsystem g_sensor(g_sensor_setup);

// --- MCU subsystem wired with callbacks ---
// Forward-declare callbacks so we can construct the subsystem first
static bool mcu_init_cb();
static bool mcu_arm_cb();
static bool mcu_begin_cb();
static void mcu_update_cb();
static void mcu_stop_cb();
static void mcu_reset_cb();

static MCUSubsystemSetup g_mcu_setup("mcu_subsystem");
static MCUSubsystemCallbacks g_mcu_cbs{mcu_init_cb,  mcu_arm_cb,
                                       mcu_begin_cb, mcu_update_cb,
                                       mcu_stop_cb,  mcu_reset_cb};
static MCUSubsystem g_mcu(g_mcu_setup, g_mcu_cbs);

// --- PCA9685 manager (servo driver) ---
static Robot::PCA9685ManagerSetup g_pca_mgr_setup("pca_manager");
static Robot::PCA9685Manager g_pca_mgr(g_pca_mgr_setup);
// create a single PCA device instance; manager owns the driver
static Robot::PCA9685Driver* g_pca0 =
    g_pca_mgr.createDriver(Robot::PCA9685DriverSetup(
        "pca0", DEFAULT_PCA9685_ADDR, DEFAULT_PCA9685_FREQ));

// --- Arm subsystem (pseudo-code) ---
static Drivers::EncoderDriverSetup g_encoder_setup("arm_encoder", /*pin1*/ 1,
                                                   /*pin2*/ 2);
static Drivers::EncoderDriver g_arm_encoder(g_encoder_setup);
static ArmSubsystemSetup g_arm_setup("arm_subsystem", g_pca0, &g_arm_encoder);
static ArmSubsystem g_arm(g_arm_setup);

// --- RC subsystem ---
static RCSubsystemSetup g_rc_setup("rc_subsystem", &Serial1);
static RCSubsystem g_rc(g_rc_setup);

// Define callbacks after g_mcu is declared
static bool mcu_init_cb() {
  // Perform any one-time hardware checks; succeed for now
  bool ok = true;
  ok = ok && g_mr.init();
  ok = ok && g_hb.init();
  ok = ok && g_battery.init();
  ok = ok && g_sensor.init();
  ok = ok && g_sensor.init();
  ok = ok && g_pca_mgr.init();
  ok = ok && g_arm_encoder.init();
  ok = ok && g_arm.init();
  ok = ok && g_rc.init();
  return ok;
}

static bool mcu_arm_cb() {
  // Ready to run; succeed immediately
  return true;
}

static bool mcu_begin_cb() {
  // Register participants before beginning, all within callbacks
  g_mr.registerParticipant(&g_mcu);
  // Register other subsystems
  g_mr.registerParticipant(&g_hb);
  g_mr.registerParticipant(&g_battery);
  g_mr.registerParticipant(&g_sensor);
  g_mr.registerParticipant(&g_arm);
  g_mr.registerParticipant(&g_rc);
  g_mr.begin();
  return true;
}

static void mcu_update_cb() {
  // Drive micro-ROS executor/service checks
  g_mr.update();
  // Let heartbeat decide when to publish via its internal timer
  g_hb.update();
  // Update battery subsystem
  g_battery.update();
  // Update sensor subsystem
  g_sensor.update();
  // Update arm subsystem
  g_arm.update();
  // Update RC subsystem
  g_rc.update();
}

static void mcu_stop_cb() {
  // Tear down micro-ROS entities cleanly
  g_mr.pause();
}

static void mcu_reset_cb() {
  // No-op reset hook for now
}

// --- RobotManager wiring ---
static RobotObject g_obj_mcu(g_mcu, MS_20);  // single updater at 50 Hz
// Manager for PCA9685 updated at 50 Hz
static RobotObject g_obj_pca(g_pca_mgr, MS_20);

static std::vector<RobotObject*> g_objects{&g_obj_mcu, &g_obj_pca};
static RobotManagerSetup g_rm_setup("robot_manager", g_objects);
static RobotManager g_rm(g_rm_setup);

// --- Arduino sketch entry points ---
void setup() {
  // Initialize all robot objects
  (void)g_rm.init();

  // Transition MCU subsystem through arm and begin
  g_mcu.arm();
  g_mcu.begin();
}

void loop() {
  // Drive timed updates for all objects
  g_rm.update();
}
