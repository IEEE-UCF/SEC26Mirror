/**
 * @file RobotLogic.h
 * @date 12/16/25
 * @author Aldem Pido
 * @brief Main logic for the SEC26 robot.
 */

#pragma once

#include <Arduino.h>

#include "robot/machines/RobotManager.h"
#include "robot/machines/McuSubsystem.h"
#include "robot/machines/HeartbeatSubsystem.h"
#include <microros_manager_robot.h>

using namespace Subsystem;

// --- micro-ROS manager ---
static MicrorosManagerSetup g_mr_setup("microros_manager");
static MicrorosManager g_mr(g_mr_setup);

// --- Heartbeat subsystem ---
static HeartbeatSubsystemSetup g_hb_setup("heartbeat_subsystem");
static HeartbeatSubsystem g_hb(g_hb_setup);

// --- MCU subsystem wired with callbacks ---
// Forward-declare callbacks so we can construct the subsystem first
static bool mcu_init_cb();
static bool mcu_arm_cb();
static bool mcu_begin_cb();
static void mcu_update_cb();
static void mcu_stop_cb();
static void mcu_reset_cb();

static MCUSubsystemSetup g_mcu_setup("mcu_subsystem");
static MCUSubsystemCallbacks g_mcu_cbs{mcu_init_cb, mcu_arm_cb, mcu_begin_cb,
																			 mcu_update_cb, mcu_stop_cb, mcu_reset_cb};
static MCUSubsystem g_mcu(g_mcu_setup, g_mcu_cbs);

// Define callbacks after g_mcu is declared
static bool mcu_init_cb() {
	// Perform any one-time hardware checks; succeed for now
	bool ok = g_mr.init();
	return ok;
}

static bool mcu_arm_cb() {
	// Ready to run; succeed immediately
	return true;
}

static bool mcu_begin_cb() {
	// Register participants before beginning, all within callbacks
	g_mr.registerParticipant(&g_hb);
	g_mr.registerParticipant(&g_mcu);
	g_mr.begin();
	return true;
}

static void mcu_update_cb() {
	// Drive micro-ROS executor/service checks
	g_mr.update();
	// Let heartbeat decide when to publish via its internal timer
	g_hb.update();
}

static void mcu_stop_cb() {
	// Tear down micro-ROS entities cleanly
	g_mr.pause();
}

static void mcu_reset_cb() {
	// No-op reset hook for now
}

// --- RobotManager wiring ---
static RobotObject g_obj_mcu(g_mcu, MS_20);   // single updater at 50 Hz

static std::vector<RobotObject*> g_objects{&g_obj_mcu};
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


