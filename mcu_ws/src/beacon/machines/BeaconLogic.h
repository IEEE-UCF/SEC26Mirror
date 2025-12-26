/**
 * @file BeaconLogic.h
 * @date 12/22/2025
 * @author SEC26 Team
 * @brief Logic for stationary UWB beacon (anchor mode)
 */

#pragma once

#include <Arduino.h>
#include <SPI.h>
#include <microros_manager_robot.h>

#include "UWBDriver.h"
#include "beacon/machines/UWBSubsystem.h"
#include "robot/machines/HeartbeatSubsystem.h"

using namespace Subsystem;

// Pin definitions for ESP32 DW3000 connection
// Adjust these based on your hardware wiring
#define UWB_CS_PIN 5    // SPI Chip Select
#define UWB_RST_PIN 4   // Reset pin (255 if not used)
#define UWB_IRQ_PIN 17  // Interrupt pin (not used in polling mode)

// IMPORTANT: Set the beacon ID here (must be unique for each beacon)
// Valid IDs: 10, 11, 12, 13 (or any unique ID in your system)
#ifndef BEACON_ID
#define BEACON_ID 10  // Change this for each beacon!
#endif

// --- micro-ROS manager ---
static MicrorosManagerSetup g_mr_setup("microros_manager");
static MicrorosManager g_mr(g_mr_setup);

// --- Heartbeat subsystem ---
static HeartbeatSubsystemSetup g_hb_setup("heartbeat_subsystem");
static HeartbeatSubsystem g_hb(g_hb_setup);

// --- UWB Driver and Subsystem ---
static Drivers::UWBDriverSetup g_uwb_driver_setup(
    "uwb_driver",
    Drivers::UWBMode::ANCHOR,  // Beacon is an ANCHOR (responder)
    BEACON_ID, UWB_CS_PIN, UWB_RST_PIN);
static Drivers::UWBDriver g_uwb_driver(g_uwb_driver_setup);

static UWBSubsystemSetup g_uwb_setup("uwb_subsystem", &g_uwb_driver);
static UWBSubsystem g_uwb(g_uwb_setup);

// --- Arduino sketch entry points ---
void setup() {
  Serial.begin(115200);
  delay(2000);  // Wait for serial
  Serial.print("UWB Beacon ");
  Serial.print(BEACON_ID);
  Serial.println(" starting...");

  // Initialize SPI for DW3000
  SPI.begin();

  // Initialize subsystems
  bool ok = true;
  ok = ok && g_mr.init();
  ok = ok && g_hb.init();
  ok = ok && g_uwb.init();

  if (!ok) {
    Serial.println("ERROR: Failed to initialize subsystems!");
    while (1) {
      delay(1000);
    }
  }

  // Register micro-ROS participants and begin
  g_mr.registerParticipant(&g_hb);
  g_mr.registerParticipant(&g_uwb);
  g_mr.begin();

  // Begin subsystems
  g_hb.begin();
  g_uwb.begin();

  Serial.print("Beacon ");
  Serial.print(BEACON_ID);
  Serial.println(" initialized and waiting for ranging requests!");
}

void loop() {
  g_mr.update();
  g_hb.update();
  g_uwb.update();

  delay(1);  // Small delay to prevent watchdog issues
}
