/**
 * @file teensy-test-rc-subsystem.cpp
 * @brief Focused test — FlySky RC receiver via IBUS, publishes to micro-ROS.
 * @date 2026-02-24
 *
 * Hardware expected:
 *   FlySky receiver IBUS output → Teensy Serial8 RX (pin 34)
 *
 * micro-ROS topics published:
 *   /mcu_robot/rc   mcu_msgs/RC   20 Hz
 *
 * Usage:
 *   ros2 topic echo /mcu_robot/rc
 */

#include <Arduino.h>
#include <TeensyThreads.h>
#include <microros_manager_robot.h>

#include "robot/RobotPins.h"
#include "robot/subsystems/RCSubsystem.h"

using namespace Subsystem;

// --- micro-ROS manager ---
static MicrorosManagerSetup g_mr_setup("rc_test_mr");
static MicrorosManager g_mr(g_mr_setup);

// --- RC subsystem (FlySky, Serial8 RX = pin 34) ---
static RCSubsystemSetup g_rc_setup("rc_subsystem", &Serial8);
static RCSubsystem g_rc(g_rc_setup);

// --- Blink task (visual heartbeat) ---
static void blink_task(void*) {
  pinMode(LED_BUILTIN, OUTPUT);
  while (true) {
    digitalWriteFast(LED_BUILTIN, HIGH);
    threads.delay(500);
    digitalWriteFast(LED_BUILTIN, LOW);
    threads.delay(500);
  }
}

// --- Serial print task (debug) ---
static void print_task(void*) {
  while (true) {
    Serial.printf("RC test running — agent %s\r\n",
                  g_mr.isConnected() ? "CONNECTED" : "waiting...");
    threads.delay(2000);
  }
}

void setup() {
  Serial.begin(921600);
  delay(500);
  Serial.println(PSTR("\r\nSEC26 Robot — RC Subsystem Test\r\n"));

  // 1. Init subsystems
  g_mr.init();
  g_rc.init();

  // 2. Register micro-ROS participant
  g_mr.registerParticipant(&g_rc);

  // 3. Start threads
  g_mr.beginThreaded(8192, 4);       // ROS agent (highest priority)
  g_rc.beginThreaded(1024, 3, 5);    // IBUS polling @ 200 Hz
  threads.addThread(print_task, nullptr, 1024);
  threads.addThread(blink_task, nullptr, 512);

  Serial.println(PSTR("setup(): RC test threads started."));
}

void loop() { threads.delay(100); }
