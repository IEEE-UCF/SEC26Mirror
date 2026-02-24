/**
 * @file teensy-test-microros-freertos.cpp
 * @brief TeensyThreads + micro-ROS integration test for Teensy41
 *
 * Creates threads using the beginThreaded() API:
 *   1. micro-ROS task — runs the MicrorosManager state machine
 *   2. blink task — toggles LED to confirm scheduler is running
 *   3. status task — publishes status when connected
 */

#include <Arduino.h>
#include <TeensyThreads.h>

#include "ExampleMicrorosSubsystem.h"
#include "microros_manager_robot.h"

static Subsystem::MicrorosManagerSetup g_mr_setup("microros_test");
static Subsystem::MicrorosManager g_mr(g_mr_setup);

static Subsystem::ExampleSubsystemSetup g_ex_setup("example_sub_test");
static Subsystem::ExampleSubsystem g_ex(g_ex_setup);

static void blink_task(void*) {
  pinMode(LED_BUILTIN, OUTPUT);
  while (true) {
    digitalWriteFast(LED_BUILTIN, HIGH);
    threads.delay(500);
    digitalWriteFast(LED_BUILTIN, LOW);
    threads.delay(500);
  }
}

static void status_task(void*) {
  while (true) {
    if (g_mr.isConnected()) {
      Threads::Scope guard(g_mr.getMutex());
      g_ex.publishStatus("OK");
    }
    threads.delay(1000);
  }
}

void setup() {
  Serial.begin(0);
  if (CrashReport) {
    Serial.print(CrashReport);
    Serial.println();
    Serial.flush();
  }

  Serial.println(PSTR("\r\nTeensyThreads + micro-ROS test.\r\n"));

  g_mr.init();
  g_mr.registerParticipant(&g_ex);

  g_mr.beginThreaded(8192, 3);
  threads.addThread(blink_task, nullptr, 512);
  threads.addThread(status_task, nullptr, 1024);

  Serial.println(PSTR("setup(): threads started."));
  Serial.flush();
}

void loop() { threads.delay(100); }
