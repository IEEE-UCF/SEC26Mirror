/**
 * @file teensy-test-microros-freertos.cpp
 * @brief FreeRTOS + micro-ROS integration test for Teensy41
 *
 * Creates FreeRTOS tasks using the beginThreaded() API:
 *   1. micro-ROS task — runs the MicrorosManager state machine
 *   2. blink task — toggles LED to confirm scheduler is running
 *   3. status task — publishes status when connected
 */

#include "arduino_freertos.h"

#include <Arduino.h>

#include <mutex>

#include "ExampleMicrorosSubsystem.h"
#include "microros_manager_robot.h"

static Subsystem::MicrorosManagerSetup g_mr_setup("microros_freertos_test");
static Subsystem::MicrorosManager g_mr(g_mr_setup);

static Subsystem::ExampleSubsystemSetup g_ex_setup("example_sub_test");
static Subsystem::ExampleSubsystem g_ex(g_ex_setup);

static void blink_task(void*) {
  pinMode(LED_BUILTIN, OUTPUT);
  while (true) {
    digitalWriteFast(LED_BUILTIN, HIGH);
    vTaskDelay(pdMS_TO_TICKS(500));
    digitalWriteFast(LED_BUILTIN, LOW);
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

static void status_task(void*) {
  while (true) {
    if (g_mr.isConnected()) {
      std::lock_guard<std::mutex> guard(g_mr.getMutex());
      g_ex.publishStatus("OK");
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void setup() {
  Serial.begin(0);
  if (CrashReport) {
    Serial.print(CrashReport);
    Serial.println();
    Serial.flush();
  }

  Serial.println(PSTR("\r\nFreeRTOS + micro-ROS test. Kernel "
                       tskKERNEL_VERSION_NUMBER "\r\n"));

  g_mr.init();
  g_mr.registerParticipant(&g_ex);

  g_mr.beginThreaded(8192, 3);
  xTaskCreate(blink_task, "blink", 128, nullptr, 2, nullptr);
  xTaskCreate(status_task, "status", 512, nullptr, 2, nullptr);

  Serial.println(PSTR("setup(): starting scheduler..."));
  Serial.flush();

  vTaskStartScheduler();
}

void loop() {}
