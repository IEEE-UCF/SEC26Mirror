/**
 * @file teensy-test-arm-servos.cpp
 * @brief Focused test — PCA9685 servo control with micro-ROS SetServo service.
 * @date 2026-02-24
 *
 * Hardware expected:
 *   Wire2 (SDA=24, SCL=25) — PCA9685 #0 at 0x40 (servo board), OE=pin 20
 *
 * micro-ROS topics published:
 *   /mcu_robot/servo/state   std_msgs/Float32MultiArray   5 Hz
 *
 * micro-ROS services:
 *   /mcu_robot/servo/set     mcu_msgs/srv/SetServo
 *
 * Usage:
 *   ros2 topic echo /mcu_robot/servo/state
 *   ros2 service call /mcu_robot/servo/set mcu_msgs/srv/SetServo \
 *       "{index: 0, angle: 90.0}"
 */

#include <Arduino.h>
#include <TeensyThreads.h>
#include <microros_manager_robot.h>

#include "I2CBusLock.h"
#include "PCA9685Manager.h"
#include "robot/RobotConstants.h"
#include "robot/RobotPins.h"
#include "robot/subsystems/ServoSubsystem.h"

using namespace Subsystem;

// --- micro-ROS manager ---
static MicrorosManagerSetup g_mr_setup("servo_test_mr");
static MicrorosManager g_mr(g_mr_setup);

// --- Wire2: PCA9685 manager + servo driver ---
static Robot::PCA9685ManagerSetup g_pca_mgr_setup("pca_manager");
static Robot::PCA9685Manager g_pca_mgr(g_pca_mgr_setup);

static Robot::PCA9685Driver* g_pca_servo =
    g_pca_mgr.createDriver(Robot::PCA9685DriverSetup(
        "pca_servo", I2C_ADDR_SERVO, DEFAULT_PCA9685_FREQ, Wire2));

// --- Servo subsystem (PCA9685 #0, OE = pin 20) ---
static ServoSubsystemSetup g_servo_setup("servo_subsystem", g_pca_servo,
                                         PIN_SERVO_OE, NUM_SERVOS);
static ServoSubsystem g_servo(g_servo_setup);

// --- PCA9685 flush task ---
static void pca_task(void*) {
  while (true) {
    g_pca_mgr.update();
    threads.delay(20);
  }
}

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
    Serial.printf("Servo test — agent %s | angles:",
                  g_mr.isConnected() ? "CONNECTED" : "waiting...");
    for (uint8_t i = 0; i < NUM_SERVOS; i++) {
      Serial.printf(" %.0f", g_servo.getAngle(i));
    }
    Serial.println();
    threads.delay(2000);
  }
}

void setup() {
  Serial.begin(921600);
  delay(500);
  Serial.println(PSTR("\r\nSEC26 Robot — Arm Servo Test\r\n"));

  // 0. I2C bus mutexes
  I2CBus::initLocks();

  // 1. Init subsystems
  g_mr.init();
  g_pca_mgr.init();
  g_servo.init();

  // 2. Register micro-ROS participant
  g_mr.registerParticipant(&g_servo);

  // 3. Start threads
  //                                     stack  pri  rate(ms)
  g_mr.beginThreaded(8192, 4);                         // ROS agent
  g_servo.beginThreaded(1024, 2, 50);                   // 20 Hz state pub
  threads.addThread(pca_task, nullptr, 512);            // PWM flush @ 50 Hz
  threads.addThread(print_task, nullptr, 1024);
  threads.addThread(blink_task, nullptr, 512);

  Serial.println(PSTR("setup(): arm servo test threads started."));
}

void loop() { threads.delay(100); }
