/**
 * @file teensy-test-drive-motors.cpp
 * @brief Focused test — PCA9685 motor control with micro-ROS SetMotor service.
 * @date 2026-02-24
 *
 * Hardware expected:
 *   Wire2 (SDA=24, SCL=25) — PCA9685 #1 at 0x41 (motor board), OE=pin 21
 *
 * Each motor uses two consecutive PCA9685 channels:
 *   even channel = PWM (0-4095 duty)
 *   odd  channel = DIR (digital HIGH/LOW)
 *   Motor 0 → ch 0,1;  Motor 1 → ch 2,3; …  Motor 7 → ch 14,15
 *
 * micro-ROS topics published:
 *   /mcu_robot/motor/state   std_msgs/Float32MultiArray   5 Hz
 *
 * micro-ROS services:
 *   /mcu_robot/motor/set     mcu_msgs/srv/SetMotor
 *
 * Usage:
 *   ros2 topic echo /mcu_robot/motor/state
 *   ros2 service call /mcu_robot/motor/set mcu_msgs/srv/SetMotor \
 *       "{index: 0, speed: 0.5}"
 */

#include <Arduino.h>
#include <TeensyThreads.h>
#include <microros_manager_robot.h>

#include "I2CBusLock.h"
#include "PCA9685Manager.h"
#include "robot/RobotConstants.h"
#include "robot/RobotPins.h"
#include "robot/subsystems/MotorManagerSubsystem.h"

using namespace Subsystem;

// --- micro-ROS manager ---
static MicrorosManagerSetup g_mr_setup("motor_test_mr");
static MicrorosManager g_mr(g_mr_setup);

// --- Wire2: PCA9685 manager + motor driver ---
static Robot::PCA9685ManagerSetup g_pca_mgr_setup("pca_manager");
static Robot::PCA9685Manager g_pca_mgr(g_pca_mgr_setup);

static Robot::PCA9685Driver* g_pca_motor =
    g_pca_mgr.createDriver(Robot::PCA9685DriverSetup(
        "pca_motor", I2C_ADDR_MOTOR, DEFAULT_PCA9685_FREQ, Wire2));

// --- Motor manager subsystem (PCA9685 #1, OE = pin 21) ---
static MotorManagerSubsystemSetup g_motor_setup("motor_subsystem", g_pca_motor,
                                                PIN_MOTOR_OE, NUM_MOTORS);
static MotorManagerSubsystem g_motor(g_motor_setup);

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
    Serial.printf("Motor test — agent %s | speeds:",
                  g_mr.isConnected() ? "CONNECTED" : "waiting...");
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
      Serial.printf(" %+.2f", g_motor.getSpeed(i));
    }
    Serial.println();
    threads.delay(2000);
  }
}

void setup() {
  Serial.begin(921600);
  delay(500);
  Serial.println(PSTR("\r\nSEC26 Robot — Drive Motor Test\r\n"));

  // 0. I2C bus mutexes
  I2CBus::initLocks();

  // 1. Init subsystems
  g_mr.init();
  g_pca_mgr.init();
  g_motor.init();

  // 2. Register micro-ROS participant
  g_mr.registerParticipant(&g_motor);

  // 3. Start threads
  //                                      stack  pri  rate(ms)
  g_mr.beginThreaded(8192, 4);                          // ROS agent
  g_motor.beginThreaded(1024, 2, 50);                    // 20 Hz state pub
  threads.addThread(pca_task, nullptr, 512);             // PWM flush @ 50 Hz
  threads.addThread(print_task, nullptr, 1024);
  threads.addThread(blink_task, nullptr, 512);

  Serial.println(PSTR("setup(): drive motor test threads started."));
}

void loop() { threads.delay(100); }
