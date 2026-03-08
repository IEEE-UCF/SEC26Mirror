/**
 * @file custom-test-robot.cpp
 * @brief Minimal micro-ROS + BNO085 IMU test environment.
 *
 * Stripped-down firmware with only MicrorosManager and ImuSubsystem to isolate
 * and debug BNO085 IMU issues without interference from other subsystems.
 *
 * Hardware required:
 *   Wire1 — BNO085 IMU (0x4B), RST=40, INT=41
 *
 * micro-ROS topics published:
 *   /mcu_robot/imu/data    sensor_msgs/Imu    ~20-33 Hz
 *
 * Serial debug output (USB Serial, 1 Hz):
 *   Quaternion (w,x,y,z), yaw, update/pub counts, micro-ROS state
 */

#include <Arduino.h>
#include <TeensyThreads.h>
#include <microros_manager_robot.h>

#include "BNO085.h"
#include "I2CBusLock.h"
#include "robot/RobotPins.h"
#include "robot/subsystems/ImuSubsystem.h"

using namespace Subsystem;

// --- micro-ROS manager ---
static MicrorosManagerSetup g_mr_setup("microros_manager");
static MicrorosManager g_mr(g_mr_setup);

// --- Wire1: IMU (BNO085) ---
static Drivers::BNO085DriverSetup g_imu_driver_setup("imu_driver", PIN_GYRO_RST,
                                                     Wire1);
static Drivers::BNO085Driver g_imu_driver(g_imu_driver_setup);
static ImuSubsystemSetup g_imu_setup("imu_subsystem", &g_imu_driver);
static ImuSubsystem g_imu(g_imu_setup);

void setup() {
  Serial.begin(0);
  delay(500);
  Serial.println("\r\n=== custom-test-robot: IMU debug ===\r\n");

  // I2C bus mutexes (required for threaded I2C access)
  I2CBus::initLocks();
  Serial.println("[INIT] I2C bus locks initialized");

  // Init subsystems
  g_mr.init();
  Serial.println("[INIT] MicrorosManager OK");

  g_imu.init();
  Serial.println("[INIT] IMU OK");

  // Register IMU as micro-ROS participant
  g_mr.registerParticipant(&g_imu);
  Serial.println("[INIT] IMU registered with micro-ROS (1 participant)");

  // Start threads
  g_mr.beginThreaded(8192, 4);        // micro-ROS agent (highest priority)
  g_imu.beginThreaded(8192, 3, 30);   // IMU ~33 Hz
  Serial.println("[INIT] Threads started — entering main loop");
}

void loop() {
  // Periodic serial debug output (1 Hz)
  static uint32_t last_print_ms = 0;
  uint32_t now = millis();

  if (now - last_print_ms >= 1000) {
    last_print_ms = now;

    float yaw = g_imu.getYaw();
    uint32_t updates = g_imu.diag_update_count_;
    uint32_t pubs = g_imu.diag_pub_count_;
    uint32_t last_upd = g_imu.diag_last_update_ms_;
    bool ros_ok = g_mr.isConnected();

    Serial.printf("[IMU] yaw=%.2f deg  updates=%lu  pubs=%lu  last_upd=%lu ms  uROS=%s\r\n",
                  yaw * 180.0f / PI,
                  updates, pubs, last_upd,
                  ros_ok ? "CONNECTED" : "WAITING");
  }

  delay(100);
}
