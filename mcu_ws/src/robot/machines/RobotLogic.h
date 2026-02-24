/**
 * @file RobotLogic.h
 * @date 12/16/25
 * @author Aldem Pido
 * @brief Main logic for the SEC26 robot — FreeRTOS-based scheduling.
 */

#pragma once

#include <Arduino.h>
#include <microros_manager_robot.h>

#include "../RobotConstants.h"
#include "BNO085.h"
#include "I2CPowerDriver.h"
#include "PCA9685Manager.h"
#include "TOF.h"
#include "arduino_freertos.h"
#include "robot/drive-base/DriveSubsystem.h"
#include "robot/machines/HeartbeatSubsystem.h"
<<<<<<< HEAD
#include "robot/machines/McuSubsystem.h"
#include "robot/machines/RobotManager.h"
=======
>>>>>>> 1efe8ca348ef17c6d21aa1f5e5f0f24367ae9fe0
#include "robot/subsystems/ArmSubsystem.h"
#include "robot/subsystems/BatterySubsystem.h"
#include "robot/subsystems/ImuSubsystem.h"
#include "robot/subsystems/IntakeBridgeSubsystem.h"
#include "robot/subsystems/IntakeSubsystem.h"
#include "robot/subsystems/OLEDSubsystem.h"
#include "robot/subsystems/RCSubsystem.h"
#include "robot/subsystems/SensorSubsystem.h"

using namespace Subsystem;

// --- micro-ROS manager ---
static MicrorosManagerSetup g_mr_setup("microros_manager");
static MicrorosManager g_mr(g_mr_setup);

// --- OLED display (SSD1306 128x64, software SPI) ---
// Serial-terminal style: call g_oled.appendText("line") from any subsystem.
// ROS2 topics: /mcu_robot/lcd/text (append) and /mcu_robot/lcd/scroll (-1/+1).
// To call from another subsystem header, forward-declare:
//   extern Subsystem::OLEDSubsystem g_oled;
//
// Software SPI: MOSI=11, CLK=13, DC=9, RST=3, CS=10
static OLEDSubsystemSetup g_oled_setup("oled_subsystem",
                                       /*mosi*/ 11,
                                       /*clk*/  13,
                                       /*dc*/    9,
                                       /*rst*/   3,
                                       /*cs*/   10);
static OLEDSubsystem g_oled(g_oled_setup);

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

// --- IMU subsystem (BNO085) ---
static Drivers::BNO085DriverSetup g_imu_driver_setup("imu_driver");
static Drivers::BNO085Driver g_imu_driver(g_imu_driver_setup);
static ImuSubsystemSetup g_imu_setup("imu_subsystem", &g_imu_driver);
static ImuSubsystem g_imu(g_imu_setup);

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

// --- Intake subsystem ---
static IntakeSubsystemSetup g_intake_setup("intake_subsystem", /*pwm*/ 3,
                                           /*dir*/ 4, /*ir*/ 5);
static IntakeSubsystem g_intake(g_intake_setup);

// --- Intake bridge subsystem (gear-and-rack for pressure plate) ---
// TODO: confirm rack motor, home switch, and TOF pin numbers with electrical team
static IntakeBridgeSubsystemSetup g_bridge_setup(
    "intake_bridge_subsystem", /*rack_pwm*/ 6, /*rack_dir*/ 7,
    /*home_switch*/ 9, /*tof_xshut*/ 8, /*tof_addr*/ 0x30,
    /*extend_timeout_ms*/ 3000, /*retract_timeout_ms*/ 3000,
    /*duck_detect_threshold_mm*/ 50, /*motor_speed*/ 200);
static IntakeBridgeSubsystem g_bridge(g_bridge_setup);

// --- Drive subsystem ---
// TODO: Replace placeholder pin values with actual hardware wiring config
// static DriveBaseSetup g_drive_base_setup( ... encoder/motor configs ... );
// static DriveSubsystemSetup g_drive_setup("drive_subsystem",
// g_drive_base_setup); static DriveSubsystem g_drive(g_drive_setup);

// --- PCA9685 flush task ---
static void pca_task(void*) {
  while (true) {
    g_pca_mgr.update();
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

// --- Arduino sketch entry points ---
void setup() {
  Serial.begin(0);
  if (CrashReport) {
    Serial.print(CrashReport);
    Serial.println();
    Serial.flush();
  }

  Serial.println(PSTR(
      "\r\nSEC26 Robot — FreeRTOS kernel " tskKERNEL_VERSION_NUMBER "\r\n"));

  // 1. Init all subsystems
  g_mr.init();
  g_oled.init();
  g_hb.init();
  g_battery.init();
  g_sensor.init();
  g_imu.init();
  g_pca_mgr.init();
  g_arm_encoder.init();
  g_arm.init();
  g_rc.init();
  g_intake.init();
  g_bridge.init();
  // g_drive.init();  // TODO: uncomment when DriveSubsystem is configured

  // 2. Register micro-ROS participants
  g_mr.registerParticipant(&g_oled);
  g_mr.registerParticipant(&g_hb);
  g_mr.registerParticipant(&g_battery);
  g_mr.registerParticipant(&g_sensor);
  g_mr.registerParticipant(&g_imu);
  g_mr.registerParticipant(&g_arm);
  g_mr.registerParticipant(&g_rc);
  g_mr.registerParticipant(&g_intake);
  g_mr.registerParticipant(&g_bridge);
  // g_mr.registerParticipant(&g_drive);  // TODO: uncomment when configured

  // 3. Start FreeRTOS tasks
  //                       stackSize  priority  rateMs
  g_mr.beginThreaded(8192, 4);            // highest — ROS agent
  g_imu.beginThreaded(2048, 3, 20);       // 50Hz sensor fusion
  g_oled.beginThreaded(2048, 1, 50);      // 20Hz display refresh
  g_rc.beginThreaded(1024, 3, 5);         // fast IBUS polling
  g_arm.beginThreaded(1024, 2, 20);       // 50Hz movement
  g_battery.beginThreaded(1024, 1, 100);  // 10Hz battery
  g_sensor.beginThreaded(1024, 1, 100);   // 10Hz TOF
  g_hb.beginThreaded(512, 1, 200);        // 5Hz heartbeat
  g_intake.beginThreaded(1024, 2, 20);    // 50Hz intake
  g_bridge.beginThreaded(1024, 2, 20);   // 50Hz intake bridge
  // g_drive.beginThreaded( 2048,     3,        20     );  // 50Hz drive control
  xTaskCreate(pca_task, "pca", 512, nullptr, 2, nullptr);  // 50Hz PWM flush

  Serial.println(PSTR("setup(): starting scheduler..."));
  Serial.flush();

  vTaskStartScheduler();
}

void loop() {}  // never reached
