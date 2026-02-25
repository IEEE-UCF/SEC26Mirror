/**
 * @file teensy-test-all-subsystems.cpp
 * @brief Integration test — wires every robot subsystem to micro-ROS so each
 *        can be observed via `ros2 topic echo` / `ros2 service call`.
 * @date 2026-02-24
 *
 * Hardware expected (see RobotPins.h for pin map):
 *   Wire0  — TCA9548A mux, TCA9555 GPIO, INA219 (mux ch0)
 *   Wire1  — BNO085 IMU
 *   Wire2  — PCA9685 #0 (servos), PCA9685 #1 (motors)
 *   SPI    — SSD1306 OLED
 *   GPIO   — WS2812B LEDs, FlySky RC (Serial8), reset button
 *
 * micro-ROS topics published:
 *   /mcu_robot/heartbeat          std_msgs/String          5 Hz
 *   /mcu_robot/battery_health     mcu_msgs/BatteryHealth   1 Hz
 *   /mcu_robot/imu/data           sensor_msgs/Imu         50 Hz
 *   /mcu_robot/rc                 mcu_msgs/RC             20 Hz
 *   /mcu_robot/tof_distances      std_msgs/Float32Multi.. 10 Hz
 *   /mcu_robot/dip_switches       std_msgs/UInt8           1 Hz
 *   /mcu_robot/buttons            std_msgs/UInt8          10 Hz
 *   /mcu_robot/servo/state        std_msgs/Float32Multi..  5 Hz
 *   /mcu_robot/motor/state        std_msgs/Float32Multi..  5 Hz
 *
 * micro-ROS services:
 *   /mcu_robot/lcd/append         mcu_msgs/srv/LCDAppend
 *   /mcu_robot/servo/set          mcu_msgs/srv/SetServo
 *   /mcu_robot/motor/set          mcu_msgs/srv/SetMotor
 *
 * micro-ROS subscriptions:
 *   /mcu_robot/lcd/scroll         std_msgs/Int8
 *   /mcu_robot/led/set_all        mcu_msgs/LedColor
 */

#include <Arduino.h>
#include <TeensyThreads.h>
#include <microros_manager_robot.h>

// Pin & address constants
#include "robot/RobotConstants.h"
#include "robot/RobotPins.h"

// Drivers
#include "BNO085.h"
#include "I2CBusLock.h"
#include "I2CMuxDriver.h"
#include "I2CPowerDriver.h"
#include "PCA9685Manager.h"
#include "TCA9555Driver.h"
#include "TOF.h"

// Subsystems
#include "robot/machines/HeartbeatSubsystem.h"
#include "robot/subsystems/BatterySubsystem.h"
#include "robot/subsystems/ButtonSubsystem.h"
#include "robot/subsystems/DipSwitchSubsystem.h"
#include "robot/subsystems/ImuSubsystem.h"
#include "robot/subsystems/LEDSubsystem.h"
#include "robot/subsystems/MotorManagerSubsystem.h"
#include "robot/subsystems/OLEDSubsystem.h"
#include "robot/subsystems/RCSubsystem.h"
#include "robot/subsystems/SensorSubsystem.h"
#include "robot/subsystems/ServoSubsystem.h"

using namespace Subsystem;

// ═══════════════════════════════════════════════════════════════════════════
//  Global driver / subsystem instances
// ═══════════════════════════════════════════════════════════════════════════

// --- micro-ROS manager ---
static MicrorosManagerSetup g_mr_setup("microros_manager");
static MicrorosManager g_mr(g_mr_setup);

// --- OLED display (SSD1306 128x64, software SPI) ---
static OLEDSubsystemSetup g_oled_setup("oled_subsystem",
                                       /*mosi*/ PIN_DISP_MOSI,
                                       /*clk*/ PIN_DISP_CLK,
                                       /*dc*/ PIN_DISP_DC,
                                       /*rst*/ PIN_DISP_RST,
                                       /*cs*/ PIN_DISP_CS);
static OLEDSubsystem g_oled(g_oled_setup);

// --- Heartbeat ---
static HeartbeatSubsystemSetup g_hb_setup("heartbeat_subsystem");
static HeartbeatSubsystem g_hb(g_hb_setup);

// --- Wire0: I2C mux (TCA9548A) ---
static Drivers::I2CMuxDriverSetup g_mux_setup("i2c_mux", I2C_ADDR_MUX, Wire);
static Drivers::I2CMuxDriver g_mux(g_mux_setup);

// --- Wire0: GPIO expander (TCA9555) ---
static Drivers::TCA9555DriverSetup g_gpio_setup("gpio_expander", I2C_ADDR_GPIO,
                                                Wire);
static Drivers::TCA9555Driver g_gpio(g_gpio_setup);

// --- Wire0: Battery subsystem (INA219 on mux ch0) ---
static Drivers::I2CPowerDriverSetup g_power_setup("power_driver",
                                                  I2C_ADDR_POWER, &g_mux,
                                                  MUX_CH_BATTERY, Wire,
                                                  SHUNT_RESISTANCE_OHM);
static Drivers::I2CPowerDriver g_power_driver(g_power_setup);
static BatterySubsystemSetup g_battery_setup("battery_subsystem",
                                             &g_power_driver);
static BatterySubsystem g_battery(g_battery_setup);

// --- Wire0: Sensor subsystem (TOF) ---
static Drivers::TOFDriverSetup g_tof_setup("tof_sensor", 500, 0, Wire);
static Drivers::TOFDriver g_tof_driver(g_tof_setup);
static std::vector<Drivers::TOFDriver*> g_tof_drivers = {&g_tof_driver};
static SensorSubsystemSetup g_sensor_setup("sensor_subsystem", g_tof_drivers);
static SensorSubsystem g_sensor(g_sensor_setup);

// --- Wire1: IMU (BNO085) ---
static Drivers::BNO085DriverSetup g_imu_driver_setup("imu_driver", PIN_GYRO_RST,
                                                     Wire1);
static Drivers::BNO085Driver g_imu_driver(g_imu_driver_setup);
static ImuSubsystemSetup g_imu_setup("imu_subsystem", &g_imu_driver);
static ImuSubsystem g_imu(g_imu_setup);

// --- Wire2: PCA9685 manager (servo + motor boards) ---
static Robot::PCA9685ManagerSetup g_pca_mgr_setup("pca_manager");
static Robot::PCA9685Manager g_pca_mgr(g_pca_mgr_setup);

static Robot::PCA9685Driver* g_pca_servo =
    g_pca_mgr.createDriver(Robot::PCA9685DriverSetup(
        "pca_servo", I2C_ADDR_SERVO, DEFAULT_PCA9685_FREQ, Wire2));
static Robot::PCA9685Driver* g_pca_motor =
    g_pca_mgr.createDriver(Robot::PCA9685DriverSetup(
        "pca_motor", I2C_ADDR_MOTOR, DEFAULT_PCA9685_FREQ, Wire2));

// --- RC subsystem (FlySky, Serial8 RX = pin 34) ---
static RCSubsystemSetup g_rc_setup("rc_subsystem", &Serial8);
static RCSubsystem g_rc(g_rc_setup);

// --- DIP switch subsystem (TCA9555 port 0) ---
static DipSwitchSubsystemSetup g_dip_setup("dip_switch_subsystem", &g_gpio);
static DipSwitchSubsystem g_dip(g_dip_setup);

// --- Button subsystem (TCA9555 port 1) ---
static ButtonSubsystemSetup g_btn_setup("button_subsystem", &g_gpio);
static ButtonSubsystem g_btn(g_btn_setup);

// --- LED subsystem (WS2812B) ---
static LEDSubsystemSetup g_led_setup("led_subsystem", PIN_RGB_LEDS,
                                     NUM_RGB_LEDS);
static LEDSubsystem g_led(g_led_setup);

// --- Servo subsystem (PCA9685 #0, OE = pin 20) ---
static ServoSubsystemSetup g_servo_setup("servo_subsystem", g_pca_servo,
                                         PIN_SERVO_OE, NUM_SERVOS);
static ServoSubsystem g_servo(g_servo_setup);

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

// ═══════════════════════════════════════════════════════════════════════════
//  Arduino entry points
// ═══════════════════════════════════════════════════════════════════════════

void setup() {
  Serial.begin(0);
  if (CrashReport) {
    Serial.print(CrashReport);
    Serial.println();
    Serial.flush();
  }

  Serial.println(
      PSTR("\r\nSEC26 Robot — All Subsystems Test (TeensyThreads)\r\n"));

  // 0. I2C bus mutexes
  I2CBus::initLocks();

  // 1. Mux reset (if wired)
  pinMode(PIN_MUX_RESET, OUTPUT);
  digitalWrite(PIN_MUX_RESET, HIGH);  // active-LOW reset — release

  // 2. Init subsystems
  g_mr.init();
  g_oled.init();
  g_hb.init();
  g_mux.init();
  g_gpio.init();
  g_battery.init();
  g_sensor.init();
  g_imu.init();
  g_pca_mgr.init();
  g_rc.init();
  g_dip.init();
  g_btn.init();
  g_led.init();
  g_servo.init();
  g_motor.init();

  // 2a. Startup LED flash (green)
  g_led.setAll(0, 32, 0);

  // 2b. Wire battery → OLED status line
  g_battery.setOLED(&g_oled);

  // 3. Register micro-ROS participants
  g_mr.registerParticipant(&g_oled);
  g_mr.registerParticipant(&g_hb);
  g_mr.registerParticipant(&g_battery);
  g_mr.registerParticipant(&g_sensor);
  g_mr.registerParticipant(&g_imu);
  g_mr.registerParticipant(&g_rc);
  g_mr.registerParticipant(&g_dip);
  g_mr.registerParticipant(&g_btn);
  g_mr.registerParticipant(&g_led);
  g_mr.registerParticipant(&g_servo);
  g_mr.registerParticipant(&g_motor);

  // 4. Start threaded tasks
  //                                 stack  pri   rate(ms)
  g_mr.beginThreaded(8192, 4);                // ROS agent
  g_imu.beginThreaded(2048, 3, 20);           // 50 Hz
  g_rc.beginThreaded(1024, 3, 5);             // IBUS polling
  g_servo.beginThreaded(1024, 2, 50);         // 20 Hz state pub
  g_motor.beginThreaded(1024, 2, 50);         // 20 Hz state pub
  g_oled.beginThreaded(2048, 1, 50);          // 20 Hz display
  g_battery.beginThreaded(1024, 1, 100);      // 10 Hz
  g_sensor.beginThreaded(1024, 1, 100);       // 10 Hz TOF
  g_dip.beginThreaded(1024, 1, 500);           // 2 Hz
  g_btn.beginThreaded(1024, 1, 20);            // 50 Hz
  g_led.beginThreaded(1024, 1, 50);            // 20 Hz
  g_hb.beginThreaded(1024, 1, 200);            // 5 Hz
  threads.addThread(pca_task, nullptr, 1024);  // PWM flush

}

void loop() { threads.delay(100); }
