/**
 * @file RobotLogic.h
 * @date 12/16/25
 * @author Aldem Pido
 * @brief Main logic for the SEC26 robot — TeensyThreads-based scheduling.
 *
 * I2C bus layout:
 *   Wire  (Wire0) — TCA9548A mux (0x70), TCA9555 GPIO expander (0x20),
 *                   INA219 power sensor (0x40, behind mux ch0)
 *   Wire1         — BNO085 IMU (0x4B)
 *   Wire2         — PCA9685 PWM driver #0 (0x40 servos), #1 (0x41 motors)
 */

#pragma once

#include <Arduino.h>
#include <SPI.h>
#include <TeensyThreads.h>
#include <microros_manager_robot.h>

#include <math_utils.h>

#include "../RobotConfig.h"
#include "DebugLog.h"
#include "../RobotConstants.h"
#include "../RobotPins.h"
#include "BNO085.h"
#include "I2CBusLock.h"
#include "I2CMuxDriver.h"
#include "I2CPowerDriver.h"
#include "PCA9685Manager.h"
#include "TCA9555Driver.h"
#include "TOF.h"
#include "UWBDriver.h"
#include "robot/drive-base/DriveSubsystem.h"
#include "robot/machines/HeartbeatSubsystem.h"
#include "robot/subsystems/ArmSubsystem.h"
#include "robot/subsystems/BatterySubsystem.h"
#include "robot/subsystems/ButtonSubsystem.h"
#include "robot/subsystems/DeploySubsystem.h"
#include "robot/subsystems/DipSwitchSubsystem.h"
#include "robot/subsystems/ImuSubsystem.h"
#include "robot/subsystems/IntakeBridgeSubsystem.h"
#include "robot/subsystems/IntakeSubsystem.h"
#include "robot/subsystems/LEDSubsystem.h"
#include "robot/subsystems/EncoderSubsystem.h"
#include "robot/subsystems/MotorManagerSubsystem.h"
#include "robot/subsystems/OLEDSubsystem.h"
#include "robot/subsystems/RCSubsystem.h"
#include "robot/subsystems/ResetSubsystem.h"
#include "robot/subsystems/SensorSubsystem.h"
#include "robot/subsystems/CrankSubsystem.h"
#include "robot/subsystems/KeypadSubsystem.h"
#include "robot/subsystems/ServoSubsystem.h"
#include "robot/subsystems/UWBSubsystem.h"

using namespace Subsystem;

// ═══════════════════════════════════════════════════════════════════════════
//  Global driver / subsystem instances
// ═══════════════════════════════════════════════════════════════════════════

// --- micro-ROS manager ---
static MicrorosManagerSetup g_mr_setup("microros_manager");
static MicrorosManager g_mr(g_mr_setup);

// --- OLED display (SSD1306 128x64, hardware SPI1) ---
// Hardware SPI1: MOSI=26, SCK=27, DC=37, RST=33, CS=38 (per RobotPins.h)
static OLEDSubsystemSetup g_oled_setup("oled_subsystem", &SPI1,
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
        "pca_motor", I2C_ADDR_MOTOR, MOTOR_PCA9685_FREQ, Wire2));

// --- Arm subsystem ---
static Drivers::EncoderDriverSetup g_encoder_setup("arm_encoder", /*pin1*/ 1,
                                                   /*pin2*/ 2);
static Drivers::EncoderDriver g_arm_encoder(g_encoder_setup);
static ArmSubsystemSetup g_arm_setup("arm_subsystem", g_pca_servo,
                                     &g_arm_encoder);
static ArmSubsystem g_arm(g_arm_setup);

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

// --- Servo subsystem (PCA9685 #0) ---
static ServoSubsystemSetup g_servo_setup("servo_subsystem", g_pca_servo,
                                         PIN_SERVO_OE, NUM_SERVOS);
static ServoSubsystem g_servo(g_servo_setup);

// --- Crank subsystem (servo on PCA9685 #0, channel 0) ---
static CrankSubsystemSetup g_crank_setup("crank_subsystem", &g_servo,
                                          CRANK_SERVO_IDX);
static CrankSubsystem g_crank(g_crank_setup);

// --- Motor manager subsystem (PCA9685 #1) ---
static MotorManagerSubsystemSetup g_motor_setup("motor_subsystem", g_pca_motor,
                                                PIN_MOTOR_OE, NUM_MOTORS);
static MotorManagerSubsystem g_motor(g_motor_setup);

// --- Encoder subsystem (QTimer hardware FG pulse counting, pins 2-9) ---
static Encoders::QTimerEncoder g_qtimer_encoder;
static EncoderSubsystemSetup g_encoder_sub_setup("encoder_subsystem",
                                                  &g_qtimer_encoder, &g_motor);
static EncoderSubsystem g_encoder_sub(g_encoder_sub_setup);

// --- UWB subsystem (DW3000 tag, SPI0) ---
static Drivers::UWBDriverSetup g_uwb_driver_setup("uwb_driver",
                                                  Drivers::UWBMode::TAG,
                                                  ROBOT_UWB_TAG_ID, PIN_UWB_CS);
static Drivers::UWBDriver g_uwb_driver(g_uwb_driver_setup);
static UWBSubsystemSetup g_uwb_setup("uwb_subsystem", &g_uwb_driver,
                                     ROBOT_UWB_TOPIC);
static UWBSubsystem g_uwb(g_uwb_setup);

// --- Intake subsystem ---
static IntakeSubsystemSetup g_intake_setup("intake_subsystem", /*pwm*/ 3,
                                           /*dir*/ 4, /*ir*/ 5);
static IntakeSubsystem g_intake(g_intake_setup);

// --- Intake bridge subsystem (gear-and-rack for pressure plate) ---
// TODO: confirm rack motor, home switch, and TOF pin numbers with electrical
static IntakeBridgeSubsystemSetup g_bridge_setup(
    "intake_bridge_subsystem", /*rack_pwm*/ 6, /*rack_dir*/ 7,
    /*home_switch*/ 9, /*tof_xshut*/ 8, /*tof_addr*/ 0x30,
    /*extend_timeout_ms*/ 3000, /*retract_timeout_ms*/ 3000,
    /*duck_detect_threshold_mm*/ 50, /*motor_speed*/ 200);
static IntakeBridgeSubsystem g_bridge(g_bridge_setup);

// --- Deploy subsystem (button-triggered deployment) ---
static DeploySubsystemSetup g_deploy_setup("deploy_subsystem", &g_btn, &g_dip,
                                           &g_led, &g_oled);
static DeploySubsystem g_deploy(g_deploy_setup);

// --- Reset subsystem (micro-ROS service to reset all subsystems) ---
static ResetSubsystemSetup g_reset_setup("reset_subsystem");
static ResetSubsystem g_reset(g_reset_setup);

// --- Drive subsystem (tank drive, 2 motors, 4 omniwheels) ---
static Drive::TankDriveLocalizationSetup g_loc_setup(
    "drive_localization",
    RobotConfig::TRACK_WIDTH_M,
    RobotConfig::WHEEL_DIAMETER_M,
    RobotConfig::RAW_TICKS_PER_REVOLUTION,
    RobotConfig::GEAR_RATIO,
    RobotConfig::START_X, RobotConfig::START_Y, RobotConfig::START_THETA);

static DriveSubsystemSetup g_drive_setup(
    "drive_subsystem", &g_motor, &g_encoder_sub, &g_imu, g_loc_setup);

// Configure drive subsystem control parameters from RobotConfig
static void configureDriveSetup() {
  using namespace RobotConfig;

  // Channel mapping
  g_drive_setup.leftMotorIdx = LEFT_MOTOR_IDX;
  g_drive_setup.rightMotorIdx = RIGHT_MOTOR_IDX;
  g_drive_setup.leftEncoderIdx = LEFT_ENCODER_IDX;
  g_drive_setup.rightEncoderIdx = RIGHT_ENCODER_IDX;
  g_drive_setup.leftEncoderInverted = LEFT_ENCODER_INVERTED;
  g_drive_setup.rightEncoderInverted = RIGHT_ENCODER_INVERTED;

  // Wheel velocity PID
  auto makePID = []() {
    PIDController::Config c;
    c.gains.kp = WHEEL_PID_KP;
    c.gains.ki = WHEEL_PID_KI;
    c.gains.kd = WHEEL_PID_KD;
    c.limits.out_min = WHEEL_PID_OUT_MIN;
    c.limits.out_max = WHEEL_PID_OUT_MAX;
    c.limits.i_min = WHEEL_PID_I_MIN;
    c.limits.i_max = WHEEL_PID_I_MAX;
    c.dmode = PIDController::DerivativeMode::OnMeasurement;
    c.d_filter_alpha = WHEEL_PID_D_FILTER;
    c.conditional_integration = true;
    return c;
  };
  g_drive_setup.leftPID = makePID();
  g_drive_setup.rightPID = makePID();

  // Linear S-curve motion profile
  g_drive_setup.linearProfile.limits.v_max = MAX_LINEAR_VEL_MPS;
  g_drive_setup.linearProfile.limits.a_max = MAX_LINEAR_ACCEL_MPS2;
  g_drive_setup.linearProfile.limits.d_max = MAX_LINEAR_ACCEL_MPS2;
  g_drive_setup.linearProfile.limits.j_max = MAX_LINEAR_JERK_MPS3;

  // Angular S-curve motion profile
  g_drive_setup.angularProfile.limits.v_max = MAX_ANGULAR_VEL_RADPS;
  g_drive_setup.angularProfile.limits.a_max = MAX_ANGULAR_ACCEL_RADPS2;
  g_drive_setup.angularProfile.limits.d_max = MAX_ANGULAR_ACCEL_RADPS2;
  g_drive_setup.angularProfile.limits.j_max = MAX_ANGULAR_JERK_RADPS3;

  // Trajectory controller
  g_drive_setup.trajConfig.lookahead_dist = TRAJ_LOOKAHEAD_M;
  g_drive_setup.trajConfig.cruise_v = TRAJ_CRUISE_V_MPS;
  g_drive_setup.trajConfig.max_v = MAX_LINEAR_VEL_MPS;
  g_drive_setup.trajConfig.max_w = MAX_ANGULAR_VEL_RADPS;
  g_drive_setup.trajConfig.slowdown_dist = TRAJ_SLOWDOWN_M;
  g_drive_setup.trajConfig.min_v_near_goal = TRAJ_MIN_V_MPS;
  g_drive_setup.trajConfig.pos_tol = TRAJ_POS_TOL_M;
  g_drive_setup.trajConfig.heading_tol = TRAJ_HEADING_TOL_RAD;
  g_drive_setup.trajConfig.k_heading = TRAJ_K_HEADING;
  g_drive_setup.trajConfig.advance_tol = TRAJ_ADVANCE_TOL_M;

  // Limits
  g_drive_setup.maxLinearVel = MAX_LINEAR_VEL_MPS;
  g_drive_setup.maxAngularVel = MAX_ANGULAR_VEL_RADPS;

  // Pose drive gains
  g_drive_setup.poseKLinear = POSE_K_LINEAR;
  g_drive_setup.poseKAngular = POSE_K_ANGULAR;
  g_drive_setup.poseDistTol = POSE_DIST_TOL_M;

  // Safety
  g_drive_setup.commandTimeoutMs = COMMAND_TIMEOUT_MS;
}

static DriveSubsystem g_drive(g_drive_setup);

// --- Keypad subsystem (servo on PCA9685 #0, channel 1 + drive press) ---
static KeypadSubsystemSetup g_keypad_setup("keypad_subsystem", &g_servo,
                                            KEYPAD_SERVO_IDX, &g_drive);
static KeypadSubsystem g_keypad(g_keypad_setup);

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
  DEBUG_BEGIN();
  if (CrashReport) {
    Serial.print(CrashReport);
    DEBUG_PRINT(CrashReport);
    Serial.println();
    Serial.flush();
    DEBUG_FLUSH();
  }

  Serial.println(PSTR("\r\nSEC26 Robot — TeensyThreads\r\n"));
  DEBUG_PRINTLN("\r\n=== SEC26 Robot — Debug Console (SerialUSB1) ===\r\n");

  // 0. I2C bus mutexes
  I2CBus::initLocks();
  DEBUG_PRINTLN("[INIT] I2C bus locks initialized");

  // 1. Mux reset (if wired)
  pinMode(PIN_MUX_RESET, OUTPUT);
  digitalWrite(PIN_MUX_RESET, HIGH);

  // 2. Init subsystems
  DEBUG_PRINTLN("[INIT] --- Subsystem initialization ---");
  g_mr.init();           DEBUG_PRINTLN("[INIT] MicrorosManager OK");
  g_oled.init();         DEBUG_PRINTLN("[INIT] OLED OK");
  g_hb.init();           DEBUG_PRINTLN("[INIT] Heartbeat OK");
  g_mux.init();          DEBUG_PRINTLN("[INIT] I2C Mux OK");
  g_gpio.init();         DEBUG_PRINTLN("[INIT] GPIO Expander OK");
  g_battery.init();      DEBUG_PRINTLN("[INIT] Battery OK");
  g_sensor.init();       DEBUG_PRINTLN("[INIT] Sensor (TOF) OK");
  g_imu.init();          DEBUG_PRINTLN("[INIT] IMU OK");
  g_pca_mgr.init();      DEBUG_PRINTLN("[INIT] PCA9685 Manager OK");
  g_arm_encoder.init();  DEBUG_PRINTLN("[INIT] Arm Encoder OK");
  g_arm.init();          DEBUG_PRINTLN("[INIT] Arm OK");
  g_rc.init();           DEBUG_PRINTLN("[INIT] RC Receiver OK");
  g_dip.init();          DEBUG_PRINTLN("[INIT] DIP Switch OK");
  g_btn.init();          DEBUG_PRINTLN("[INIT] Buttons OK");
  g_led.init();          DEBUG_PRINTLN("[INIT] LEDs OK");
  g_servo.init();        DEBUG_PRINTLN("[INIT] Servo OK");
  g_crank.init();        DEBUG_PRINTLN("[INIT] Crank OK");
  g_motor.init();        DEBUG_PRINTLN("[INIT] Motor Manager OK");
  g_encoder_sub.init();  DEBUG_PRINTLN("[INIT] Encoder OK");
  g_intake.init();       DEBUG_PRINTLN("[INIT] Intake OK");
  g_bridge.init();       DEBUG_PRINTLN("[INIT] Intake Bridge OK");
  g_deploy.init();       DEBUG_PRINTLN("[INIT] Deploy OK");
  SPI.begin();
  g_uwb.init();          DEBUG_PRINTLN("[INIT] UWB OK");
  g_uwb_driver.setTargetAnchors(ROBOT_UWB_ANCHOR_IDS, ROBOT_UWB_NUM_ANCHORS);
  configureDriveSetup();
  g_drive.init();        DEBUG_PRINTLN("[INIT] Drive OK");
  g_keypad.init();       DEBUG_PRINTLN("[INIT] Keypad OK");

  // 2a. Clear LEDs on startup
  g_led.setAll(0, 0, 0);
  FastLED.show();

  // 2b. OLED startup debug info
  g_oled.appendText("SEC26 Robot v1.0");
  g_oled.appendText("Subsystems OK");
  g_oled.appendText("Waiting for uROS...");

  // 2c. Wire battery → OLED status line
  g_battery.setOLED(&g_oled);

  // 3. Register micro-ROS participants
  DEBUG_PRINTLN("[INIT] --- Registering micro-ROS participants ---");
  g_mr.registerParticipant(&g_oled);
  g_mr.registerParticipant(&g_hb);
  g_mr.registerParticipant(&g_battery);
  g_mr.registerParticipant(&g_sensor);
  g_mr.registerParticipant(&g_imu);
  g_mr.registerParticipant(&g_arm);
  g_mr.registerParticipant(&g_rc);
  g_mr.registerParticipant(&g_dip);
  g_mr.registerParticipant(&g_btn);
  g_mr.registerParticipant(&g_led);
  g_mr.registerParticipant(&g_servo);
  g_mr.registerParticipant(&g_crank);
  g_mr.registerParticipant(&g_motor);
  g_mr.registerParticipant(&g_encoder_sub);
  g_mr.registerParticipant(&g_uwb);
  g_mr.registerParticipant(&g_intake);
  g_mr.registerParticipant(&g_bridge);
  g_mr.registerParticipant(&g_deploy);
  g_mr.registerParticipant(&g_reset);
  g_mr.registerParticipant(&g_drive);
  g_mr.registerParticipant(&g_keypad);

  // 3a. Register subsystems as reset targets
  g_reset.addTarget(&g_motor);
  g_reset.addTarget(&g_servo);
  g_reset.addTarget(&g_encoder_sub);
  g_reset.addTarget(&g_arm);
  g_reset.addTarget(&g_intake);
  g_reset.addTarget(&g_bridge);
  g_reset.addTarget(&g_led);
  g_reset.addTarget(&g_drive);
  g_reset.addTarget(&g_crank);
  g_reset.addTarget(&g_keypad);
  DEBUG_PRINTF("[INIT] Registered %d participants\n", 21);

  // 4. Start threaded tasks
  DEBUG_PRINTLN("[INIT] --- Starting threads ---");
  //                                 stack  pri   rate(ms)
  g_mr.beginThreaded(8192, 4);       // ROS agent
  g_imu.beginThreaded(2048, 3, 10);  // 100 Hz
  // NOTE: RC is polled from loop() — IBusBM NOTIMER mode requires main thread
  g_servo.beginThreaded(1024, 2, 25);     // 40 Hz state pub
  g_motor.beginThreaded(1024, 2, 1);      // 1000 Hz — NFPShop reverse-pulse
  g_encoder_sub.beginThreaded(1024, 2, 20);  // 50 Hz encoder reading
  g_oled.beginThreaded(2048, 1, 25);      // 40 Hz display
  g_battery.beginThreaded(1024, 1, 100);  // 10 Hz
  g_sensor.beginThreaded(1024, 1, 100);   // 10 Hz TOF
  g_dip.beginThreaded(1024, 1, 500);      // 2 Hz
  g_btn.beginThreaded(1024, 1, 20);       // 50 Hz
  g_led.beginThreaded(1024, 1, 50);       // 20 Hz
  g_hb.beginThreaded(1024, 1, 200);       // 5 Hz
  g_uwb.beginThreaded(2048, 2, 50);       // 20 Hz UWB ranging
  g_arm.beginThreaded(1024, 2, 20);       // 50 Hz arm
  g_intake.beginThreaded(1024, 2, 20);    // 50 Hz intake
  g_deploy.beginThreaded(1024, 1, 20);    // 50 Hz deploy button
  g_crank.beginThreaded(1024, 1, 50);     // 20 Hz crank
  g_keypad.beginThreaded(1024, 1, 50);   // 20 Hz keypad
  g_drive.beginThreaded(4096, 3, 20);     // 50 Hz drive control
  threads.addThread(pca_task, nullptr, 1024);  // 50 Hz PWM flush
  DEBUG_PRINTLN("[INIT] All threads started — entering main loop");
}

// RC polled from main loop — IBusBM NOTIMER doesn't work from TeensyThreads
void loop() {
  g_rc.update();

  // RC manual drive — enabled when DIP switch 5 is OFF (bit 5 = 0)
  if (!(g_dip.getState() & (1 << 5))) {
    const auto& rc = g_rc.getData();
    bool swa_high = rc.channels[6] > 0;  // SWA = motor enable

    if (swa_high) {
      // Normalize LY (ch1) and RX (ch2) from [-255, 255] to [-1.0, 1.0]
      float ly = static_cast<float>(rc.channels[1]) / 255.0f;
      float rx = static_cast<float>(rc.channels[2]) / 255.0f;

      // Dead zone (ignore small stick noise)
      if (ly > -0.05f && ly < 0.05f) ly = 0.0f;
      if (rx > -0.05f && rx < 0.05f) rx = 0.0f;

      // Arcade drive mix: left = ly + rx, right = ly - rx
      float left = secbot::utils::clamp(ly + rx, -1.0f, 1.0f);
      float right = secbot::utils::clamp(ly - rx, -1.0f, 1.0f);

      g_drive.manualDrive(left, right);
    } else {
      // SWA off — stop if we were in manual mode
      if (g_drive.getMode() == Subsystem::DriveMode::MANUAL) {
        g_drive.stop();
      }
    }
  } else {
    // DIP 5 ON — not in RC mode; stop if we were in manual
    if (g_drive.getMode() == Subsystem::DriveMode::MANUAL) {
      g_drive.stop();
    }
  }

  delay(5);
}
