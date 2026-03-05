# Project Status — SEC26 Robot

**Last Updated:** 2026-03-03
**Branch:** `master`
**Latest master:** `27afe33` (Merge feat/encoders-new #173)

---

## Maturity Summary

### MCU Subsystems (Teensy41 Robot Firmware)

| Subsystem | Status | Notes |
|-----------|--------|-------|
| MicrorosManager | Mature | 20 participants registered, 24 executor handles, MAX_PARTICIPANTS=32 |
| BatterySubsystem | Mature | INA219, OLED status line, getVoltage/getCurrentA |
| ImuSubsystem | Mature | BNO085, gyro calibration, getYaw accessor |
| RCSubsystem | Mature | FlySky IBUS, main-loop polling, RC override via DIP 1 |
| OLEDSubsystem | Mature | Terminal + debug dashboard mode (DIP 6), lcd/append + lcd/scroll subscriptions |
| SensorSubsystem | Implemented | Single VL53L0X TOF |
| ServoSubsystem | Mature | PCA9685, SetServo service, 8 channels |
| MotorManagerSubsystem | Mature | PCA9685, 1kHz reverse-pulse, SetMotor service, 8 channels |
| EncoderSubsystem | Implemented | QTimer hardware FG counting, 50Hz publish, pins 2-9 (pin 8 disabled) |
| DriveSubsystem | Implemented | Tank drive, PID, S-curve profiles, trajectory controller, RC manual mode, pose drive |
| DipSwitchSubsystem | Mature | 8-bit named constants, isSwitchOn(), conditional behaviors |
| ButtonSubsystem | Mature | TCA9555 port 1, interrupt-ready |
| LEDSubsystem | Mature | WS2812B (5 LEDs), LedColor subscription |
| UWBSubsystem | Implemented | DW3000 tag ID=13, conditional on DIP 2 |
| ArmSubsystem | Implemented | Servo + encoder, kinematics |
| IntakeSubsystem | Implemented | Linear rail + spinning motor, limit switch buttons, state/command topics |
| CrankSubsystem | Implemented | Servo channel 0, state/command topics |
| KeypadSubsystem | Implemented | Servo channel 1 + drive-to-press, state/command topics |
| DeploySubsystem | Implemented | Button-triggered, DIP 8 target select, trigger/status topics |
| ResetSubsystem | Implemented | micro-ROS Reset service, 9 reset targets |
| HeartbeatSubsystem | Mature | 5 Hz heartbeat string |
| MiniRobotSubsystem | Not Active | Header exists but NOT instantiated in RobotLogic.h |

### MCU Subsystems (Other Targets)

| Target | Status | Notes |
|--------|--------|-------|
| Beacons (x3) | Implemented | ESP32 + DW3000, micro-ROS WiFi UDP, IDs 10/11/12 |
| Minibot | Scaffolding | Seeed XIAO ESP32-S3, pin assignments are TODOs, MPU6050 + DW3000 + drive |
| Drone | Implemented | Flight controller, BNO085 IMU, VL53L1X height, IR, DW3000 UWB (ESP32) |
| Field Elements (x6) | Implemented | ESP-NOW communication (button, crank, earth, keypad, pressure, controller) |

### ROS2 Packages

| Package | Version | Status | Notes |
|---------|---------|--------|-------|
| mcu_msgs | 1.0.0 | Mature | 20 msgs, 6 srvs — shared MCU<>ROS2 interface |
| secbot_msgs | 0.1.0 | Implemented | 3 msgs, 2 actions (NavigatePath, ApproachTarget), 2 srvs (SetTask, StartStop) |
| secbot_autonomy | 0.0.1 | Implemented | 7 task state machines, 50Hz orchestration, mission node |
| secbot_navigation | 0.0.1 | Implemented | D* Lite + pure pursuit, yaml-cpp config, arena layout |
| secbot_vision | 0.1.0 | Implemented | HSV duck detection (Python), antenna LED detection, odom TF broadcaster |
| secbot_fusion | 0.0.1 | Scaffolding | DriveBase -> Odometry pass-through, ready for EKF |
| secbot_uwb | 0.1.0 | Implemented | Trilateration, outlier rejection, Eigen, 450+ line README |
| secbot_sim | 0.0.1 | Implemented | Gazebo Harmonic + MCU subsystem emulator, arena world + SDF models |
| secbot_health | 0.0.1 | Implemented | MCU heartbeat watchdog |
| secbot_bridge_i2c | 0.0.1 | Implemented | I2C bridge + packet codec + rate limiter + fake Teensy for sim |
| secbot_deploy | 0.0.1 | Implemented | Deployment trigger bridge (MCU button -> orchestrator) |
| secbot_tf | 0.0.1 | Implemented | Static transform configuration (frames.yaml) |
| my_robot_description | 0.0.1 | Implemented | URDF + 23 STL drivetrain meshes for RViz visualization |

### Native Test Suites

| Suite | Tests | Status |
|-------|-------|--------|
| test-math-vector2d | ~30 | Passing |
| test-math-pose2d | ~25 | Passing |
| test-math-pose3d | ~20 | Passing |
| test-control-pid | ~40 | Passing |
| test-control-arm-kinematics | ~20 | Passing |
| test-control-trapezoidal-motion-profile | ~25 | Passing |
| test-control-scurve-motion-profile | ~25 | Passing |
| test-control-trajectory-controller | ~30 | Passing |
| test-drive-tankdrivelocalization | ~20 | Passing |
| test-utils-filters | ~10 | Passing |
| test-utils-units | ~10 | Passing |
| **Total** | **253+** | **All passing** |

### CI/CD Workflows

| Workflow | Trigger | Status |
|----------|---------|--------|
| `test-mcu.yml` | PR touching mcu_ws/mcu_msgs | Active — runs 11 native test suites (253+ tests) |
| `platformio-robot.yml` | PR touching mcu_ws/mcu_msgs | Active — builds all default PIO envs + uploads artifacts |
| `build-ws.yml` | PR/push to master (ros2_ws) | Active — Docker image + colcon build with caching |
| `commit-linter.yml` | All PRs | Active — enforces Conventional Commits |
| `prod-push.yml` | Push to master | Active — force-pushes master -> prod |
| `auto-format.yml` | PR labeled `format` OR push to master | Active — clang-format auto-commit |
| `test-multiarch.yml` | Manual dispatch | Active — multi-architecture runner test |

---

## Hardware Drivers

| Driver | IC | Bus | Library |
|--------|----|-----|---------|
| BNO085Driver | BNO085 | Wire1 (I2C, 0x4B) | lib/robot/ |
| TCA9555Driver | TCA9555 | Wire0 (I2C, 0x20) | lib/robot/ |
| PCA9685Driver | PCA9685 | Wire2 (I2C, 0x40/0x41) | lib/robot/ |
| PCA9685Manager | — | Wire2 (manages PCA9685 boards) | lib/robot/ |
| I2CMuxDriver | TCA9548A | Wire0 (I2C, 0x70) | lib/robot/ |
| I2CPowerDriver | INA219 | Wire0 (I2C, 0x40 via mux ch0) | lib/robot/ |
| TOFDriver | VL53L0X | Wire0 (I2C) | lib/robot/ |
| I2CBusLock | — | RAII mutex for all I2C buses | lib/robot/ |
| AnalogMuxDriver | — | GPIO analog multiplexer | lib/robot/ |
| AnalogRead | — | Teensy ADC wrapper | lib/robot/ |
| UWBDriver | DW3000 | SPI0 (CS=10) | lib/uwb/ |
| QTimerEncoder | — | GPIO 2-9 (hardware FG pulse counting) | lib/encoders/ |
| MPU6050 | MPU6050 | I2C (minibot/drone) | lib/mpu6050/ |
| CD74HC4067 | CD74HC4067 | GPIO (analog mux) | lib/drivers/ |

---

## micro-ROS Entity Limits

From `custom_microros.meta`:

| Entity | Max |
|--------|-----|
| Nodes | 1 |
| Publishers | 22 |
| Subscriptions | 14 |
| Services | 6 |
| Clients | 1 |
| History depth | 4 |

---

## Deployment Pipeline

```
Developer pushes to master
    |
    +-> CI: test-mcu.yml (253+ unit tests)
    +-> CI: platformio-robot.yml (build all MCU envs)
    +-> CI: build-ws.yml (Docker + colcon build)
    +-> CI: commit-linter.yml (conventional commits)
    |
    v
prod-push.yml auto-syncs master -> prod
    |
    v
deploy-orchestrator.py (systemd on Pi)
    |  watches for: MCU button trigger OR CI trigger file
    |
    +-> git pull (prod branch)
    +-> detect changes (MCU vs ROS vs micro-ROS rebuild)
    +-> OTA flash ESP32s (beacons, minibot, drone) [hash-tracked]
    +-> USB flash Teensy (robot, done last)
    +-> Docker restart + colcon build (if ROS changed)
```

---

## Critical TODOs

| Priority | File | Issue |
|----------|------|-------|
| High | `src/robot/RobotConfig.h:28` | Drive kinematics constants (TRACK_WIDTH, WHEEL_DIAMETER) need measurement/calibration |
| High | `src/robot/machines/RobotLogic.h:181-186` | Intake motor/encoder/button indices need electrical confirmation |
| Medium | `src/minibot/machines/MinibotLogic.h:28-43` | 10 GPIO pin assignments are placeholders (I2C, UWB, motors) |

---

## Recent Changes (merged to master)

Latest commits on master:

| Commit | Change |
|--------|--------|
| `27afe33` | Merge feat/encoders-new (#173) — encoder integration + drive fixes |
| `d65a464` | Quick fix |
| `c4e882e` | Localization logic for drive subsystem |
| `f3910cd` | Wire bus to intake subsystem |
| `9d6da5a` | Merge feat/encoders-new (#166) — first encoder merge |
| `090e72a` | Revert back to Teensy, fix micro-ROS issues |
| `826b096` | Ultra-reset functionality |
| `9514827` | Drive working (initial) |
| `1b22f89` | Remove QTimer4 channel 2 (pin 8 disabled) |
| `8edcac5` | Merge feat/intake-subsystem (#164) |

Key changes since last update:
- EncoderSubsystem integrated with QTimer hardware pulse counting (second merge)
- DriveSubsystem fully wired with PID, S-curve profiles, trajectory controller, RC control
- IntakeSubsystem wired to motor bus with state/command topics
- Localization logic added to drive subsystem
- MicrorosManager: MAX_PARTICIPANTS=32, executor handles=24
- ResetSubsystem manages 9 reset targets
- micro-ROS participant count: 20 active (with UWB enabled)
