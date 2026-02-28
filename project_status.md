# Project Status — SEC26 Robot

**Last Updated:** 2026-02-28
**Branch:** `feat/dip-switches` (7 files changed, +280 -14 vs master)
**Latest master:** `94ec4c3` (Merge feat/full-drivebase)

---

## Maturity Summary

### MCU Subsystems (Teensy41 Robot Firmware)

| Subsystem | Status | Notes |
|-----------|--------|-------|
| MicrorosManager | Mature | 32 participants / 24 executor handles |
| BatterySubsystem | Mature | INA219, OLED status line, getVoltage/getCurrentA |
| ImuSubsystem | Mature | BNO085, gyro calibration, getYaw accessor |
| RCSubsystem | Mature | FlySky IBUS, main-loop polling |
| OLEDSubsystem | Mature | Terminal + debug dashboard mode (DIP 6) |
| SensorSubsystem | Implemented | Single VL53L0X TOF |
| ServoSubsystem | Mature | PCA9685, SetServo service |
| MotorManagerSubsystem | Mature | PCA9685, 1kHz reverse-pulse, SetMotor service |
| EncoderSubsystem | Implemented | QTimer hardware FG counting |
| DriveSubsystem | Implemented | Tank drive, PID, S-curve profiles, trajectory controller |
| DipSwitchSubsystem | Mature | 8-bit named constants, isSwitchOn(), conditional behaviors |
| ButtonSubsystem | Mature | TCA9555 port 1, interrupt-ready |
| LEDSubsystem | Mature | WS2812B, LedColor subscription |
| UWBSubsystem | Implemented | DW3000 tag, conditional on DIP 2 |
| ArmSubsystem | Implemented | Servo + encoder, kinematics |
| IntakeSubsystem | Implemented | Intake/eject mechanism |
| IntakeBridgeSubsystem | Scaffolding | Pin numbers unconfirmed (TODO) |
| CrankSubsystem | Implemented | Servo channel 0 |
| KeypadSubsystem | Implemented | Servo channel 1 + drive-to-press |
| DeploySubsystem | Implemented | Button-triggered, DIP 8 target select |
| ResetSubsystem | Implemented | micro-ROS Reset service |
| HeartbeatSubsystem | Mature | 5 Hz heartbeat string |

### MCU Subsystems (Other Targets)

| Target | Status | Notes |
|--------|--------|-------|
| Beacons (×3) | Implemented | ESP32 + DW3000, micro-ROS WiFi UDP |
| Minibot | Scaffolding | Pin assignments are TODOs |
| Drone | Implemented | Flight controller, IMU, height, IR |
| Field Elements (×6) | Implemented | ESP-NOW communication |

### ROS2 Packages

| Package | Version | Status | Notes |
|---------|---------|--------|-------|
| mcu_msgs | 1.0.0 | Mature | 21 msgs, 6 srvs — shared MCU↔ROS2 interface |
| secbot_msgs | 0.1.0 | Implemented | TaskStatus, DuckDetection, NavigatePath, ApproachTarget |
| secbot_autonomy | 0.0.1 | Implemented | 7 competition task state machines, 50Hz orchestration |
| secbot_navigation | 0.0.1 | Implemented | D* Lite + pure pursuit, yaml-cpp config |
| secbot_vision | 0.1.0 | Implemented | HSV duck detection (Python), cv_bridge |
| secbot_fusion | 0.0.1 | Scaffolding | DriveBase → Odometry pass-through, ready for EKF |
| secbot_uwb | 0.1.0 | Implemented | Trilateration, outlier rejection, Eigen |
| secbot_sim | 0.0.1 | Implemented | Gazebo Harmonic + MCU subsystem emulator |
| secbot_health | 0.0.1 | Implemented | MCU heartbeat watchdog |
| secbot_bridge_i2c | 0.0.1 | Scaffolding | I2C bridge + fake Teensy for sim |
| secbot_deploy | 0.0.1 | Implemented | Deployment trigger bridge (MCU button → orchestrator) |
| secbot_tf | 0.0.1 | Implemented | Static transform configuration |
| my_robot_description | 0.0.1 | Implemented | URDF for RViz visualization |

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
| test-mcu.yml | PR touching mcu_ws/mcu_msgs | Active — runs 253+ unit tests |
| platformio-robot.yml | PR touching mcu_ws/mcu_msgs | Active — builds all default PIO envs |
| build-ws.yml | PR/push to master (ros2_ws) | Active — Docker image + colcon build |
| commit-linter.yml | All PRs | Active — enforces Conventional Commits |
| prod-push.yml | Push to master | Active — force-pushes master → prod |
| auto-format.yml | PR labeled `format` | Active — clang-format auto-commit |
| test-multiarch.yml | Manual dispatch | Active — multi-architecture test |

---

## Hardware Drivers

| Driver | IC | Bus | Library |
|--------|----|-----|---------|
| BNO085Driver | BNO085 | Wire1 (I2C, 0x4B) | lib/robot/ |
| TCA9555Driver | TCA9555 | Wire0 (I2C, 0x20) | lib/robot/ |
| PCA9685Driver | PCA9685 | Wire2 (I2C, 0x40/0x41) | lib/robot/ |
| I2CMuxDriver | TCA9548A | Wire0 (I2C, 0x70) | lib/robot/ |
| I2CPowerDriver | INA219 | Wire0 (I2C, 0x40 via mux) | lib/robot/ |
| TOFDriver | VL53L0X | Wire0 (I2C) | lib/robot/ |
| UWBDriver | DW3000 | SPI0 (CS=10) | lib/uwb/ |
| QTimerEncoder | — | GPIO 2-9 (hardware FG) | lib/encoders/ |
| MPU6050 | MPU6050 | I2C (alternative) | lib/mpu6050/ |
| CD74HC4067 | CD74HC4067 | GPIO (analog mux) | lib/drivers/ |

---

## Deployment Pipeline

```
Developer pushes to master
    │
    ├─► CI: test-mcu.yml (253+ unit tests)
    ├─► CI: platformio-robot.yml (build all MCU envs)
    ├─► CI: build-ws.yml (Docker + colcon build)
    ├─► CI: commit-linter.yml (conventional commits)
    │
    ▼
prod-push.yml auto-syncs master → prod
    │
    ▼
deploy-orchestrator.py (systemd on Pi)
    │  watches for: MCU button trigger OR CI trigger file
    │
    ├─► git pull (prod branch)
    ├─► detect firmware changes (hash comparison)
    ├─► OTA flash ESP32s (beacons, minibot, drone)
    ├─► USB flash Teensy (robot)
    └─► Docker restart + colcon build
```

---

## Critical TODOs

| Priority | File | Issue |
|----------|------|-------|
| High | `src/robot/RobotConfig.h:28` | Drive kinematics constants need measurement/calibration |
| High | `src/robot/machines/RobotLogic.h:184` | IntakeBridge pin numbers unconfirmed with electrical |
| High | `src/robot/subsystems/IntakeBridgeSubsystem.cpp:23` | Pin numbers need electrical team confirmation |
| Medium | `src/minibot/machines/MinibotLogic.h:28-43` | 11 GPIO pin assignments are placeholders |

---

## Recent Changes (feat/dip-switches branch)

Changes vs master (`94ec4c3`):

| File | Change |
|------|--------|
| `DipSwitchSubsystem.h` | +8 named bit constants, +isSwitchOn() helper |
| `RobotLogic.h` | RC override moved to bit 0, conditional UWB (DIP 2), OLED dashboard wiring |
| `OLEDSubsystem.h/.cpp` | Debug dashboard mode (DIP 6): battery, IMU, UWB, DIP binary display |
| `BatterySubsystem.h` | +getVoltage(), +getCurrentA() accessors |
| `UWBSubsystem.h` | +getDriverData() accessor |
| `teensy-test-all-subsystems.cpp` | +RC drive in loop, +configureDriveSetup(), +conditional UWB, +OLED dashboard |
