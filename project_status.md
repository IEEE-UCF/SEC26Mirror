# Project Status — SEC26 Robot

**Last Updated:** 2026-03-01
**Branch:** `feat/encoders-new` (encoder integration + drive fixes vs master)
**Latest master:** `8edcac5` (Merge feat/intake-subsystem #164)

---

## Maturity Summary

### MCU Subsystems (Teensy41 Robot Firmware)

| Subsystem | Status | Notes |
|-----------|--------|-------|
| MicrorosManager | Mature | 20 participants registered, 10 executor handles |
| BatterySubsystem | Mature | INA219, OLED status line, getVoltage/getCurrentA |
| ImuSubsystem | Mature | BNO085, gyro calibration, getYaw accessor |
| RCSubsystem | Mature | FlySky IBUS, main-loop polling |
| OLEDSubsystem | Mature | Terminal + debug dashboard mode (DIP 6) |
| SensorSubsystem | Implemented | Single VL53L0X TOF |
| ServoSubsystem | Mature | PCA9685, SetServo service |
| MotorManagerSubsystem | Mature | PCA9685, 1kHz reverse-pulse, SetMotor service |
| EncoderSubsystem | Implemented | QTimer hardware FG counting, 50Hz publish |
| DriveSubsystem | Implemented | Tank drive, PID, S-curve profiles, trajectory controller, RC manual mode |
| DipSwitchSubsystem | Mature | 8-bit named constants, isSwitchOn(), conditional behaviors |
| ButtonSubsystem | Mature | TCA9555 port 1, interrupt-ready |
| LEDSubsystem | Mature | WS2812B, LedColor subscription |
| UWBSubsystem | Implemented | DW3000 tag, conditional on DIP 2 |
| ArmSubsystem | Implemented | Servo + encoder, kinematics |
| IntakeSubsystem | Implemented | Linear rail + spinning motor, limit switch buttons |
| CrankSubsystem | Implemented | Servo channel 0 |
| KeypadSubsystem | Implemented | Servo channel 1 + drive-to-press |
| DeploySubsystem | Implemented | Button-triggered, DIP 8 target select |
| ResetSubsystem | Implemented | micro-ROS Reset service, 9 reset targets |
| HeartbeatSubsystem | Mature | 5 Hz heartbeat string |
| MiniRobotSubsystem | Implemented | Mini-robot control |

### MCU Subsystems (Other Targets)

| Target | Status | Notes |
|--------|--------|-------|
| Beacons (x3) | Implemented | ESP32 + DW3000, micro-ROS WiFi UDP |
| Minibot | Scaffolding | Seeed XIAO ESP32-S3, pin assignments are TODOs |
| Drone | Implemented | Flight controller, IMU, height, IR (ESP32) |
| Field Elements (x6) | Implemented | ESP-NOW communication |

### ROS2 Packages

| Package | Version | Status | Notes |
|---------|---------|--------|-------|
| mcu_msgs | 1.0.0 | Mature | 20 msgs, 6 srvs — shared MCU<>ROS2 interface |
| secbot_msgs | 0.1.0 | Implemented | TaskStatus, DuckDetection, NavigatePath, ApproachTarget |
| secbot_autonomy | 0.0.1 | Implemented | 7 competition task state machines, 50Hz orchestration |
| secbot_navigation | 0.0.1 | Implemented | D* Lite + pure pursuit, yaml-cpp config |
| secbot_vision | 0.1.0 | Implemented | HSV duck detection (Python), cv_bridge |
| secbot_fusion | 0.0.1 | Scaffolding | DriveBase -> Odometry pass-through, ready for EKF |
| secbot_uwb | 0.1.0 | Implemented | Trilateration, outlier rejection, Eigen |
| secbot_sim | 0.0.1 | Implemented | Gazebo Harmonic + MCU subsystem emulator |
| secbot_health | 0.0.1 | Implemented | MCU heartbeat watchdog |
| secbot_bridge_i2c | 0.0.1 | Scaffolding | I2C bridge + fake Teensy for sim |
| secbot_deploy | 0.0.1 | Implemented | Deployment trigger bridge (MCU button -> orchestrator) |
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
| prod-push.yml | Push to master | Active — force-pushes master -> prod |
| auto-format.yml | PR labeled `format` OR push to master | Active — clang-format auto-commit |
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
| MPU6050 | MPU6050 | I2C (minibot/drone) | lib/mpu6050/ |
| CD74HC4067 | CD74HC4067 | GPIO (analog mux) | lib/drivers/ |

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
| High | `src/robot/RobotConfig.h` | Drive kinematics constants need measurement/calibration |
| High | `src/robot/machines/RobotLogic.h:181-186` | Intake motor/encoder/button indices need electrical confirmation |
| Medium | `src/minibot/machines/MinibotLogic.h` | 6 GPIO pin assignments are placeholders |

---

## Recent Changes (feat/encoders-new branch)

Changes vs master (`8edcac5`):

| Commit | Change |
|--------|--------|
| `090e72a` | Revert back to Teensy, fix micro-ROS issues |
| `826b096` | Ultra-reset functionality |
| `9514827` | Drive working (initial) |
| `1b22f89` | Remove QTimer4 channel 2 (pin 8 disabled) |
| `64d26e0` | New encoder fix integrations |
| `ef1b4dc` | Encoder wiring updates |
| `be22fe9` | Proper internal counter support for encoders |
| `5d6d1e5` | Encoder fixes |

Key changes on this branch:
- EncoderSubsystem integrated with QTimer hardware pulse counting
- DriveSubsystem fully wired with PID, S-curve profiles, trajectory controller
- IntakeSubsystem added (linear rail + spinning motor mechanism)
- IntakeBridgeSubsystem removed (replaced by IntakeSubsystem)
- QTimer4 channel 2 (pin 8) disabled due to hardware conflict
- ResetSubsystem now manages 9 reset targets
- micro-ROS participant count increased to 20
