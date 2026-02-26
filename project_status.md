# SEC26 Project Status

**Last Updated**: 2026-02-26
**Branch**: `feat/hardware-test` (`1bfa9e0 fix: rc`)
**Base**: `master`

## Maturity Summary

### MCU Subsystems (Robot — Teensy41)

| Subsystem | Status | ROS2 | Threaded | Notes |
|-----------|--------|------|----------|-------|
| MicrorosManager | Mature | Core | Pri 4 | Agent lifecycle, 16 participants, 10 executor handles |
| OLEDSubsystem | Mature | Sub | Pri 1, 25ms | Hardware SPI1 (pins 26/27), text append + scroll |
| HeartbeatSubsystem | Mature | Pub | Pri 1, 200ms | String heartbeat at 5 Hz |
| BatterySubsystem | Mature | Pub | Pri 1, 100ms | INA219 via I2C mux ch0, OLED status line |
| ImuSubsystem | Mature | Pub | Pri 3, 10ms | BNO085 quaternion + angular velocity at 100 Hz |
| RCSubsystem | Mature | Pub | Main loop | FlySky IBUS, 10 channels, failsafe on ch9 |
| SensorSubsystem | Mature | Pub | Pri 1, 100ms | VL53L0X TOF distance at 10 Hz |
| ServoSubsystem | Mature | Pub+Srv | Pri 2, 25ms | PCA9685 #0, 8 servos, SetServo service |
| MotorManagerSubsystem | Mature | Pub+Srv | Pri 2, 1ms | PCA9685 #1, 8 motors, NFPShop reverse-pulse |
| LEDSubsystem | Mature | Sub | Pri 1, 50ms | WS2812B via FastLED, set_all subscription |
| ButtonSubsystem | Mature | Pub | Pri 1, 20ms | TCA9555 port 1, UInt8 bitmask |
| DipSwitchSubsystem | Mature | Pub | Pri 1, 500ms | TCA9555 port 0, UInt8 bitmask |
| UWBSubsystem | Mature | Pub | Pri 2, 50ms | DW3000 TAG mode, TWR ranging to 3 anchors |
| ArmSubsystem | Implemented | Pub+Srv | Pri 2, 20ms | Servo + encoder, IK solver available |
| IntakeSubsystem | Implemented | Pub | Pri 2, 20ms | PWM motor + IR sensor |
| IntakeBridgeSubsystem | Implemented | Pub+Srv | Pri 2, 20ms | Gear-and-rack, TOF feedback, pin numbers TBD |
| DriveSubsystem | Not Integrated | — | — | Code exists in `drive-base/`, commented out in RobotLogic.h |

### MCU Subsystems (Other Platforms)

| Platform | Status | Notes |
|----------|--------|-------|
| Beacon (ESP32, x3) | Mature | UWB ANCHOR mode, WiFi micro-ROS, inter-beacon ranging |
| Minibot (ESP32) | Implemented | Differential drive, UWB TAG, cmd_vel subscription |
| Drone (ESP32) | Scaffolding | Empty main.cpp, no subsystems |
| Field Elements (ESP32, x6) | Mature | Button, Crank, Earth, Keypad, Pressure, Controller via ESP-NOW |

### ROS2 Packages

| Package | Status | Language | Notes |
|---------|--------|----------|-------|
| secbot_autonomy | Implemented | C++ | 7 task state machines, 50 Hz orchestration |
| secbot_navigation | Implemented | C++ | D* Lite + pure pursuit, grid map, path smoother |
| secbot_vision | Implemented | Python | HSV duck detection, TF broadcaster, antenna LED (stubbed) |
| secbot_uwb | Implemented | C++ | Trilateration, outlier rejection (2.0σ), 2 test files |
| secbot_fusion | Minimal | C++ | DriveBase → Odometry pass-through, ready for EKF |
| secbot_health | Minimal | C++ | Heartbeat watchdog, 2 launch files |
| secbot_sim | Implemented | C++/Python | Gazebo Harmonic, MCU subsystem emulator |
| secbot_bridge_i2c | Implemented | C++ | Packet codec, rate limiter, fake Teensy for sim |
| secbot_tf | Scaffolding | YAML | Static transform config, launch files only |
| mcu_msgs | Mature | IDL | 21 messages, 5 services |
| secbot_msgs | Mature | IDL | 3 messages, 2 services, 2 actions |
| my_robot_description | Scaffolding | URDF | Xacro robot model for visualization |
| test_package | Scaffolding | — | CI workflow verification only |

### Native Test Suites (253+ tests)

| Suite | Tests | Status |
|-------|-------|--------|
| test-math-pose2d | Pose2D transforms | Passing |
| test-math-vector2d | Vector2D operations | Passing |
| test-math-pose3d | Pose3D 3D transforms | Passing |
| test-control-pid | PID controller | Passing |
| test-control-arm-kinematics | Arm IK solver | Passing |
| test-control-trapezoidal-motion-profile | Trapezoidal profiles | Passing |
| test-control-scurve-motion-profile | S-curve profiles | Passing |
| test-control-trajectory-controller | Path tracking | Passing |
| test-drive-tankdrivelocalization | Tank drive odometry | Passing |
| test-utils-filters | Signal filters | Passing |
| test-utils-units | Unit conversions | Passing |

### CI/CD Workflows

| Workflow | Trigger | Status |
|----------|---------|--------|
| test-mcu.yml | PR (mcu_ws/mcu_msgs) | Active — runs 11 native test envs |
| platformio-robot.yml | PR (mcu_ws) | Active — builds all default envs |
| build-ws.yml | PR/push master (ros2_ws) | Active — Docker build + colcon |
| pi-deploy.yml | Push to prod/CD-testing | Active — flash Teensy + deploy ROS |
| prod-push.yml | Push to master | Active — auto-sync master → prod |
| auto-format.yml | PR labeled 'format' | Active — clang-format |
| commit-linter.yml | All PRs | Active — Conventional Commits |
| test-multiarch.yml | Manual only | Available — multi-arch runner test |

## Hardware Drivers

| Driver | IC/Module | Bus | Location |
|--------|-----------|-----|----------|
| BNO085Driver | BNO085 IMU | Wire1 (I2C) | lib/robot/BNO085.h |
| TCA9555Driver | TCA9555 GPIO expander | Wire0 (I2C) | lib/robot/TCA9555Driver.h |
| I2CMuxDriver | TCA9548A multiplexer | Wire0 (I2C) | lib/robot/I2CMuxDriver.h |
| I2CPowerDriver | INA219 current/voltage | Wire0 via mux | lib/robot/I2CPowerDriver.h |
| PCA9685Driver | PCA9685 PWM (servo/motor) | Wire2 (I2C) | lib/robot/PCA9685Driver.h |
| TOFDriver | VL53L0X distance | Wire0 (I2C) | lib/robot/TOF.h |
| UWBDriver | DW3000 UWB | SPI0 | lib/uwb/UWBDriver.h |
| I2CBusLock | — (mutex) | Wire0/1/2 | lib/robot/I2CBusLock.h |
| AnalogMuxDriver | CD74HC4067 | GPIO/ADC | lib/robot/AnalogMuxDriver.h |
| EncoderDriver | Quadrature encoder | GPIO | src/robot/drive-base/EncoderDriver.h |

## Critical Path / Outstanding TODOs

1. **DriveSubsystem not integrated** — Code in `src/robot/drive-base/` is complete but commented out in `RobotLogic.h:174-177`. Needs encoder/motor pin configuration from electrical team.
2. **IntakeBridge pin numbers TBD** — `RobotLogic.h:165` and `IntakeBridgeSubsystem.cpp:21` — rack motor, home switch, and TOF pins need confirmation from electrical.
3. **RobotConfig.h placeholder values** — Drive kinematics constants (track width, wheel diameter, encoder ticks) are not yet measured.
4. **DW3000 RST_PIN conflict** — The DW3000 Arduino library hardcodes `RST_PIN 27` in `DW3000Constants.h`, which conflicts with OLED SPI1 SCK. Workaround in `UWBDriver.cpp` skips `hardReset()` when no rst_pin is wired. If a reset pin is ever wired, it must NOT be pin 27.

## Known Hardware Quirks

- **NFPShop brushless motors**: Require a brief reverse-pulse (~3ms at ~2% duty) every ~103ms to prevent the integrated controller from faulting. Handled in `MotorManagerSubsystem::update()`.
- **IBusBM NOTIMER**: The FlySky IBUS library's NOTIMER mode only works from the main Arduino `loop()` thread — TeensyThreads context breaks it.
- **Pin 35 shared**: Serial8 TX and WS2812B (FastLED) share pin 35. RC `ibus_.begin()` must run before `FastLED.addLeds()` so FastLED reclaims the pin for GPIO output. IBUS is RX-only so losing TX is fine.
- **SPI bus separation**: OLED uses SPI1 (pins 26/27), UWB uses SPI0 (pins 11/12/13). They must not share a bus.

## Deployment Pipeline

```
Push to master
     │
     ▼
prod-push.yml ──► auto-sync to prod branch
     │
     ▼
pi-deploy.yml
     │
     ├── Job 1 (AMD64 runner): Build robot firmware → upload artifact
     │
     └── Job 2 (rpi-ros2 runner on Pi):
              ├── Download firmware artifact
              ├── Stage into .pio/build/robot/
              ├── flash_mcu.sh (teensy_loader_cli, 5 retries)
              ├── deploy-all.py (colcon build + rosdep)
              └── Launch test node
```

## Recent Changes (feat/hardware-test branch)

| Commit | Description |
|--------|-------------|
| `1bfa9e0` | Fix RC: moved to main loop polling (IBusBM NOTIMER thread issue) |
| `3467cfa` | Fix UWB/OLED SPI conflict (DW3000 RST_PIN 27 vs SPI1 SCK) |
| `2ec1d9e` | Fix missing RobotPins.h include |
| `905d469` | OLED switched to hardware SPI1, motor reverse-pulse duty reduced |
| `7d53420` | Fix arm64 compilation issues |
| `d94a8df` | Add DriveSubsystem skeleton (commented out, needs config) |
| `6be8d7f` | Add minibot differential drive subsystem |
| `921bd42` | Add UWB subsystem to teensy-test-all-subsystems |

**Uncommitted**: `RobotLogic.h` updated to match all teensy-test-all-subsystems fixes (SPI1 OLED, 1000 Hz motor, RC in loop, all subsystems wired).
