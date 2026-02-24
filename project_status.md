# SEC26 Project Status

**Last Updated:** 2026-02-24
**Branch:** feat/pi-deploy
**Last Commit:** 25cb492 (Merge feat/display into master)

---

## Maturity Summary

| Component | Status | Notes |
|-----------|--------|-------|
| **MCU Firmware (Teensy)** | | |
| OLED Display | Mature | Serial terminal, ROS2 topic control |
| Battery Monitor | Mature | INA228, voltage/current/temp |
| TOF Sensors | Mature | VL53L0X/VL53L1X, multi-sensor via TCA9548 mux |
| IMU (BNO085) | Mature | 9-DOF, 50Hz update rate |
| Arm Control | Mature | Forward/inverse kinematics, PCA9685 servos |
| RC Input | Mature | FlySky IBUS, 5ms polling |
| Intake | Mature | PWM motor + IR sensor, ROS2 integration |
| Heartbeat | Mature | LED indicator, 5Hz |
| micro-ROS Manager | Mature | Serial transport, FreeRTOS priority 4 |
| **Drive Base** | **Not Integrated** | Code exists but commented out in RobotLogic.h (TODO line 100) |
| **MCU Firmware (ESP32)** | | |
| Field Elements (6) | Mature | button, controller, crank, earth, keypad, pressure |
| UWB Beacons (2) | Mature | DW3000, micro-ROS |
| Drone | Minimal | Basic main.cpp only |
| **ROS2 Packages** | | |
| secbot_autonomy | Implemented | Task state machines for all competition elements |
| secbot_vision | Implemented | antenna_led_detector, duck_detector, teleop_pid |
| secbot_uwb | Implemented | UWB positioning with Eigen, includes unit tests |
| secbot_sim | Implemented | Gazebo integration, MCU lib seeding |
| secbot_tf | Implemented | Static/dynamic TF broadcasting, sim + hardware |
| secbot_msgs / mcu_msgs | Implemented | Custom message definitions, symlinked to mcu_ws |
| secbot_navigation | Scaffolding | Basic structure, needs Nav2 integration |
| secbot_fusion | Scaffolding | Depends on robot_localization, needs tuning |
| secbot_health | Scaffolding | Heartbeat monitoring structure only |
| secbot_bridge_i2c | Scaffolding | Interface definitions only |
| **Testing** | | |
| Native Unit Tests | Mature | 253+ test cases across 13 envs, ~30-40s |
| Hardware Tests | Mature | 9 envs (Teensy + ESP32), manual execution |
| Integration Tests | Documented | Procedures in INTEGRATION_TESTS.md, manual |
| **CI/CD** | | |
| PlatformIO Build All | Mature | PR gate, all non-test envs |
| MCU Unit Tests | Mature | PR gate, 13 native test suites |
| Pi Deploy | Mature | amd64 cross-build + Pi deploy via Gitea Actions |
| Docker Build | Mature | Multi-arch (amd64/arm64), registry push |
| Commit Linter | Active | Conventional commits enforced |
| Auto Formatter | Active | clang-format, label-triggered |
| **Deployment** | | |
| Button Daemon | Mature | systemd service, internet check, Gitea dispatch |
| Flash Script | Mature | Teensy only, 5 retries, prebuilt artifact support |
| deploy-all.py | Mature | MCU flash + ROS build + launch orchestration |
| LED Indication | Mature | systemd transient unit for blink during deploy |

---

## Critical Path Items

### 1. Drive Subsystem Integration (BLOCKING)

The drive subsystem code exists (`lib/robot/`, `lib/control/`) but is **not wired into** the robot firmware. In `mcu_ws/src/robot/machines/RobotLogic.h` (line ~100), the drive init and FreeRTOS task are commented out with a TODO to configure actual hardware pin values.

**What's needed:**
- Configure encoder and motor pins for actual hardware wiring
- Uncomment drive subsystem init/register in RobotLogic.h
- Test tank drive odometry with real motors
- Verify micro-ROS drive topic publishing

### 2. Navigation Stack Integration

`secbot_navigation` has basic scaffolding but needs Nav2 or custom SLAM integration. Path planning, obstacle avoidance, and UWB-based localization accuracy need real-robot testing.

### 3. Sensor Fusion

`secbot_fusion` depends on `robot_localization` for EKF/UKF. Needs IMU + TOF + UWB + encoder fusion tuning on the physical robot.

---

## Hardware Drivers Implemented

| Sensor/Actuator | IC | Bus | Driver Location |
|----------------|----|-----|-----------------|
| Power monitor | INA228 | I2C (0x40) | lib/sensors/ |
| TOF distance | VL53L0X/VL53L1X | I2C + TCA9548 mux | lib/sensors/ |
| IMU | BNO085 | I2C | lib/sensors/ |
| Servo driver | PCA9685 | I2C | lib/robot/ |
| Motor | PWM H-bridge | GPIO PWM | lib/robot/ |
| Encoder | Quadrature | GPIO | lib/robot/ |
| RC receiver | FlySky IBUS | UART1 | lib/robot/ |
| UWB ranging | DW3000 | SPI | lib/subsystems/ |
| I2C mux | TCA9548 | I2C | External lib |
| GPIO expander | MCP23017 | I2C | lib/robot/ |
| Analog mux | CD74HC4067 | GPIO + analog | lib/robot/ |
| OLED display | SSD1306 128x64 | Software SPI | External lib |
| IR comm | NEC protocol | GPIO | lib/ir/ |

---

## Library Architecture (platformio.ini)

Libraries are split into targeted groups to speed up builds:

- **`lib_core`** — SEC-Base-Classes only. Used by ESP32 field elements and beacons.
- **`lib_robot`** — Full set (14+ libs). Used by Teensy robot and test environments.
- **Per-env overrides** — field-button gets ezButton, field-earth gets IRremote, beacons get DW3000, drone gets nothing.

---

## CI/CD Workflows

| Workflow | Trigger | Runner | Purpose |
|----------|---------|--------|---------|
| `pi-deploy.yml` | push to `prod`, manual dispatch | amd64 + rpi-ros2 | Cross-build firmware, deploy to Pi |
| `platformio-robot.yml` | PR to master (mcu_ws changes) | ubuntu-latest | Build all PlatformIO environments |
| `test-mcu.yml` | PR to master (mcu_ws changes) | ubuntu-latest | Run 13 native test suites |
| `build-ws.yml` | PR/push to master (ros2_ws changes) | ubuntu-latest | Docker build + colcon build |
| `commit-linter.yml` | All PRs | ubuntu-latest | Conventional commit enforcement |
| `auto-format.yml` | PRs with `format` label | ubuntu-latest | clang-format auto-commit |

---

## Deployment Pipeline

See `scripts/README.md` for full setup instructions.

```
Button press (BCM 22) / git push to prod
    -> check_connectivity() (ping 8.8.8.8 / curl Gitea API)
    -> Gitea workflow_dispatch
    -> pi-deploy.yml
        Job 1: build Teensy firmware on amd64
        Job 2: on Pi —
            LED blink (systemd transient unit)
            git pull + docker compose rebuild
            deploy-all.py inside container:
                stage prebuilt firmware.hex
                flash Teensy (5 retries)
                colcon build ROS2
                launch test node
            LED solid (deploy complete)
```

---

## Recent Changes (feat/pi-deploy branch)

- Removed `robotcomms` entirely (source, env definition, all CI/deploy references)
- Split `platformio.ini` lib_deps into targeted groups (`lib_core` / `lib_robot`)
- Added internet connectivity check to button daemon and workflow
- Replaced fragile `gpioset -z` LED control with `systemd-run` transient unit
- Trimmed `default_envs` to deployable targets only
- Created `scripts/README.md` deployment guide
