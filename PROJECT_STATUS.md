# SEC26 - SoutheastCon 2026 Project Status

> **Last Updated**: 2026-02-24
> **Competition**: IEEE SoutheastCon 2026, Alabama
> **Theme**: Rescue ducks stranded on the moon
> **Field**: 4x8 feet | Robot start: 12x12 inch square in corner

---

## Table of Contents

1. [Scoring Overview](#1-scoring-overview)
2. [System Architecture](#2-system-architecture)
3. [Robot MCU Firmware](#3-robot-mcu-firmware)
4. [Minibot](#4-minibot)
5. [Drone](#5-drone)
6. [ROS2 Software Stack](#6-ros2-software-stack)
7. [UWB Positioning System](#7-uwb-positioning-system)
8. [Field Elements](#8-field-elements)
9. [DevOps & Deployment](#9-devops--deployment)
10. [Communication Protocols](#10-communication-protocols)
11. [Test Infrastructure](#11-test-infrastructure)
12. [Master TODO Tracker](#12-master-todo-tracker)

---

## 1. Scoring Overview

| Task | Points | Mechanism | Status |
|------|--------|-----------|--------|
| **Beacon 1 - Button** | TBD | Bump button 3 times | Field element done; robot approach via autonomy |
| **Beacon 2 - Crank** | TBD | Servo with 1 DOF rotates crank | Field element done; crank task stubbed in autonomy |
| **Beacon 3 - Pressure** | TBD | Rack-and-pinion intake with surgical tubing (crater center) | Field element done; intake subsystem done |
| **Beacon 4 - Keypad** | TBD | Servo picks gear to enter hardcoded number (1 DOF) | Field element done; keypad task done in autonomy |
| **Duck Pickup** | TBD | Pick up ducks, place in 12x12 square | Intake state machine done; vision duck detection done |
| **Drone** | TBD | Take off from robot, fly, land on robot | **NOT STARTED** - firmware is empty stub |
| **IR Color Transmission** | TBD | Read antenna LED colors, transmit via IR to "Earth" | IR library done; vision LED detector **STUBBED** |
| **Minibot Crater Loop** | TBD | Minibot exits intake, enters crater, full loop, exits | MiniRobot subsystem done; minibot ESP32 firmware **NOT STARTED** |
| **Deploy UWB Beacons** | N/A (enables localization) | Robot places beacons at 2 corners | UWB firmware done; deployment mechanism **NOT IMPLEMENTED** |

---

## 2. System Architecture

### Hardware Overview

```
┌─────────────────────────────────────────────────────────────┐
│                     MAIN ROBOT (12x12)                       │
│                                                              │
│  ┌──────────┐   Serial    ┌──────────┐    WiFi    ┌───────┐ │
│  │ Teensy41 │◄──────────►│ Rasp Pi  │◄──────────►│ESP32  │ │
│  │ (robot)  │  micro-ROS │ (ROS2    │  micro-ROS │(comms)│ │
│  │          │            │  Jazzy)  │            │UWB Tag│ │
│  └──────────┘            └──────────┘            └───────┘ │
│       │                       │                      │      │
│  I2C Buses:              ROS2 Nodes:            DW3000 SPI  │
│  Wire0: TCA9548A mux     - Autonomy                        │
│         TCA9555 GPIO      - Navigation                      │
│         INA219 (power)    - Vision                          │
│  Wire1: BNO085 (IMU)     - Fusion                          │
│  Wire2: PCA9685 x2       - UWB Positioning                 │
│         (servos/PWM)      - TF / Health                     │
│                                                              │
│  Other: SSD1306 OLED (SPI), FlySky RC (Serial1)            │
│         Motors (PWM), Encoders, IR breakbeam                │
│         Addressable LEDs, Buttons                           │
│                                                              │
│  ┌──────────┐    ┌───────────┐                              │
│  │ Minibot  │    │   Drone   │                              │
│  │ (stored  │    │ (stored   │                              │
│  │ in       │    │ on robot) │                              │
│  │ intake)  │    │           │                              │
│  └──────────┘    └───────────┘                              │
└─────────────────────────────────────────────────────────────┘

              UWB Ranging
    ┌──────────┐         ┌──────────┐
    │ Beacon 1 │         │ Beacon 2 │
    │ ID=10    │         │ ID=11    │
    │ (corner) │         │ (corner) │
    └──────────┘         └──────────┘
              ↕ DW3000 TWR ↕
           ┌──────────────────┐
           │  Robotcomms Tag  │
           │  ID=12 (on robot)│
           └──────────────────┘
                    ↕
              ┌──────────┐
              │ Beacon 3 │
              │ ID=13    │
              └──────────┘
```

### PCB Components

| Component | Address | I2C Bus | Purpose |
|-----------|---------|---------|---------|
| TCA9548A | 0x70 | Wire0 | I2C Multiplexer |
| TCA9555 | 0x20 | Wire0 | 16-bit GPIO Expander |
| INA219 | 0x40 (mux ch0) | Wire0 | Power monitoring |
| BNO085 | 0x4A | Wire1 | 9-DOF IMU |
| PCA9685 #1 | 0x40 | Wire2 | PWM/Servo driver |
| PCA9685 #2 | 0x41 | Wire2 | PWM/Servo driver |
| SSD1306 | N/A (SPI) | SPI | 128x64 OLED display |

---

## 3. Robot MCU Firmware

**Platform**: Teensy 4.1 | **Framework**: Arduino + TeensyThreads | **Transport**: micro-ROS serial
**Entry Point**: `mcu_ws/src/robot/machines/RobotLogic.h`

### Subsystem Status

| Subsystem | File(s) | Status | ROS Topic | Update Rate |
|-----------|---------|--------|-----------|-------------|
| **micro-ROS Manager** | `lib/microros/microros_manager_robot.*` | DONE | Agent lifecycle | 10ms (prio 4) |
| **Heartbeat** | `src/robot/machines/HeartbeatSubsystem.h` | DONE | `/mcu_robot/heartbeat` | 200ms (prio 1) |
| **Battery** | `src/robot/subsystems/BatterySubsystem.*` | DONE | `/mcu_robot/battery_health` | 100ms (prio 1) |
| **IMU** | `src/robot/subsystems/ImuSubsystem.*` | DONE | `/mcu_robot/imu/data` | 20ms (prio 3) |
| **TOF Sensor** | `src/robot/subsystems/SensorSubsystem.*` | DONE | `/mcu_robot/tof_distances` | 100ms (prio 1) |
| **RC Receiver** | `src/robot/subsystems/RCSubsystem.*` | DONE | `/mcu_robot/rc` | 5ms (prio 3) |
| **Intake** | `src/robot/subsystems/IntakeSubsystem.*` | DONE | `/mcu_robot/intake/state` | 20ms (prio 2) |
| **OLED Display** | `src/robot/subsystems/OLEDSubsystem.*` | DONE | `/mcu_robot/lcd/append` (srv) | 50ms (prio 1) |
| **Mini Robot** | `src/robot/subsystems/MiniRobotSubsystem.*` | DONE | `/mcu_robot/mini_robot/state` | 20ms (prio 2) |
| **Arm** | `src/robot/subsystems/ArmSubsystem.h` | PSEUDO-CODE | Publisher/service commented out | 20ms (prio 2) |
| **Drive Base** | `src/robot/drive-base/*` | CODE EXISTS, NOT INTEGRATED | `drive_base/status` | Commented out |
| **Servo Manager** | `src/robot/subsystems/ServoSubsystem.h` | DONE | `/mcu_robot/servo/state`, `/mcu_robot/servo/set` (srv) | 50ms (prio 2) |
| **Motor Manager** | `src/robot/subsystems/MotorManagerSubsystem.h` | DONE | `/mcu_robot/motor/state`, `/mcu_robot/motor/set` (srv) | 50ms (prio 2) |
| **Buttons (TCA9555)** | `src/robot/subsystems/ButtonSubsystem.h` | DONE | `/mcu_robot/buttons` | 20ms (prio 1) |
| **DIP Switches (TCA9555)** | `src/robot/subsystems/DipSwitchSubsystem.h` | DONE | `/mcu_robot/dip_switches` | 500ms (prio 1) |
| **LED** | `src/robot/subsystems/LEDSubsystem.h` | DONE | `/mcu_robot/led/set_all` (sub) | 50ms (prio 1) |
| **Localization** | N/A | NOT STARTED (EKF) | N/A | N/A |
| **Light Sensor (BH1750)** | N/A | NOT STARTED | N/A | N/A |
| **IR Subsystem** | N/A (robot-side) | NOT STARTED | N/A | N/A |

### Drive Subsystem Details

The drive subsystem code is **complete but not wired into RobotLogic.h**:
- `DriveSubsystem.h/cpp` - Main subsystem wrapper
- `RobotDriveBase.h/cpp` - Tank drive with 4 modes (MANUAL, VELOCITY_DRIVE, POSE_DRIVE, TRAJECTORY_DRIVE)
- `MotorDriver.h/cpp` - HAL-abstracted PWM/direction motor control
- `EncoderDriver.h/cpp` - Quadrature encoder reading

**Blockers**:
- Motor pin assignments undefined (TODO in RobotLogic.h)
- Encoder pin assignments undefined
- `RobotConfig.h` has PLACEHOLDER values marked "NOT ACCURATE CHANGE LATER!!!"
  - `TRACK_WIDTH = 10.0f` (wrong)
  - `WHEEL_DIAMETER = 3.25f` (wrong)
  - `RAW_TICKS_PER_REVOLUTION = 3` (wrong)

### Intake State Machine

```
IDLE ──startIntake()──► SPINNING ──IR trigger──► CAPTURED
                           │                        │
                           │ 3s timeout             │ stopIntake()
                           ▼                        ▼
                        JAMMED                    IDLE
                                                    ▲
EJECTING ──500ms──────────────────────────────────────┘
```
- Motor: PWM pin 3, direction pin 4
- IR breakbeam: pin 5 (rising-edge duck detection)
- Fully autonomous (runs without ROS)

### Hardware Drivers

| Driver | Chip | Status | Notes |
|--------|------|--------|-------|
| BNO085Driver | BNO085 IMU | DONE | Wire1, 50Hz quaternion/gyro/accel |
| I2CPowerDriver | INA219 | DONE | Wire0 via mux ch0 |
| I2CMuxDriver | TCA9548A | DONE | Wire0, channel selection |
| TCA9555Driver | TCA9555 | DONE | Wire0, 16-bit GPIO, newly added |
| PCA9685Driver | PCA9685 | DONE | Wire2, dual instances (0x40, 0x41) |
| PCA9685Manager | N/A | DONE | Double-buffered write to multiple PCA9685s |
| TOFDriver | VL53L0X | DONE | Wire0 |
| I2CBusLock | N/A | DONE | RAII mutex per bus (Wire0/1/2) |

### Control Algorithms (lib/control/)

| Algorithm | Status | Tests |
|-----------|--------|-------|
| PID Controller | DONE | 16 tests |
| Trapezoidal Motion Profile | DONE | 22 tests |
| S-Curve Motion Profile | DONE | 25 tests |
| Trajectory Controller (Pure Pursuit) | DONE | 22 tests |
| Arm Kinematics (2-link FK/IK) | DONE | 20 tests |
| Tank Drive Localization | DONE | 40+ tests |

---

## 4. Minibot

### MCU Subsystem (on Teensy41 main robot)

**File**: `src/robot/subsystems/MiniRobotSubsystem.h/cpp`
**Status**: DONE - Production ready

**State Machine**:
```
IDLE ──startMission()──► DRIVING_TO_TARGET ──arrival──► AT_TARGET
  ▲                          │                             │
  │                     timeout/comms fail                  │
  │                          ▼                             │
  └──────stop()───────── ERROR ◄───────────────────────────┘
```

**Communication**: I2C to ESP32 at address 0x42
- `CMD_DRIVE (0x01)`: target_x, target_y (floats)
- `CMD_STOP (0x02)`: immediate stop
- `CMD_RETURN (0x03)`: home_x, home_y (floats)

**Safety**: 1s comms timeout, 30s mission timeout, arrival detection at 0.15m

### Minibot ESP32 Firmware

**Status**: NOT STARTED

**Planned Subsystems** (from user spec):
- [ ] UWB Subsystem (for localization)
- [ ] MPU6050 Gyro Subsystem
- [ ] LED Indicators
- [ ] Heartbeat
- [ ] DriveBase (tank drive)

**TODO**:
- [ ] Create `mcu_ws/src/minibot/` source directory
- [ ] Create `minibot` PlatformIO environment in platformio.ini
- [ ] Implement I2C slave command receiver (address 0x42)
- [ ] Implement minibot drive base (motor control)
- [ ] Implement MPU6050 gyro integration
- [ ] Implement UWB tag for positioning
- [ ] Test crater loop navigation autonomy

---

## 5. Drone

### Firmware Status: NOT STARTED

**Current Code**: `mcu_ws/src/drone/main.cpp` - **Empty placeholder** (just `setup()` and `loop()` stubs)

**PlatformIO Environment**: `[env:drone]` extends `esp32_base` (no micro-ROS currently)

**Planned Subsystems** (from user spec):
- [ ] DroneControl (flight controller: twist or position modes)
- [ ] BNO085 IMU Subsystem
- [ ] VL53L1X Height Subsystem
- [ ] Heartbeat
- [ ] LED Indicators
- [ ] IR Subsystem (transmit antenna colors to Earth)
- [ ] UWB Subsystem (if readings are good enough for position mode)

**TODO**:
- [ ] Define drone hardware (flight controller, motors, ESCs, frame)
- [ ] Decide communication protocol (serial, I2C, ESP-NOW to main robot)
- [ ] Implement basic motor control / ESC interface
- [ ] Implement flight stabilization (PID on IMU feedback)
- [ ] Implement height hold (VL53L1X TOF sensor)
- [ ] Implement takeoff/land state machine
- [ ] Implement IR transmission subsystem
- [ ] Integrate with main robot orchestration (launch/land commands)

---

## 6. ROS2 Software Stack

**Distribution**: ROS2 Jazzy | **Build**: colcon | **Container**: Docker (prod/dev stages)

### Package Status

| Package | Status | Description |
|---------|--------|-------------|
| **secbot_autonomy** | IMPLEMENTED | 7 task state machines, 50Hz orchestration |
| **secbot_navigation** | IMPLEMENTED | D* Lite path planning + pure pursuit control |
| **secbot_vision** | PARTIAL | Duck HSV detection done; antenna LED detection **STUBBED** |
| **secbot_fusion** | MINIMAL | Simple DriveBase → Odometry pass-through, **no actual EKF** |
| **secbot_uwb** | IMPLEMENTED | Trilateration with outlier rejection, multi-beacon support |
| **secbot_sim** | IMPLEMENTED | Full MCU subsystem emulation with physics |
| **secbot_bridge_i2c** | STUBBED | Only headers and config; no implementation |
| **secbot_health** | STUBBED | Placeholder only |
| **secbot_tf** | STUBBED | Config exists; implementation empty |
| **secbot_msgs** | DONE | TaskStatus, DuckDetection, DuckDetections |
| **mcu_msgs** | DONE | 19 msg types, 5 services, shared with MCU |

### secbot_autonomy - Task Details

**Node**: `autonomy_node` (C++)
**Topics**: Publishes `autonomy/task_status`, subscribes to `autonomy/task_command`

| Task | Implementation | State Machine |
|------|---------------|---------------|
| AntennaAlign | DONE | APPROACHING → ALIGNING → SUCCEEDED/FAILED |
| ButtonPress | DONE | SETTLE → PRESS_HOLD → RELEASE_HOLD (x3) |
| CrankTurn | STUBBED | Minimal implementation |
| CraterEntry | DONE | APPROACH_RIM → DESCEND → DWELL → EXIT |
| FlagPlant | DONE | SETTLE → UNLATCH → POST_DROP → DONE |
| KeypadEnter | DONE | SETTLE → MOVE_TO_KEY → PRESS → RELEASE (per digit) |
| PressureClear | STUBBED | Minimal implementation |

### secbot_navigation

**Node**: `pathing_node` (C++)
- **Algorithm**: D* Lite with incremental replanning
- **Path Smoothing**: Removes redundant waypoints
- **Trajectory**: Grid → world coords with speed profiles
- **Control**: Pure pursuit with configurable lookahead
- **Config**: `arena_layout.yaml` (field definition), `nav.yaml` (planning params)
- **Topics**: Subscribes `/odom`, `/goal_pose`; publishes `/cmd_vel`, `/global_path`, `drive_base/trajectory`

### secbot_vision

**Node**: `detector_node` (Python)
- **Duck Detection**: HSV color filtering → contour detection → bounding boxes
- **Pipeline**: BGR→HSV → mask → morphological ops → contours → scoring
- **Publishes**: `/detected_objects`, `/duck_detections`, `/vision/debug_image`
- **Antenna LED Detector**: **STUBBED** (just a placeholder comment)

### secbot_uwb

**Node**: `positioning_node` (C++)
- **Algorithm**: Least-squares trilateration (Eigen solver)
- **Features**: 2D/3D modes, outlier rejection, covariance scaling
- **Config**: `beacons.yaml` (beacon positions, IDs, types)
- **Subscribes**: `mcu_uwb/ranging`; publishes per-tag pose topics

### secbot_sim - MCU Subsystem Simulator

**Node**: `mcu_subsystem_sim` (C++)
- Emulates ALL 9 MCU subsystems for Gazebo simulation
- Drive physics: PWM → wheel velocity → encoder ticks, S-curve profiles, PID
- Supports VELOCITY_DRIVE, POSE_DRIVE, TRAJECTORY_DRIVE modes
- Publishes all the same topics as real MCU firmware
- 100Hz physics, 50Hz drive/IMU publish

### Message Definitions Summary

**mcu_msgs** (19 messages, 3 services):
- State: McuState, BatteryHealth, IntakeState, ArmSubsystem, DroneState, MiniRobotState
- Sensor: UWBRanging, UWBRange, RC, AntennaMarker, RobotInputs
- Command: DriveCommand, DriveBase, ArmCommand, MiniRobotControl, DroneControl, LedColor, IRCommand
- Services: ArmControl, OLEDControl, LCDAppend, SetServo, SetMotor

**secbot_msgs** (3 messages, 2 services, 2 actions):
- Messages: TaskStatus, DuckDetection, DuckDetections
- Services: SetTask, StartStop
- Actions: ApproachTarget, NavigatePath

---

## 7. UWB Positioning System

### Architecture

```
Beacon 1 (ID=10, anchor) ◄──── DW3000 TWR ────► Robotcomms (ID=12, tag)
Beacon 2 (ID=11, anchor) ◄──── DW3000 TWR ────► Robotcomms (ID=12, tag)
Beacon 3 (ID=13, anchor) ◄──── DW3000 TWR ────► Robotcomms (ID=12, tag)
                                                        │
                                                  micro-ROS WiFi
                                                        │
                                                  Raspberry Pi
                                                  (secbot_uwb)
                                                        │
                                                  Trilateration
                                                        │
                                                  /uwb/robot_pose
```

### Component Status

| Component | Status | Notes |
|-----------|--------|-------|
| **UWB Driver (DW3000)** | DONE | `lib/uwb/UWBDriver.h/cpp`, 4-stage TWR protocol |
| **UWB Subsystem** | DONE | `lib/subsystems/UWBSubsystem.h/cpp`, TAG mode only publishes |
| **Beacon Firmware** | DONE | `src/beacon/machines/BeaconLogic.h`, anchor responders |
| **Robotcomms Firmware** | DONE | `src/robotcomms/machines/RobotcommsLogic.h`, tag initiator |
| **ROS2 Positioning** | DONE | `secbot_uwb`, trilateration with outlier rejection |
| **Beacon Deployment** | NOT DONE | Physical mechanism to place beacons at field corners |

### UWB IDs
- Beacon 1: ID=10 (corner)
- Beacon 2: ID=11 (corner)
- Beacon 3: ID=13 (robot-deployed or fixed)
- Robotcomms Tag: ID=12 (on robot)

### Ranging Protocol (DW3000 Two-Way Ranging)
1. Tag sends ranging request frame
2. Anchor responds with timing data
3. Tag sends final frame with round-trip info
4. Anchor calculates distance, sends RT-info
- Rate: ~20Hz initiation, 10Hz publish
- Timeout: 100ms per anchor

---

## 8. Field Elements

**Platform**: ESP32 | **Communication**: ESP-NOW broadcast | **Status**: ALL IMPLEMENTED

| Element | PIO Environment | Task | Hardware |
|---------|----------------|------|----------|
| **Button** | `field-button` | Press 3x | Button pin 5, RGB LED, status LED |
| **Crank** | `field-crank` | Full rotation | Rotary encoder pins 25/33, RGB LED |
| **Pressure** | `field-pressure` | Place/remove duck | Force sensor pin 34 (ADC1), RGB LED |
| **Keypad** | `field-keypad` | Enter password | 4x3 matrix keypad, RGB LED |
| **Earth** | `field-earth` | Receive IR colors | IR receiver pin 15, ESP-NOW verify |
| **Controller** | `field-controller` | Menu/timer/status | I2C LCD 20x4, joystick, buttons |

### Field Communication Flow
```
Antennas (button/crank/pressure/keypad)
    │
    ├── ESP-NOW broadcast ──► Controller (status updates, 1Hz)
    ├── ESP-NOW broadcast ──► Earth (color reports)
    └── IR NEC emission ──► Earth (antenna ID + color)

Controller
    └── ESP-NOW broadcast ──► All elements (RESET, START, CYCLE_DISPLAY)

Earth
    └── ESP-NOW ──► Controller (COLOR_RESULT scoring)
```

### IR Protocol (NEC encoding, 38kHz carrier)
- Address = antenna ID (high nibble): 0x00, 0x30, 0x50, 0x60
- Command = color code: RED=0x09, GREEN=0x0A, BLUE=0x0C, PURPLE=0x0F

---

## 9. DevOps & Deployment

### Docker Environment

**Dockerfile**: Multi-stage build (base → dev/prod)
- **base**: ros:jazzy-ros-base + PlatformIO + micro-ROS + cloudflared
- **dev**: + Gazebo Harmonic + RViz2 + GDB (for PC development)
- **prod**: Minimal runtime only (for robot Raspberry Pi)
- **Current**: `.env` set to `BUILD_TARGET=prod`

**docker-compose.yml**:
- Mounts: ros2_ws/src, mcu_ws, scripts
- Named volumes for build artifacts (prevent root-ownership issues)
- Hardware access: `/dev`, `/sys`
- Init-bootstrap service for volume ownership setup

### CI/CD Pipeline (Gitea Actions)

| Workflow | Trigger | Purpose | Status |
|----------|---------|---------|--------|
| `build-ws.yml` | PR/push to master | Docker build + colcon build | WORKING |
| `test-mcu.yml` | PR touching mcu_ws | Native unit tests (253+ tests) | WORKING |
| `platformio-robot.yml` | PR touching mcu_ws | Build ALL PIO environments | WORKING |
| `pi-deploy.yml` | Push to `prod` branch | Full deployment to Raspberry Pi | WORKING |
| `prod-push.yml` | Push to `master` | Auto-sync master → prod | WORKING |
| `auto-format.yml` | PR labeled `format` | clang-format auto-commit | WORKING |
| `commit-linter.yml` | All PRs | Conventional commit validation | WORKING |

### Deployment Flow (pi-deploy.yml)

```
1. Push to `prod` or `CD-testing` branch
2. Ubuntu runner: Build robot + robotcomms firmware → upload artifacts
3. Raspberry Pi runner (rpi-ros2):
   a. Flash BOOTLED (GPIO 17) during deployment
   b. Pull latest code, download MCU artifacts
   c. Docker rebuild if Dockerfile changed
   d. Smart MCU flash (only if mcu_ws changed)
   e. Run deploy-all.py (stage artifacts + flash + colcon build)
   f. Cleanup dangling images, stop LED
```

### Key Scripts

| Script | Purpose |
|--------|---------|
| `scripts/flash_mcu.sh` | Flash Teensy + ESP32 with retry (5 attempts) |
| `scripts/start_robot.sh` | colcon build (sequential) |
| `scripts/deploy-all.py` | Master orchestrator: stage artifacts + flash + build + launch |
| `scripts/export_urdf.sh` | Export URDF/STL from Onshape CAD |
| `scripts/initialize-env.sh` | One-time `.env` setup |

### WiFi / Network

- **SSID**: UCFIEEEBot | **Password**: goodlife
- **Network**: 192.168.4.x/24
- **Pi AP Mode**: Creates own WiFi network (hostapd/dnsmasq or NetworkManager AP)
- **micro-ROS Agent**: UDP port 8888 on Pi
- **ESP32 micro-ROS WiFi**: Connects to UCFIEEEBot, targets agent at 192.168.4.1:8888

### OLED Deployment Display

The SSD1306 OLED (128x64) on the robot can display deployment status via ROS2:
- **Service**: `/mcu_robot/lcd/append` (LCDAppend.srv) - append text lines
- **Topic**: `/mcu_robot/lcd/scroll` (Int8) - scroll viewport
- Ring buffer: 128 lines, 21 chars wide, 8 visible lines
- Thread-safe, 20Hz refresh

---

## 10. Communication Protocols

| Protocol | Transport | Devices | Purpose |
|----------|-----------|---------|---------|
| **micro-ROS (Serial)** | UART 921600 | Teensy41 ↔ Pi | Robot sensor/command data |
| **micro-ROS (WiFi)** | UDP 8888 | ESP32s ↔ Pi | UWB ranging, beacons |
| **UWB TWR (DW3000)** | SPI/RF | Tag ↔ Anchors | Distance measurement |
| **ESP-NOW** | WiFi broadcast | Field elements | Status/commands |
| **I2C** | Wire0/1/2 | Teensy ↔ sensors/drivers | Sensor data, GPIO, PWM |
| **I2C** | Wire | Teensy ↔ ESP32 (0x42) | Minibot commands |
| **IR NEC** | 38kHz IR | Antennas → Earth | Color verification |
| **IBUS** | Serial1 | FlySky RC → Teensy | Manual control |
| **SPI** | Software SPI | Teensy → SSD1306 | OLED display |

---

## 11. Test Infrastructure

### Native Tests (253+ tests, run on PC)

| Suite | Tests | What's Tested |
|-------|-------|---------------|
| test-math-pose2d | 19 | Pose2D math, angle normalization |
| test-math-vector2d | 33 | Vector2D operations, rotation |
| test-math-pose3d | 18 | 3D position + quaternion |
| test-control-pid | 16 | PID with anti-windup, derivative filtering |
| test-control-arm-kinematics | 20 | 2-link FK/IK, joint limits |
| test-control-trapezoidal-motion-profile | 22 | Accel-limited profile phases |
| test-control-scurve-motion-profile | 25 | Jerk-limited S-curve profile |
| test-control-trajectory-controller | 22 | Pure pursuit waypoint following |
| test-drive-tankdrivelocalization | 40+ | Tank drive odometry |
| test-utils-filters | 37 | Median, moving average, low-pass |
| test-utils-units | 43 | Length, speed, angular, temperature conversions |

### Hardware Tests (build-verified in CI)

| Test | Platform | What's Tested |
|------|----------|---------------|
| teensy-test-microros-subsystem | Teensy41 | micro-ROS connection lifecycle |
| teensy-test-battery-subsystem | Teensy41 | INA219/228 via mux, battery health |
| teensy-test-sensor-subsystem | Teensy41 | VL53L0X TOF sensors |
| teensy-test-oled-subsystem | Teensy41 | SSD1306 OLED display + ROS service |
| teensy-test-all-subsystems | Teensy41 | All subsystems integration (servo, motor, button, dip, LED + existing) |
| teensy-test-freertos | Teensy41 | TeensyThreads multitasking |
| teensy-test-microros-freertos | Teensy41 | micro-ROS + TeensyThreads combined |
| esp32-test-simple-wifi | ESP32 | Basic WiFi connectivity |
| esp32-test-microros-wifi | ESP32 | micro-ROS over WiFi UDP |

### Test Gaps

- No tests for: RC subsystem, arm servos, drive motors (individual unit tests — all-subsystems integration test exists)
- No integration tests (multi-MCU, UWB end-to-end)
- No vision system tests
- No autonomy/navigation tests
- No sensor fusion tests

---

## 12. Master TODO Tracker

### CRITICAL PATH (Must-have for competition)

#### Robot Firmware
- [ ] **Wire up drive subsystem** - Define motor/encoder pins, update RobotConfig.h with real values, uncomment in RobotLogic.h
- [ ] **Measure robot dimensions** - Track width, wheel diameter, ticks per revolution
- [ ] **Complete arm subsystem** - Implement ROS publisher/service (currently pseudo-code)
- [x] **Implement servo subsystem** - ~~New subsystem for crank 1-DOF servo~~ Generic ServoSubsystem with SetServo service
- [x] **Implement motor manager subsystem** - ~~New subsystem for keypad gear selector~~ Generic MotorManagerSubsystem with SetMotor service
- [x] **Implement GPIO subsystem** - ButtonSubsystem (TCA9555 port 1) + DipSwitchSubsystem (TCA9555 port 0)
- [x] **Implement LED subsystem** - LEDSubsystem with WS2812B + ROS subscription
- [ ] **Implement on-MCU localization EKF** - Fuse UWB + drive odometry in real-time
- [ ] **Implement IR subsystem (robot-side)** - Transmit antenna colors to Earth

#### Minibot
- [ ] **Create minibot firmware** - New ESP32 firmware (`src/minibot/`)
- [ ] **Implement I2C slave receiver** - Receive commands from Teensy (address 0x42)
- [ ] **Implement minibot drive base** - Motor control for crater navigation
- [ ] **Implement MPU6050 gyro** - Heading tracking for loop navigation
- [ ] **Test crater loop autonomy** - Enter, full loop, exit

#### Drone
- [ ] **Define drone hardware** - Flight controller, motors, ESCs
- [ ] **Implement flight control** - Motor mixing, PID stabilization
- [ ] **Implement height hold** - VL53L1X TOF for altitude
- [ ] **Implement takeoff/land** - State machine
- [ ] **Implement IR transmission** - Read LED colors, encode and transmit to Earth
- [ ] **Integrate with robot** - Launch/land commands via communication link

#### ROS2
- [ ] **Implement sensor fusion EKF** - Replace pass-through in secbot_fusion with real EKF (UWB + IMU + odometry)
- [ ] **Complete antenna LED detector** - Vision node to read antenna LED colors
- [ ] **Implement crank turn task** - Autonomy state machine (currently stubbed)
- [ ] **Implement pressure clear task** - Autonomy state machine (currently stubbed)
- [ ] **Implement secbot_bridge_i2c** - Or confirm it's unnecessary with current micro-ROS setup
- [ ] **Implement secbot_health** - System health monitoring
- [ ] **Implement secbot_tf** - Static transform broadcaster for robot frame tree
- [ ] **Complete launch files** - Many launch files are placeholder comments only

#### Mechanical / Integration
- [ ] **UWB beacon deployment mechanism** - Physical way to place beacons at field corners
- [ ] **Minibot deployment from intake** - Mechanical release of minibot
- [ ] **Drone mounting/launch** - Physical mount on robot + launch mechanism

### IMPORTANT (Should-have)

#### DevOps
- [ ] **Move WiFi credentials to Gitea Secrets** - Currently hardcoded in platformio.ini
- [ ] **Move Onshape credentials to Gitea Secrets** - Currently in .env file
- [ ] **Enable ARM64 Docker builds** - Currently disabled due to cmake issue
- [ ] **Automate micro-ROS rebuild on mcu_msgs changes** - Currently manual clean_microros
- [ ] **Add OLED deployment output** - Script to push build status to SSD1306 via `/mcu_robot/lcd/append`
- [ ] **Implement firmware version tagging** - Track what's deployed on robot

#### Testing
- [ ] **Add RC subsystem hardware test** - Verify FlySky IBUS parsing
- [ ] **Add arm subsystem hardware test** - Verify PCA9685 servo control
- [ ] **Add drive subsystem hardware test** - Verify motor/encoder operation
- [ ] **Add UWB end-to-end test** - Verify tag-anchor ranging pipeline
- [ ] **Add vision system tests** - Unit tests for duck detection
- [ ] **Add navigation integration tests** - Verify path planning + following

### NICE-TO-HAVE

- [ ] Duck collection strategy optimization (path planning for duck pickup)
- [ ] Deployment rollback capability (keep prior firmware version)
- [ ] Real-time telemetry dashboard (web UI showing robot state)
- [ ] Battery level alerts (low voltage warning via LED/OLED)
- [ ] Competition timer integration (3-minute countdown awareness)
- [ ] Field element auto-detection (vision-based beacon finding)

---

## Appendix: Build Commands Quick Reference

```bash
# Enter Docker container
docker compose up -d && docker compose exec devcontainer bash

# MCU builds (inside container)
pio run -e robot              # Teensy41 main robot
pio run -e robotcomms         # ESP32 comms bridge (UWB tag)
pio run -e beacon1            # UWB beacon ID=10
pio run -e beacon2            # UWB beacon ID=11
pio run -e drone              # Drone (placeholder)
pio run -e field-button       # Field element
pio run -e field-crank        # Field element
pio run -e field-earth        # Field element
pio run -e field-keypad       # Field element
pio run -e field-pressure     # Field element
pio run -e field-controller   # Field controller

# Run native tests
pio test -e test-math-pose2d
pio test -e test-control-pid
./mcu_ws/scripts/run_all_mcu_tests.sh  # All tests

# ROS2 builds (inside container)
source /opt/ros/jazzy/setup.bash
cd /home/ubuntu/ros2_workspaces
colcon build
source install/setup.bash

# Simulation
ros2 launch secbot_sim sim.launch.py
ros2 launch secbot_autonomy sim_autonomy.launch.py

# Flash firmware
pio run -e robot --target upload
pio run -e robotcomms --target upload

# Clean micro-ROS (after mcu_msgs changes)
pio run -e robot -t clean_microros
pio run -e robotcomms -t clean_microros
```
