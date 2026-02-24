# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

SEC26 is the IEEE UCF SoutheastCon 2026 Hardware Competition robot repository. It contains firmware for multiple microcontrollers (field elements, robot, beacons, drone) and ROS2 software for robot autonomy, navigation, and vision.

## Repository Structure

The codebase is split into two workspaces:

- `mcu_ws/`: PlatformIO workspace for all microcontroller firmware (ESP32, Teensy41)
- `ros2_ws/`: ROS2 Jazzy workspace for high-level robot control and autonomy

### MCU Workspace (`mcu_ws/`)

- **Platforms**: ESP32 (field elements, beacons, drone, robotcomms) and Teensy41 (main robot controller)
- **Framework**: Arduino with PlatformIO build system
- **Micro-ROS**: Robot and beacons use micro-ROS for ROS2 communication (stored in `libs_external/`)
- **Source organization**:
  - `src/robot/`: Main Teensy41 robot firmware (uses micro-ROS)
  - `src/robotcomms/`: ESP32 communication bridge (UWB tag)
  - `src/beacon/`: UWB beacon nodes (differentiated by `BEACON_ID` build flag)
  - `src/drone/`: Drone controller
  - `src/field/`: Field element controllers (button, crank, earth, keypad, pressure)
  - `src/test/`: Hardware test programs
- **Libraries**:
  - `lib/subsystems/`: Core subsystem abstractions (McuSubsystem lifecycle, UWBSubsystem, IMicroRosParticipant)
  - `lib/robot/`: Hardware drivers (BNO085, TCA9555, PCA9685, I2CMux, I2CPower, TOF, I2CBusLock)
  - `lib/control/`: Control algorithms (PID, motion profiles, trajectory controller, arm kinematics)
  - `lib/uwb/`: UWB DW3000 driver
  - `lib/ir/`: IR NEC communication (ESP32 RMT-based)
  - `lib/field/`: Field element message definitions and shared protocol
  - `lib/hal/`: Hardware abstraction layer for pin/PWM

### ROS2 Workspace (`ros2_ws/`)

- `secbot_autonomy`: Task state machines (7 competition tasks), 50Hz orchestration
- `secbot_navigation`: D* Lite path planning + pure pursuit trajectory control
- `secbot_vision`: HSV color-based duck detection (Python), antenna LED detection (stubbed)
- `secbot_fusion`: Sensor fusion (currently simple DriveBase → Odometry pass-through)
- `secbot_uwb`: UWB trilateration positioning with outlier rejection
- `secbot_sim`: Gazebo simulation with full MCU subsystem emulator
- `secbot_bridge_i2c`, `secbot_health`, `secbot_tf`: Stubbed/placeholder packages
- `secbot_msgs`: Custom messages (TaskStatus, DuckDetection)
- `mcu_msgs`: Shared MCU↔ROS2 messages (symlinked to `mcu_ws/extra_packages/mcu_msgs`)

## Development Commands

**IMPORTANT**: All development commands (PlatformIO, ROS2, colcon, etc.) MUST be run inside the Docker container. Do NOT run them on the host machine.

### Docker Environment

All development happens inside Docker containers defined in `docker-compose.yml`. The environment uses multi-stage builds controlled by `.env`:

- `BUILD_TARGET=dev`: Includes Gazebo, RViz, GDB, GUI tools (for PC/laptop development)
- `BUILD_TARGET=prod`: Minimal runtime only (for robot deployment)

```bash
docker compose up -d                      # Start container (on host)
docker compose exec devcontainer bash     # Enter container (on host)
docker compose down                       # Stop and clean (on host)
```

### MCU Development (PlatformIO)

All PlatformIO commands run from `/home/ubuntu/mcu_workspaces/sec26mcu` inside the container.

```bash
pio run -e <environment>                  # Build specific environment
pio run -e <environment> --target upload  # Upload firmware
pio run -e <environment> --target clean   # Clean build
pio device monitor -e <environment>       # Monitor serial output
```

Key environments:
```bash
pio run -e robot              # Teensy41 main robot
pio run -e robotcomms         # ESP32 comms bridge (UWB tag)
pio run -e beacon1            # UWB beacon ID=10
pio run -e beacon2            # UWB beacon ID=11
pio run -e drone              # Drone controller (ESP32)
pio run -e field-button       # Field element (ESP32)
pio run -e field-crank
pio run -e field-earth
pio run -e field-keypad
pio run -e field-pressure
pio run -e field-controller
```

**Clean micro-ROS** (required after changing `mcu_msgs` definitions):
```bash
pio run -e robot -t clean_microros        # Teensy micro-ROS
pio run -e robotcomms -t clean_microros   # ESP32 micro-ROS
```
After cleaning, the next build auto-regenerates micro-ROS libraries from `extra_packages/mcu_msgs`.

**Flash robot (automated)**:
```bash
/home/ubuntu/scripts/flash_mcu.sh        # Flashes both Teensy + ESP32 with retries
```

### ROS2 Development

All commands run from `/home/ubuntu/ros2_workspaces` inside the container.

```bash
source /opt/ros/jazzy/setup.bash
colcon build                              # Build all packages
colcon build --packages-select <pkg>      # Build specific package
source install/setup.bash                 # Source after building
ros2 launch <package> <launch_file>       # Run launch file
```

Example launches:
```bash
ros2 launch secbot_sim sim.launch.py
ros2 launch secbot_autonomy sim_autonomy.launch.py
ros2 launch secbot_vision vision.launch.py
```

### Deployment

```bash
python3 /home/ubuntu/scripts/deploy-all.py           # Full deployment
python3 /home/ubuntu/scripts/deploy-all.py --skip-mcu  # Skip MCU flash
/home/ubuntu/scripts/start_robot.sh                   # Build ROS2 only
```

## Testing

### Native Tests (run on PC, no hardware needed)

Location: `mcu_ws/test/test_*/` | Framework: Unity | Naming: `test-<category>-<component>`

```bash
pio test -e test-control-pid                    # Run single test suite
pio test -e test-math-vector2d                  # Another example
./mcu_ws/scripts/run_all_mcu_tests.sh           # Run all native + build hardware tests
```

Test suites: `test-math-{pose2d,vector2d,pose3d}`, `test-control-{pid,arm-kinematics,trapezoidal-motion-profile,scurve-motion-profile,trajectory-controller}`, `test-drive-tankdrivelocalization`, `test-utils-{filters,units}`

### Hardware Tests (require physical MCU)

Location: `mcu_ws/src/test/` | Naming: `teensy-test-<subsystem>` or `esp32-test-<subsystem>`

```bash
pio run -e teensy-test-battery-subsystem --target upload  # Build + flash
pio device monitor -e teensy-test-battery-subsystem       # Watch output
```

See `mcu_ws/test/README.md` for detailed test documentation.

## Architecture

### Robot Firmware (Teensy41) — Threading Model

Entry point: `src/robot/main.cpp` → includes `machines/RobotLogic.h` which instantiates all subsystems.

Each subsystem runs as an independent **TeensyThreads** task with configurable priority and update rate:

| Priority | Subsystem | Rate |
|----------|-----------|------|
| 4 (highest) | micro-ROS Manager | 10ms |
| 3 | IMU, RC Receiver | 20ms, 5ms |
| 2 | Arm, Intake | 20ms |
| 1 | OLED, Battery, Sensors, Heartbeat | 50-200ms |
| dedicated | PCA9685 PWM flush | 20ms |

Subsystems implement `beginThreaded(stackSize, priority, updateRateMs)` and the `IMicroRosParticipant` interface (`onCreate`/`onDestroy`) for ROS2 lifecycle.

### I2C Bus Layout

```
Wire0: TCA9548A mux (0x70), TCA9555 GPIO (0x20), INA219 power (0x40 behind mux ch0)
Wire1: BNO085 IMU (0x4A)
Wire2: PCA9685 #1 (0x40), PCA9685 #2 (0x41) — servos/PWM
```

I2C bus access is mutex-protected via `I2CBusLock` (RAII pattern: `I2CBus::Lock lock(Wire);`).

### Subsystem Lifecycle

All MCU subsystems inherit from `Classes::BaseSubsystem` with lifecycle: `init()` → `begin()` → `update()` (loop) / `pause()` → `reset()`. Subsystems that need ROS2 also implement `IMicroRosParticipant` and register with `MicrorosManager::registerParticipant()`.

### micro-ROS Topic Namespace

All robot topics use `/mcu_robot/` prefix:
- `/mcu_robot/heartbeat`, `/mcu_robot/battery_health`, `/mcu_robot/imu/data`
- `/mcu_robot/tof_distances`, `/mcu_robot/rc`, `/mcu_robot/intake/state`
- `/mcu_robot/mini_robot/state`, `/mcu_robot/lcd/append` (service)
- UWB: `mcu_uwb/ranging` (from robotcomms ESP32)
- Drive: `drive_base/status`, `drive_base/command` (currently commented out)

### ROS2 Data Flow

```
MCU (Teensy)                    Raspberry Pi (ROS2 Jazzy)
─────────────                   ──────────────────────────
                  micro-ROS
drive_base/status ──serial──► secbot_fusion ──► /odom/unfiltered
/mcu_robot/imu    ──serial──► (sensor data available to all nodes)
                              secbot_navigation ──► /cmd_vel ──► drive_base/command
                              secbot_autonomy (orchestrates tasks)
                              secbot_vision ──► /duck_detections

ESP32 (robotcomms)
─────────────────
mcu_uwb/ranging ───WiFi UDP──► secbot_uwb ──► /uwb/robot_pose
```

### UWB Positioning System

- **Beacons** (anchors): ID=10, ID=11 deployed at field corners, ID=13 (third anchor)
- **Robotcomms** (tag): ID=12 on main robot, ranges to all anchors via DW3000 TWR
- **Transport**: ESP32 micro-ROS WiFi UDP to Pi, then `secbot_uwb` trilaterates position
- **Rate**: ~20Hz ranging initiation, 10Hz publish to ROS2

### PlatformIO Configuration Inheritance

```
[esp32_base]         [teensy_base]         [microros_base]       [native_test_base]
     │                    │                      │                      │
[esp32_microros]     [teensy_microros]           │               test-* envs
     │                    │                      │
beacons, robotcomms  robot env                   │
                                                 │
[esp32_microros_wifi] ───────────────────────────┘
```

Concrete environments use `build_src_filter` to select source files and `extends` to inherit bases.

### Field Element Communication

Field elements (ESP32) communicate via **ESP-NOW** broadcast (not micro-ROS):
- Status updates: element → controller (1Hz)
- Commands: controller → all elements (RESET, START)
- Color reports: antennas → Earth (for IR verification)
- IR NEC protocol (38kHz): antennas emit `[antenna_id | color_code]` to Earth receiver

### Message Sharing Between MCU and ROS2

`mcu_msgs` is the shared message package:
- Source of truth: `ros2_ws/src/mcu_msgs`
- Symlinked to: `mcu_ws/extra_packages/mcu_msgs`

**When you add/modify a `.msg` or `.srv` file**: you must clean and rebuild micro-ROS on both platforms:
```bash
pio run -e robot -t clean_microros && pio run -e robot
pio run -e robotcomms -t clean_microros && pio run -e robotcomms
```

### Simulation

`secbot_sim` includes `mcu_subsystem_sim` — a C++ node that emulates all MCU subsystems with physics (drive kinematics, S-curve profiles, PID). It publishes the same topics as the real Teensy firmware, so all ROS2 nodes work identically in simulation.

## Common Workflows

### Adding a new robot subsystem

1. Create header/implementation in `mcu_ws/src/robot/subsystems/`
2. If it needs ROS2: implement `IMicroRosParticipant` (override `onCreate`/`onDestroy`)
3. Instantiate in `mcu_ws/src/robot/machines/RobotLogic.h`
4. Register with micro-ROS manager: `g_mr.registerParticipant(&subsystem)`
5. Start threaded: `subsystem.beginThreaded(stackSize, priority, updateRateMs)`
6. Build: `pio run -e robot`

### Adding a new field element

1. Create source file in `mcu_ws/src/field/<name>.cpp`
2. Add environment in `platformio.ini`:
   ```ini
   [env:field-<name>]
   extends = esp32_base
   build_src_filter = +<field/field-<name>.cpp>
   ```
3. Build: `pio run -e field-<name>`

### Adding/changing mcu_msgs

1. Edit `.msg`/`.srv`/`.action` files in `ros2_ws/src/mcu_msgs/`
2. Rebuild ROS2: `colcon build --packages-select mcu_msgs`
3. Clean and rebuild micro-ROS for affected MCU targets (see above)

## CI/CD

Hosted on **Gitea** (not GitHub). Workflows in `.gitea/workflows/`:

| Workflow | Trigger | Purpose |
|----------|---------|---------|
| `test-mcu.yml` | PR touching mcu_ws | Runs all native PlatformIO unit tests |
| `platformio-robot.yml` | PR touching mcu_ws | Builds ALL PlatformIO environments |
| `build-ws.yml` | PR/push to master | Docker image build + colcon build |
| `pi-deploy.yml` | Push to `prod` branch | Full deployment to Raspberry Pi robot |
| `prod-push.yml` | Push to `master` | Auto-syncs master → prod branch |
| `auto-format.yml` | PR labeled `format` | Runs clang-format, auto-commits |
| `commit-linter.yml` | All PRs | Enforces Conventional Commits |

Commits must follow **Conventional Commits** format (enforced by CI): `feat:`, `fix:`, `chore:`, `refactor:`, etc.

## Network Configuration

- **WiFi SSID**: UCFIEEEBot | **Password**: goodlife
- **Pi IP** (AP mode): 192.168.4.1 | **Subnet**: 192.168.4.x/24
- **micro-ROS agent**: UDP port 8888 on Pi
- **Teensy transport**: Serial (directly connected to Pi)
- **ESP32 transport**: WiFi UDP to Pi agent
