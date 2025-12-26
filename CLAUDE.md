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
  - `src/robotcomms/`: ESP32 communication bridge
  - `src/beacon/`: UWB beacon nodes (differentiated by `BEACON_ID` build flag)
  - `src/drone/`: Drone controller
  - `src/field/`: Field element controllers (button, crank, earth, keypad, pressure)
  - `src/test/`: Test programs
- **Libraries**:
  - `lib/subsystems/`: Core robot subsystem abstractions (RobotManager, McuSubsystem, TimedSubsystem, UWBSubsystem)
  - `lib/robot/`, `lib/sensors/`, `lib/control/`, `lib/ir/`: Specific drivers and subsystem implementations
  - External dependencies defined in `platformio.ini` (SEC-Base-Classes, Adafruit sensors, etc.)

### ROS2 Workspace (`ros2_ws/`)

ROS2 packages using Jazzy distribution:

- `secbot_autonomy`: High-level autonomy and task planning
- `secbot_navigation`: Navigation stack integration
- `secbot_vision`: Computer vision for object detection
- `secbot_fusion`: Sensor fusion
- `secbot_uwb`: Ultra-wideband positioning
- `secbot_bridge_i2c`: Hardware bridge for I2C communication
- `secbot_health`: System health monitoring
- `secbot_tf`: Transform frames
- `secbot_sim`: Gazebo simulation
- `secbot_msgs`: Custom ROS2 message definitions
- `mcu_msgs`: Shared messages between MCU and ROS2 (symlinked to `mcu_ws/extra_packages/mcu_msgs`)

## Development Commands

**IMPORTANT**: All development commands (PlatformIO, ROS2, colcon, etc.) MUST be run inside the Docker container. Do NOT run them on the host machine.

### Docker Environment

All development happens inside Docker containers defined in `docker-compose.yml`. The environment uses multi-stage builds controlled by `.env`:

- `BUILD_TARGET=dev`: Includes Gazebo, RViz, GDB, GUI tools (for PC/laptop development)
- `BUILD_TARGET=prod`: Minimal runtime only (for robot deployment)

**Start development container** (run on host):
```bash
docker compose up -d
```

**Enter the container** (run on host):
```bash
docker compose exec devcontainer bash
```

Once inside the container, you can run all PlatformIO and ROS2 commands.

**Stop and clean** (run on host):
```bash
docker compose down
```

### MCU Development (PlatformIO)

All PlatformIO commands must be run from `/home/ubuntu/mcu_workspaces/sec26mcu` inside the container.

**List all environments**:
```bash
pio project config
```

**Build specific environment**:
```bash
pio run -e <environment>
```

Examples:
```bash
pio run -e robot              # Teensy41 main robot
pio run -e robotcomms         # ESP32 comms bridge
pio run -e beacon1            # UWB beacon with ID=10
pio run -e beacon2            # UWB beacon with ID=11
pio run -e drone              # Drone controller
pio run -e field-button       # Field button element
pio run -e field-pressure     # Field pressure sensor
```

**Upload firmware**:
```bash
pio run -e <environment> --target upload
```

**Clean build**:
```bash
pio run -e <environment> --target clean
```

**Clean micro-ROS** (when micro-ROS messages are outdated):
```bash
# Clean ESP32 micro-ROS (for robotcomms, beacons)
pio run -e robotcomms -t clean_microros

# Clean Teensy micro-ROS (for robot)
pio run -e robot -t clean_microros
```
Note: After cleaning micro-ROS, the next build will automatically regenerate the micro-ROS libraries with updated message definitions from `extra_packages/mcu_msgs`.

**Monitor serial output**:
```bash
pio device monitor -e <environment>
```

**Flash robot (automated script)**:
```bash
/home/ubuntu/scripts/flash_mcu.sh
```
This script handles flashing both Teensy (robot) and ESP32 (robotcomms) with retries and prebuilt artifact support.

### ROS2 Development

All ROS2 commands must be run from `/home/ubuntu/ros2_workspaces` inside the container after sourcing ROS2:

```bash
source /opt/ros/jazzy/setup.bash
```

**Build workspace**:
```bash
colcon build
```

**Build specific package**:
```bash
colcon build --packages-select <package_name>
```

**Source workspace after building**:
```bash
source install/setup.bash
```

**Run launch files**:
```bash
ros2 launch <package_name> <launch_file>
```

Examples:
```bash
ros2 launch secbot_sim sim.launch.py                    # Start Gazebo simulation
ros2 launch secbot_autonomy sim_autonomy.launch.py      # Autonomy in simulation
ros2 launch secbot_vision vision.launch.py              # Vision system
```

**Build and run robot (automated script)**:
```bash
/home/ubuntu/scripts/start_robot.sh
```

### Deployment

**Deploy all systems to robot**:
```bash
python3 /home/ubuntu/scripts/deploy-all.py
```

This script:
1. Stages prebuilt firmware artifacts if available
2. Builds ROS2 workspace
3. Flashes Teensy and ESP32 firmware
4. Launches ROS2 nodes

## Architecture Notes

### Robot Firmware Architecture (Teensy41)

The robot firmware (`src/robot/`) uses a subsystem-based architecture:

- **RobotManager**: Manages update timing for all subsystems using configurable timers (MS_1000, MS_200, MS_50, MS_20, etc.)
- **RobotObject**: Wraps drivers and subsystems with timing configuration
- **MCUSubsystem**: State machine for robot lifecycle (init → arm → begin → update/pause → reset)
- **Subsystems**: Battery, Sensors (TOF), Arm, RC, Heartbeat, etc.
- **micro-ROS**: Provides ROS2 communication via serial or network transport

Entry point: `src/robot/main.cpp` includes `machines/RobotLogic.h` which instantiates all subsystems and wires callbacks.

### PlatformIO Configuration

The `platformio.ini` uses inheritance for hardware bases:

- `[esp32_base]`: Standard ESP32 configuration
- `[teensy_base]`: Teensy41 configuration
- `[esp32_microros]`: ESP32 + micro-ROS (extends esp32_base + microros_base)
- `[teensy_microros]`: Teensy41 + micro-ROS (extends teensy_base + microros_base)

Concrete environments use `build_src_filter` to select specific source files and `extends` to inherit base configurations.

### Micro-ROS Integration

Micro-ROS libraries are stored in `mcu_ws/libs_external/` (separate for esp32 and teensy) and mounted as Docker volumes. The distro is configured to match ROS2 Jazzy. Transport defaults to serial (`MICRO_ROS_TRANSPORT_ARDUINO_SERIAL`).

### Message Sharing

`mcu_msgs` package is shared between ROS2 and MCU workspaces via symlink:
- Source: `ros2_ws/src/mcu_msgs`
- Linked to: `mcu_ws/extra_packages/mcu_msgs`

## Common Workflows

### Adding a new field element

1. Create source file in `mcu_ws/src/field/<name>.cpp`
2. Add environment in `platformio.ini`:
   ```ini
   [env:field-<name>]
   extends = esp32_base
   build_src_filter = +<field/field-<name>.cpp>
   ```
3. Build: `pio run -e field-<name>`

### Adding a new robot subsystem

1. Create header/implementation in `mcu_ws/lib/subsystems/` or `mcu_ws/src/robot/subsystems/`
2. Instantiate in `mcu_ws/src/robot/machines/RobotLogic.h`
3. Wire callbacks and register with RobotManager
4. Add RobotObject with appropriate TimerConfig
5. Build robot: `pio run -e robot`

### Testing in simulation

1. Build ROS2 workspace: `colcon build`
2. Source: `source install/setup.bash`
3. Launch simulation: `ros2 launch secbot_sim sim.launch.py`
4. Launch sim versions of other nodes (e.g., `sim_autonomy.launch.py`)

## Testing

### MCU Test Architecture

The MCU workspace uses PlatformIO's Unity test framework with two test types:

**Native Tests** (`test-*` environments):
- Run on PC without hardware (no Arduino framework)
- Test math/control code (PID, motion profiles, trajectory, filters, units, kinematics, pose/vector math)
- Fast execution, integrated into CI/CD
- Location: `mcu_ws/test/test_*/`

**MCU Hardware Tests** (`teensy-test-*`, `esp32-test-*` environments):
- Run on physical hardware with connected sensors/peripherals
- Test subsystems (Battery, Sensors, micro-ROS)
- Require hardware setup (some need multiple MCUs)
- Location: `mcu_ws/src/test/`

**Running tests**:
```bash
# Run all native tests
pio test

# Run specific native test
pio test -e test-control-pid

# Build MCU hardware test (requires upload to hardware)
pio run -e teensy-test-battery-subsystem
pio run -e teensy-test-battery-subsystem --target upload

# Run all tests (automated)
./mcu_ws/scripts/run_all_mcu_tests.sh
```

**Naming convention**:
- Native: `test-<category>-<component>` (e.g., `test-control-pid`)
- Hardware: `teensy-test-<subsystem>` or `esp32-test-<subsystem>`

See `mcu_ws/test/README.md` for detailed test documentation.
