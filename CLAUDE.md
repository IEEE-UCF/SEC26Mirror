# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Shortcut Path

**`C:/Users/medla/SEC2026/SEC26`** is a symlink to the full OneDrive path. Use this shorter path when possible for convenience.

## Project Overview

SEC26 is the IEEE UCF SoutheastCon 2026 Hardware Competition robot repository. It contains firmware for multiple microcontrollers (field elements, robot, beacons, drone) and ROS2 software for robot autonomy, navigation, and vision.

## Repository Structure

The codebase is split into two workspaces:

- `mcu_ws/`: PlatformIO workspace for all microcontroller firmware (ESP32, Teensy41)
- `ros2_ws/`: ROS2 Jazzy workspace for high-level robot control and autonomy

### MCU Workspace (`mcu_ws/`)

- **Platforms**: ESP32 (field elements, beacons, drone) and Teensy41 (main robot controller)
- **Framework**: Arduino with PlatformIO build system
- **Micro-ROS**: Robot and beacons use micro-ROS for ROS2 communication (stored in `libs_external/`)
- **Source organization**:
  - `src/robot/`: Main Teensy41 robot firmware (uses micro-ROS)
  - `src/beacon/`: UWB beacon nodes (differentiated by `BEACON_ID` build flag)
  - `src/drone/`: Drone controller (flight controller, IMU, height, IR subsystems)
  - `src/minibot/`: Mini-robot ESP32 firmware (MinibotMotorDriver, MinibotDriveSubsystem, UWB tag, micro-ROS WiFi)
  - `src/field/`: Field element controllers (button, crank, earth, keypad, pressure)
  - `src/test/`: Hardware test programs
  - `src/platform/`: Platform support files (e.g., `atomic_stubs_arm.c`)
- **Robot subsystem headers** (`src/robot/subsystems/`):
  - `BatterySubsystem` — INA219 power monitoring
  - `ImuSubsystem` — BNO085 9-axis IMU
  - `RCSubsystem` — FlySky IBUS receiver
  - `OLEDSubsystem` — SSD1306 128x64 display (subscriptions for text append and scroll)
  - `SensorSubsystem` — VL53L0X time-of-flight distance
  - `ArmSubsystem` — Arm control with servo/encoder
  - `IntakeSubsystem` — Intake/ejection mechanism
  - `IntakeBridgeSubsystem` — Gear-and-rack mechanism for pressure plate duck retrieval
  - `MiniRobotSubsystem` — Mini-robot control
  - `ServoSubsystem` — PCA9685 servo manager with SetServo service
  - `MotorManagerSubsystem` — PCA9685 motor manager with SetMotor service
  - `ButtonSubsystem` — TCA9555 push button input with callbacks
  - `DipSwitchSubsystem` — TCA9555 DIP switch reading
  - `LEDSubsystem` — WS2812B addressable RGB LEDs
- **Robot pin/constant files** (`src/robot/`):
  - `RobotPins.h` — All Teensy 4.1 GPIO pin assignments and I2C addresses
  - `RobotConstants.h` — PCA9685 defaults (freq, channels)
  - `RobotConfig.h` — Drive kinematics constants (placeholder values)
- **Libraries**:
  - `lib/subsystems/`: Core subsystem abstractions (McuSubsystem lifecycle, UWBSubsystem, IMicroRosParticipant, TimedSubsystem)
  - `lib/robot/`: Hardware drivers (BNO085, TCA9555, PCA9685Driver/Manager, I2CMux, I2CPower, TOF, I2CBusLock, AnalogMux, AnalogRead)
  - `lib/drivers/`: Additional drivers (CD74HC4067 analog mux)
  - `lib/control/`: Control algorithms (PID, trapezoidal/S-curve motion profiles, trajectory controller, arm kinematics)
  - `lib/drive/`: Tank drive localization
  - `lib/math/`: Vector2D, Pose2D, Pose3D
  - `lib/uwb/`: UWB DW3000 driver
  - `lib/ir/`: IR NEC communication (ESP32 RMT-based)
  - `lib/field/`: Field element message definitions, ESP-NOW transport, shared protocol
  - `lib/hal/`: Hardware abstraction layer (NativeGPIO, PCA9685GPIO, MCP23017GPIO, CD74HC4067GPIO)
  - `lib/utils/`: Signal filters, unit conversions, duck tracker, math helpers
  - `lib/sensors/`: Photodiode signal conditioning
  - `lib/microros/`: MicrorosManager (agent lifecycle, participant registry, executor)

### ROS2 Workspace (`ros2_ws/`)

- `secbot_autonomy`: Task state machines (7 competition tasks), 50Hz orchestration (C++)
- `secbot_navigation`: D* Lite path planning + pure pursuit trajectory control (C++)
- `secbot_vision`: HSV color-based duck detection (Python), TF broadcasting, antenna LED detection (stubbed)
- `secbot_fusion`: Sensor fusion — currently simple DriveBase → Odometry pass-through (C++, ready for EKF)
- `secbot_uwb`: UWB trilateration positioning with outlier rejection (C++)
- `secbot_sim`: Gazebo Harmonic simulation with full MCU subsystem emulator (C++)
- `secbot_health`: System health monitoring and MCU heartbeat watchdog (C++)
- `secbot_bridge_i2c`: ROS ↔ Teensy I2C bridge with packet codec and fake Teensy for simulation (C++)
- `secbot_tf`: TF static transform configuration (launch files + YAML)
- `secbot_msgs`: Custom messages (TaskStatus, DuckDetection) and actions (NavigatePath, ApproachTarget)
- `mcu_msgs`: Shared MCU↔ROS2 messages (symlinked to `mcu_ws/extra_packages/mcu_msgs`)
- `my_robot_description`: URDF robot description for visualization

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

**Note**: On first run or after updating git submodules (e.g., `libs_external/`), you may need to recreate the Docker volume: `docker volume rm robot-setup_mcu_lib_external` then `docker compose up -d` to reseed.

### MCU Development (PlatformIO)

All PlatformIO commands run from `/home/ubuntu/mcu_workspaces/sec26mcu` inside the container.

```bash
pio run -e <environment>                  # Build specific environment
pio run -e <environment> --target upload  # Upload firmware
pio run -e <environment> --target clean   # Clean build
pio device monitor -e <environment>       # Monitor serial output (921600 baud)
```

Key environments:
```bash
pio run -e robot                          # Teensy41 main robot
pio run -e beacon1                        # UWB beacon ID=10
pio run -e beacon2                        # UWB beacon ID=11
pio run -e beacon3                        # UWB beacon ID=12
pio run -e minibot                        # Minibot ESP32 (drive + UWB)
pio run -e drone                          # Drone controller (ESP32)
pio run -e teensy-test-all-subsystems     # All subsystems integration test
pio run -e field-button                   # Field element (ESP32)
pio run -e field-crank
pio run -e field-earth
pio run -e field-keypad
pio run -e field-pressure
pio run -e field-controller
```

**Default build environments** (built by `pio run` with no `-e`): `robot`, `drone`, `minibot`, `beacon1`, `beacon2`, `beacon3`, `teensy-test-all-subsystems`.

**Clean micro-ROS** (required after changing `mcu_msgs` definitions):
```bash
pio run -e robot -t clean_microros && pio run -e robot
```
After cleaning, the next build auto-regenerates micro-ROS libraries from `extra_packages/mcu_msgs`.

**Flash robot (automated)**:
```bash
/home/ubuntu/scripts/flash_mcu.sh        # Flashes Teensy with retries
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
ros2 launch secbot_navigation sim_nav.launch.py
ros2 launch secbot_uwb uwb_positioning.launch.py
ros2 launch secbot_fusion fusion.launch.py
```

### Deployment

```bash
python3 /home/ubuntu/scripts/deploy-all.py             # Full deployment
python3 /home/ubuntu/scripts/deploy-all.py --skip-mcu   # Skip MCU flash
python3 /home/ubuntu/scripts/deploy-all.py --skip-ros   # Skip ROS build
/home/ubuntu/scripts/start_robot.sh                      # Build ROS2 only
```

## Testing

### Native Tests (run on PC, no hardware needed)

Location: `mcu_ws/test/test_*/` | Framework: Unity | Naming: `test-<category>-<component>`

```bash
pio test -e test-control-pid                    # Run single test suite
pio test -e test-math-vector2d                  # Another example
./mcu_ws/scripts/run_all_mcu_tests.sh           # Run all native + build hardware tests
```

Test suites (253+ tests total): `test-math-{pose2d,vector2d,pose3d}`, `test-control-{pid,arm-kinematics,trapezoidal-motion-profile,scurve-motion-profile,trajectory-controller}`, `test-drive-tankdrivelocalization`, `test-utils-{filters,units}`

### Hardware Tests (require physical MCU)

Location: `mcu_ws/src/test/` | Naming: `teensy-test-<subsystem>` or `esp32-test-<subsystem>`

```bash
pio run -e teensy-test-battery-subsystem --target upload  # Build + flash
pio device monitor -e teensy-test-battery-subsystem       # Watch output
pio run -e teensy-test-all-subsystems --target upload     # Flash all-subsystems test
```

Hardware test environments: `teensy-test-battery-subsystem`, `teensy-test-sensor-subsystem`, `teensy-test-oled-subsystem`, `teensy-test-all-subsystems`, `teensy-test-rc-subsystem`, `teensy-test-arm-servos`, `teensy-test-drive-motors`, `esp32-test-microros-wifi`, `esp32-test-simple-wifi`.

See `mcu_ws/test/README.md` for detailed test documentation.

## Architecture

### Robot Firmware (Teensy41) — Threading Model

Entry point: `src/robot/main.cpp` → includes `machines/RobotLogic.h` which instantiates all subsystems.

Each subsystem runs as an independent **TeensyThreads** task with configurable priority and update rate:

| Priority | Subsystem | Rate | Stack |
|----------|-----------|------|-------|
| 4 (highest) | micro-ROS Manager | — | 8192 |
| 3 | IMU | 10ms (100 Hz) | 2048 |
| 2 | Servo | 25ms (40 Hz) | 1024 |
| 2 | Motor Manager | 1ms (1000 Hz) | 1024 |
| 2 | UWB | 50ms (20 Hz) | 2048 |
| 2 | Arm, Intake | 20ms (50 Hz) | 1024 |
| 1 | OLED | 25ms (40 Hz) | 2048 |
| 1 | Battery | 100ms (10 Hz) | 1024 |
| 1 | Sensor (TOF) | 100ms (10 Hz) | 1024 |
| 1 | DIP Switch | 500ms (2 Hz) | 1024 |
| 1 | Button | 20ms (50 Hz) | 1024 |
| 1 | LED | 50ms (20 Hz) | 1024 |
| 1 | Heartbeat | 200ms (5 Hz) | 1024 |
| main loop | RC Receiver | 5ms delay | — |
| dedicated | PCA9685 PWM flush | 20ms | 1024 |

**Important hardware quirks:**
- **RC Receiver**: Must be polled from main `loop()`, not a TeensyThreads thread — IBusBM NOTIMER mode only works from the main thread context.
- **Motor Manager**: Runs at 1000 Hz to handle NFPShop brushless motor reverse-pulse timing (~3ms pulse every ~103ms at low duty to prevent controller fault).
- **OLED Display**: Uses hardware SPI1 (pins 26/27) — software SPI is unreliable under TeensyThreads preemption.
- **UWB Init**: Must call `SPI.begin()` before `g_uwb.init()`. The DW3000 library has a hardcoded `RST_PIN 27` in `DW3000Constants.h` that conflicts with OLED SPI1 SCK — `UWBDriver.cpp` skips `hardReset()` when no reset pin is wired to avoid this.

Subsystems implement `beginThreaded(stackSize, priority, updateRateMs)` and the `IMicroRosParticipant` interface (`onCreate`/`onDestroy`) for ROS2 lifecycle.

### I2C Bus Layout

```
Wire0 (pins 18/19): TCA9548A mux (0x70), TCA9555 GPIO (0x20), INA219 power (0x40 behind mux ch0)
Wire1 (pins 17/16): BNO085 IMU (0x4B), INT=41, RST=40
Wire2 (pins 24/25): PCA9685 #0 servos (0x40, OE=28), PCA9685 #1 motors (0x41, OE=29)
```

I2C bus access is mutex-protected via `I2CBusLock` (RAII pattern: `I2CBus::Lock lock(Wire);`).

### Teensy 4.1 Pin Map

See `src/robot/RobotPins.h` for full assignments. Key pins:
- **Motors**: GPIO 2-9 (reserved for motor outputs)
- **UWB SPI**: CS=10, MOSI=11, MISO=12, CLK=13
- **Display SPI1**: MOSI=26, SCK=27, CS=38, DC=37, RST=33 (hardware SPI1, not SPI0)
- **Misc GPIO**: WS2812B=35, RC RX=34 (Serial8), Mux Reset=23, Button INT=36
- **PCA9685 OE**: Servo=28, Motor=29

### Subsystem Lifecycle

All MCU subsystems inherit from `Classes::BaseSubsystem` with lifecycle: `init()` → `begin()` → `update()` (loop) / `pause()` → `reset()`. Subsystems that need ROS2 also implement `IMicroRosParticipant` and register with `MicrorosManager::registerParticipant()`.

The MicrorosManager supports up to 16 registered participants and initializes 10 executor handles (for subscriptions, services, and timers — publishers do not consume handles).

### micro-ROS Topic Namespace

All robot topics use `/mcu_robot/` prefix:
- `/mcu_robot/heartbeat` (String), `/mcu_robot/battery_health` (BatteryHealth), `/mcu_robot/imu/data` (Imu)
- `/mcu_robot/tof_distances` (Float32MultiArray), `/mcu_robot/rc` (RC), `/mcu_robot/intake/state` (IntakeState)
- `/mcu_robot/mini_robot/state` (MiniRobotState), `/mcu_robot/lcd/append` (subscription: String)
- `/mcu_robot/servo/state` (Float32MultiArray), `/mcu_robot/servo/set` (service: SetServo)
- `/mcu_robot/motor/state` (Float32MultiArray), `/mcu_robot/motor/set` (service: SetMotor)
- `/mcu_robot/buttons` (UInt8), `/mcu_robot/dip_switches` (UInt8)
- `/mcu_robot/led/set_all` (subscription: LedColor)
- UWB: `mcu_uwb/ranging` (from beacons/robot tag)
- Drive: `drive_base/status`, `drive_base/command` (currently commented out in RobotLogic.h)

Minibot topics use `/mcu_minibot/` prefix:
- `/mcu_minibot/cmd_vel` (subscription: geometry_msgs/Twist — differential drive mix)
- `/mcu_minibot/state` (MiniRobotState — mission state publisher)

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

ESP32 Beacons (×3)
──────────────────
mcu_uwb/ranging ───WiFi UDP──► secbot_uwb ──► /uwb/robot_pose
```

### UWB Positioning System

- **Beacons** (anchors): ID=10, ID=11, ID=12 deployed at field corners (ESP32 + DW3000)
- **Tag**: ID=13 on main robot (reserved), ranges to all anchors via DW3000 TWR
- **Transport**: ESP32 micro-ROS WiFi UDP to Pi, then `secbot_uwb` trilaterates position
- **Rate**: ~20Hz ranging initiation, 10Hz publish to ROS2
- **Trilateration**: Min 3 beacons (2D), outlier rejection at 2.0σ, max residual 0.5m

### PlatformIO Configuration Inheritance

```
[esp32_base]         [teensy_base]         [microros_base]       [native_test_base]
     │                    │                      │                      │
[esp32_microros]     [teensy_microros]           │               test-* envs
     │                    │                      │
[esp32_microros_wifi]  robot, teensy-test-*      │
     │                                           │
beacons, minibot ───────────────────────────────┘
```

Concrete environments use `build_src_filter` to select source files and `extends` to inherit bases. The `teensy_base` includes TeensyThreads and FastLED as default dependencies.

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

**Current message inventory** (21 msgs, 5 srvs):
- Messages: AntennaMarker, ArmCommand, ArmSusbsytem, BatteryHealth, DriveBase, DriveCommand, DroneControl, DroneState, IntakeBridgeCommand, IntakeBridgeState, IntakeState, IRCommand, LedColor, McuState, MiniRobotControl, MiniRobotState, RC, RobotInputs, UWBAnchorInfo, UWBRange, UWBRanging
- Services: ArmControl, LCDAppend, OLEDControl, SetServo, SetMotor

**When you add/modify a `.msg` or `.srv` file**: you must clean and rebuild micro-ROS:
```bash
pio run -e robot -t clean_microros && pio run -e robot
```

### Simulation

`secbot_sim` includes `mcu_subsystem_sim` — a C++ node that emulates all MCU subsystems with physics (drive kinematics, S-curve profiles, PID). It publishes the same topics as the real Teensy firmware, so all ROS2 nodes work identically in simulation. Uses Gazebo Harmonic (dev build only).

## Common Workflows

### Adding a new robot subsystem

1. Create header in `mcu_ws/src/robot/subsystems/` (header-only preferred for simple subsystems)
2. If it needs ROS2: implement `IMicroRosParticipant` (override `onCreate`/`onDestroy`)
3. Use setup pattern: `SubsystemSetup` class extending `Classes::BaseSetup` with configurable topic/service names
4. Instantiate in `mcu_ws/src/robot/machines/RobotLogic.h`
5. Register with micro-ROS manager: `g_mr.registerParticipant(&subsystem)`
6. Start threaded: `subsystem.beginThreaded(stackSize, priority, updateRateMs)`
7. If the subsystem has a `.cpp` file, add it to build_src_filter in relevant environments
8. Build: `pio run -e robot`

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
2. Update `ros2_ws/src/mcu_msgs/CMakeLists.txt` to include new files in `rosidl_generate_interfaces`
3. Rebuild ROS2: `colcon build --packages-select mcu_msgs`
4. Clean and rebuild micro-ROS for affected MCU targets (see above)

## CI/CD

Hosted on **Gitea** (not GitHub). Workflows in `.gitea/workflows/`:

| Workflow | Trigger | Purpose |
|----------|---------|---------|
| `test-mcu.yml` | PR touching mcu_ws or mcu_msgs | Runs all native PlatformIO unit tests |
| `platformio-robot.yml` | PR touching mcu_ws or mcu_msgs | Builds ALL non-test PlatformIO environments |
| `build-ws.yml` | PR/push to master touching ros2_ws | Docker image build + colcon build |
| `pi-deploy.yml` | Push to `prod` or `CD-testing` | Full deployment to Raspberry Pi robot |
| `prod-push.yml` | Push to `master` | Auto-syncs master → prod branch |
| `auto-format.yml` | PR labeled `format` | Runs clang-format, auto-commits |
| `commit-linter.yml` | All PRs | Enforces Conventional Commits |
| `test-multiarch.yml` | Manual dispatch only | Multi-architecture runner test |

Commits must follow **Conventional Commits** format (enforced by CI): `feat:`, `fix:`, `chore:`, `refactor:`, etc.

## Network Configuration

- **WiFi SSID**: UCFIEEEBot | **Password**: goodlife
- **Pi IP** (AP mode): 192.168.4.1 | **Subnet**: 192.168.4.x/24
- **micro-ROS agent**: UDP port 8888 on Pi
- **Teensy transport**: Serial (directly connected to Pi)
- **ESP32 transport**: WiFi UDP to Pi agent
- **Beacon IPs**: beacon1=192.168.4.20, beacon2=192.168.4.21, beacon3=192.168.4.22
- **Minibot IP**: 192.168.4.24
