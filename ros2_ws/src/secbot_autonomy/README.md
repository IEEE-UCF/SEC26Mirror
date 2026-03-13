# secbot_autonomy

Task state machine orchestration and mission-level sequencing for the SEC26 competition. Contains 7 task implementations and a mission sequencer that coordinates the full competition run.

## Nodes

### autonomy_node

Owns and runs individual task state machines at 50 Hz.

### mission_node

Mission-level sequencer coordinating all tasks, drive commands, and subsystem control.

### drive_test_node

Drive integration testing using Teensy internal PID (DRIVE_GOAL mode).

## ROS2 Interface

### autonomy_node

#### Publishers

| Topic | Message Type | Rate | Description |
|-------|-------------|------|-------------|
| `autonomy/task_status` | `secbot_msgs/TaskStatus` | 50 Hz | Task execution status (task_id, ok, error_msg) |

#### Subscribers

| Topic | Message Type | Description |
|-------|-------------|-------------|
| `autonomy/task_command` | `std_msgs/UInt8` | External task command (task ID 0-7) |
| `autonomy/antenna_target` | `std_msgs/UInt8` | Set antenna target for align task |

#### Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `step_rate_hz` | double | 50.0 | Task stepping frequency |

### mission_node

#### Publishers

| Topic | Message Type | Description |
|-------|-------------|-------------|
| `drive_base/command` | `mcu_msgs/DriveBase` | Drive commands (DRIVE_VECTOR/DRIVE_GOAL) |
| `/autonomy/task_command` | `std_msgs/UInt8` | Task ID to execute |
| `/autonomy/antenna_target` | `std_msgs/UInt8` | Antenna ID for align task |
| `/mcu_minibot/cmd_vel` | `geometry_msgs/Twist` | Minibot movement command |
| `/arm_command` | `mcu_msgs/ArmCommand` | Servo/actuator commands |
| `/mcu_robot/intake/command` | `mcu_msgs/IntakeCommand` | Intake rail control |
| `/mcu_drone/control` | `mcu_msgs/DroneControl` | Drone control |

#### Subscribers

| Topic | Message Type | Description |
|-------|-------------|-------------|
| `drive_base/status` | `mcu_msgs/DriveBase` | Robot pose feedback |
| `/autonomy/task_status` | `secbot_msgs/TaskStatus` | Task completion feedback |
| `/mcu_minibot/state` | `mcu_msgs/MiniRobotState` | Minibot state |
| `/mcu_drone/state` | `mcu_msgs/DroneState` | Drone state |
| `/mcu_robot/inputs` | `mcu_msgs/RobotInputs` | Button/DIP switch feedback |
| `/mcu_robot/intake/state` | `mcu_msgs/IntakeState` | Intake position/limits |
| `/duck_detections` | `secbot_msgs/DuckDetections` | Vision duck detections |
| `/antenna_marker` | `mcu_msgs/AntennaMarker` | LED color from antennas |

#### Services (Server)

| Service | Type | Description |
|---------|------|-------------|
| `/mission_start` | `std_srvs/Trigger` | Trigger mission start |

### drive_test_node

#### Publishers

| Topic | Message Type | Description |
|-------|-------------|-------------|
| `drive_base/command` | `mcu_msgs/DriveBase` | DRIVE_GOAL goals for testing |

#### Subscribers

| Topic | Message Type | Description |
|-------|-------------|-------------|
| `drive_base/status` | `mcu_msgs/DriveBase` | Robot pose |
| `/mcu_robot/imu/data` | `sensor_msgs/Imu` | Gyro for drift measurement |

#### Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `distance` | double | 0.5 | Goal distance (meters) |
| `turn_angle` | double | 1.571 | Turn goal (radians, 90 deg) |
| `with_turn` | bool | false | Include CW/CCW turns |
| `loop` | bool | false | Infinite back-and-forth |
| `reverse` | bool | false | Drive backward to goals |
| `pause_time` | double | 1.0 | Pause between goals (seconds) |
| `goal_tolerance` | double | 0.03 | Position tolerance (meters) |
| `goal_timeout` | double | 10.0 | Timeout per goal (seconds) |
| `calibrate_time` | double | 3.0 | Gyro calibration duration |

## Task IDs

| ID | Task | Description |
|----|------|-------------|
| 1 | antenna_align | Approach antenna, align to target face |
| 2 | button_press | 3x forward/hold/backward presses |
| 3 | crank_turn | Rotate crank >= 540 deg via motor |
| 4 | crater_entry | Drive into crater, touch line, reverse |
| 5 | flag_plant | Release flag via latch servo |
| 6 | keypad_enter | Type reset code "73738#" |
| 7 | pressure_clear | Extend intake rail, capture duck, retract |

## Launch Files

| File | Description |
|------|-------------|
| `autonomy.launch.py` | autonomy_node + mission_node (real robot) |
| `mission.launch.py` | mission_node only |
| `sim_autonomy.launch.py` | Simulation configuration (stub) |

## Usage Examples

```bash
# Launch full autonomy
ros2 launch secbot_autonomy autonomy.launch.py

# Send task command (e.g., button press = 2)
ros2 topic pub --once /autonomy/task_command std_msgs/UInt8 '{data: 2}'

# Set antenna target
ros2 topic pub --once /autonomy/antenna_target std_msgs/UInt8 '{data: 3}'

# Trigger mission start
ros2 service call /mission_start std_srvs/srv/Trigger

# Monitor task status
ros2 topic echo /autonomy/task_status

# Run drive test with turning and looping
ros2 run secbot_autonomy drive_test_node --ros-args \
  -p loop:=true -p with_turn:=true -p distance:=1.0
```
