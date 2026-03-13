# secbot_sim

Full-fidelity simulation of the Teensy 4.1 robot firmware. Emulates all MCU subsystems with physics (drive kinematics, S-curve profiles, PID) so all ROS2 nodes work identically in simulation.

## Nodes

### mcu_subsystem_sim (C++)

Simulates 9 MCU subsystems: drive (with S-curve + PID + localization), battery, heartbeat, IMU, TOF, RC, intake, mini-robot, and MCU state. Uses the real MCU control libraries compiled as pure C++.

## ROS2 Interface

### Publishers

| Topic | Message Type | Rate | Description |
|-------|-------------|------|-------------|
| `drive_base/status` | `mcu_msgs/DriveBase` | 50 Hz | Robot odometry + drive state |
| `/mcu_robot/battery_health` | `mcu_msgs/BatteryHealth` | 1 Hz | Simulated battery (12.0V, 0A) |
| `/mcu_robot/heartbeat` | `std_msgs/String` | 5 Hz | Periodic alive message |
| `/mcu_robot/tof_distances` | `std_msgs/Float32MultiArray` | 10 Hz | Simulated TOF distances (4 values) |
| `/mcu_robot/rc` | `mcu_msgs/RC` | 20 Hz | RC receiver data (stub) |
| `/mcu_robot/intake/state` | `mcu_msgs/IntakeState` | 20 Hz | Intake mechanism state |
| `/mcu_robot/mini_robot/state` | `mcu_msgs/MiniRobotState` | 10 Hz | Mini-robot state (stub) |
| `/mcu_robot/mcu_state` | `mcu_msgs/McuState` | 50 Hz | MCU lifecycle state (RUNNING) |
| `/cmd_vel_out` | `geometry_msgs/Twist` | 50 Hz | Current velocity feedback |

### Subscribers

| Topic | Message Type | Description |
|-------|-------------|-------------|
| `drive_base/command` | `mcu_msgs/DriveBase` | Drive commands (VECTOR/GOAL/TRAJ modes) |
| `/cmd_vel_in` | `geometry_msgs/Twist` | Alternative velocity commands |
| `drive_base/trajectory` | `nav_msgs/Path` | Trajectory waypoints for TRAJECTORY mode |
| `/arm_command` | `mcu_msgs/ArmCommand` | Arm servo commands (stub) |
| `/intake_speed` | `std_msgs/Int16` | Intake motor speed |
| `/odom/ground_truth` | `nav_msgs/Odometry` | Gazebo ground-truth |
| `/joint_states` | `sensor_msgs/JointState` | Joint feedback |
| `/odometry/filtered` | `nav_msgs/Odometry` | EKF-fused odometry |

### Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `track_width` | double | 12.0 | Track width (inches) |
| `wheel_diameter` | double | 4.0 | Wheel diameter (inches) |
| `encoder_ticks_per_rev` | int | 2048 | Encoder resolution |
| `gear_ratio` | double | 1.0 | Motor gear ratio |
| `max_velocity` | double | 24.0 | Max velocity (in/s) |
| `max_angular_velocity` | double | 3.0 | Max angular velocity (rad/s) |
| `pid_kp/ki/kd` | double | 0.8/0.1/0.05 | Wheel PID gains |
| `num_tof_sensors` | int | 4 | Number of simulated TOF sensors |

## Control Pipeline (mirrors real firmware)

```
Command -> [S-Curve Profile] -> [Inverse Kinematics] -> [Wheel PID] -> Motor PWM
                                                              |
                                                        Physics sim
                                                              |
Odometry <- [Tank Drive Localization] <- Encoder ticks + IMU
```

## Launch Files

| File | Description |
|------|-------------|
| `sim.launch.py` | Full sim with Gazebo + all nodes |
| `mcu_sim.launch.py` | MCU simulator only |
| `sim_autonomy_secbot.launch.py` | Sim + autonomy |
| `sim_viz.launch.py` | Gazebo + RViz visualization |

## Usage Examples

```bash
# Full simulation
ros2 launch secbot_sim sim.launch.py

# MCU simulator only
ros2 launch secbot_sim mcu_sim.launch.py

# Monitor simulated drive state
ros2 topic echo drive_base/status

# Send velocity command
ros2 topic pub /cmd_vel_in geometry_msgs/Twist \
  '{linear: {x: 0.5}, angular: {z: 0.0}}'

# Send drive_base command (VECTOR mode)
ros2 topic pub --once drive_base/command mcu_msgs/msg/DriveBase \
  "{drive_mode: 0, goal_velocity: {linear: {x: 0.3}, angular: {z: 0.0}}}"

# Check simulated battery
ros2 topic echo /mcu_robot/battery_health

# Monitor heartbeat
ros2 topic hz /mcu_robot/heartbeat
```
