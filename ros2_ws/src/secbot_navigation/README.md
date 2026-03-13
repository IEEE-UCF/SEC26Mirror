# secbot_navigation

Path planning (D* Lite) and trajectory tracking (pure pursuit) for robot navigation.

## Nodes

### pathing_node

Single node that handles path planning, smoothing, and trajectory following.

## ROS2 Interface

### Publishers

| Topic | Message Type | QoS | Rate | Description |
|-------|-------------|-----|------|-------------|
| `/cmd_vel` | `geometry_msgs/Twist` | 10 | 10 Hz | Velocity commands (legacy) |
| `drive_base/command` | `mcu_msgs/DriveBase` | 10 | 10 Hz | Drive commands (DRIVE_VECTOR mode) |
| `drive_base/trajectory` | `nav_msgs/Path` | Reliable, transient_local | On plan | Planned path trajectory |
| `/global_path` | `nav_msgs/Path` | 1 | On plan | RViz visualization |
| `/nav/goal_reached` | `std_msgs/Bool` | 10 | Event | Goal reached signal |

### Subscribers

| Topic | Message Type | QoS | Description |
|-------|-------------|-----|-------------|
| `/odometry/filtered` | `nav_msgs/Odometry` | 10 | Robot pose feedback (topic configurable) |
| `/goal_pose` | `geometry_msgs/PoseStamped` | 10 | Dynamic goal input |

### TF Lookups

Listens to `planning_frame` (default: "odom") -> `base_frame` (default: "base_link").

### Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `use_sim` | bool | false | Use simulation config |
| `use_sim_time` | bool | false | Use /clock for timing |
| `config_file` | string | "" | Path to nav.yaml |
| `arena_file` | string | "" | Path to arena_layout.yaml |
| `planning_frame` | string | "odom" | Frame for path planning |
| `odom_topic` | string | "/odometry/filtered" | Odometry input topic |
| `control_output` | string | "cmd_vel" | Output mode |
| `robot_radius` | double | 0.07 | Robot safety radius (m) |
| `speed` | double | 0.5 | Default navigation speed (m/s) |
| `max_v` | float | 1.0 | Max linear velocity (m/s) |
| `max_w` | float | 1.7 | Max angular velocity (rad/s) |
| `slowdown_dist` | float | 3.0 | Distance to start slowing (m) |
| `pos_tol` | float | 0.45 | Position tolerance (m) |
| `advance_tol` | float | 0.45 | Advance tolerance (m) |
| `min_v_near_goal` | float | 0.15 | Minimum speed near goal (m/s) |
| `lookahead` | double | 1.0 | Pure pursuit lookahead (m) |
| `replan_hz` | double | 3.0 | Replanning frequency |
| `arrive_dist` | double | 0.40 | Arrival threshold (m) |
| `arrive_yaw_deg` | double | 10.0 | Heading tolerance (degrees) |

## Planning Algorithm

- **D* Lite**: Incremental path planner with dynamic replanning
- **Path Smoother**: Removes unnecessary waypoints via Bresenham line checks
- **Pure Pursuit Controller**: Follows path with curvature-based steering

## Config Files

| File | Purpose |
|------|---------|
| `config/nav.yaml` | Real robot parameters |
| `config/nav_sim.yaml` | Simulation parameters |
| `config/arena_layout.yaml` | Grid map (0=free, 1=obstacle), dimensions, origin |

## Launch Files

| File | Description |
|------|-------------|
| `nav.launch.py` | Real robot navigation |
| `sim_nav.launch.py` | Simulation navigation |

## Usage Examples

```bash
# Launch navigation
ros2 launch secbot_navigation nav.launch.py

# Send goal via topic
ros2 topic pub --once /goal_pose geometry_msgs/PoseStamped \
  '{header: {frame_id: "odom"}, pose: {position: {x: 1.0, y: 0.5}}}'

# Monitor planned path
ros2 topic echo /global_path

# Check if goal reached
ros2 topic echo /nav/goal_reached

# Monitor drive commands being sent
ros2 topic echo drive_base/command

# Check planning rate
ros2 topic hz /global_path
```
