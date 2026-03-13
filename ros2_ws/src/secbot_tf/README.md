# secbot_tf

Static transform publishers for the robot's TF frame hierarchy (camera, TOF sensors, arm mount points).

## Status

**Scaffolding.** Config files and launch files exist but frame definitions are not yet populated.

## Intended Frame Hierarchy

```
map -> odom -> base_link -> camera_link
                         -> tof_front
                         -> tof_rear
                         -> arm_base
```

## Contents

| Directory | Contents |
|-----------|----------|
| `config/` | `frames.yaml` (real hardware), `frames_sim.yaml` (simulation) |
| `launch/` | `tf_static.launch.py`, `sim_tf_static.launch.py` |

## Launch Files

| File | Description |
|------|-------------|
| `tf_static.launch.py` | Real hardware transforms |
| `sim_tf_static.launch.py` | Simulation transforms |

## Usage

```bash
# Launch static transforms
ros2 launch secbot_tf tf_static.launch.py

# View TF tree
ros2 run tf2_tools view_frames

# Check specific transform
ros2 run tf2_ros tf2_echo odom base_link
```
