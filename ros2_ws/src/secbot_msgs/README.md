# secbot_msgs

Custom ROS2 message, service, and action definitions for high-level autonomy and task orchestration.

## Messages

### TaskStatus

Current task execution state, published by `autonomy_node`.

```
uint8 task_id       # Active task ID (0-7)
bool ok             # Operating normally?
string error_msg    # Empty if ok, description if error
```

### DuckDetections

Array of duck detections from the vision pipeline.

```
std_msgs/Header header
DuckDetection[] detections
```

### DuckDetection

Single duck detection with bounding box and confidence.

```
int32 id                    # Detection ID
string label                # Color name (e.g., "yellow")
float32 x, y, w, h         # Bounding box (pixels)
float32 center_x, center_y # Center coordinates (pixels)
float32 area                # Contour area (pixels^2)
float32 confidence          # Detection confidence (0-100%)
```

### AntennaDetections

Array of antenna LED detections.

```
std_msgs/Header header
AntennaDetection[] detections
```

### AntennaDetection

Single antenna LED detection with color.

```
int32 id
string label        # Color name
string color        # Detected color
float32 x, y, w, h
float32 center_x, center_y
float32 area, confidence
```

## Actions

### NavigatePath

Navigate along a waypoint path.

```
# Goal
bool start
---
# Result
bool success
---
# Feedback
float32 progress    # 0.0-1.0
```

### ApproachTarget

Navigate to a single target position.

### ReadBeaconColor

Query beacon LED color for the antenna task.

## Services

### SetTask

Switch the active autonomy task.

```
# Request
uint8 task_id
---
# Response
bool accepted
```

### StartStop

Start or stop task execution.

## Usage Examples

```bash
# Monitor task status
ros2 topic echo /autonomy/task_status

# Monitor duck detections
ros2 topic echo /duck_detections

# Switch to task 1 (antenna_align)
ros2 service call /autonomy/set_task secbot_msgs/srv/SetTask '{task_id: 1}'

# Start navigation action
ros2 action send_goal /navigate secbot_msgs/action/NavigatePath '{start: true}'

# Check message definition
ros2 interface show secbot_msgs/msg/DuckDetection
ros2 interface show secbot_msgs/msg/TaskStatus
```

## Building

```bash
cd /home/ubuntu/ros2_workspaces
colcon build --packages-select secbot_msgs
source install/setup.bash
```
