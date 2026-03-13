# secbot_health

System health monitoring and MCU heartbeat watchdog. Tracks MCU liveness, battery voltage, and ROS2 node health. Publishes standard diagnostics.

## Nodes

### health_node (C++)

Monitors MCU topics (heartbeat, state, battery) and ROS2 topics (odometry, autonomy). Publishes aggregated diagnostics at 2 Hz.

## ROS2 Interface

### Publishers

| Topic | Message Type | QoS | Rate | Description |
|-------|-------------|-----|------|-------------|
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | 10 | 2 Hz | Standard ROS diagnostics (5-6 status items) |
| `/system/ok` | `std_msgs/Bool` | 10 | 2 Hz | Aggregated boolean: all critical checks OK |

### Subscribers

| Topic | Message Type | QoS | Description |
|-------|-------------|-----|-------------|
| `/mcu_robot/heartbeat` | `std_msgs/String` | best_effort | MCU alive signal (1 Hz nominal) |
| `/mcu_robot/mcu_state` | `mcu_msgs/McuState` | best_effort | MCU lifecycle state |
| `/mcu_robot/battery_health` | `mcu_msgs/BatteryHealth` | best_effort | Battery data |
| `/odom` | `nav_msgs/Odometry` | 10 | Navigation output |
| `/autonomy/task_status` | `secbot_msgs/TaskStatus` | 10 | Autonomy task state |

### Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `use_sim` | bool | false | Skip MCU monitoring in simulation |
| `heartbeat_timeout_sec` | double | 1.0 | MCU heartbeat timeout |
| `mcu_state_timeout_sec` | double | 2.0 | MCU state timeout |
| `battery_timeout_sec` | double | 5.0 | Battery data timeout |
| `battery_warn_voltage` | double | 11.0 | Warning voltage threshold |
| `battery_error_voltage` | double | 10.5 | Critical voltage threshold |
| `odom_timeout_sec` | double | 3.0 | Odometry timeout |
| `autonomy_timeout_sec` | double | 3.0 | Autonomy timeout |
| `publish_rate_hz` | double | 2.0 | Diagnostic publish rate |

### Diagnostic Status Items

| Name | OK | WARN | ERROR |
|------|-----|------|-------|
| MCU Heartbeat | age < 1s | — | timeout or never received |
| MCU Lifecycle | RUNNING or ARMED | INIT* | INIT_FAIL |
| Battery | V > 11.0 | 10.5 < V <= 11.0 | V <= 10.5 |
| Navigation (/odom) | age < 3s | — | timeout |
| Autonomy | age < 3s | — | timeout |

## Launch Files

| File | Description |
|------|-------------|
| `health.launch.py` | Real hardware monitor |
| `sim_health.launch.py` | Sim mode (skips MCU topics) |

## Usage Examples

```bash
# Launch health monitor
ros2 launch secbot_health health.launch.py

# Monitor diagnostics
ros2 topic echo /diagnostics

# Check system OK flag
ros2 topic echo /system/ok

# View/change parameters
ros2 param list /health_node
ros2 param set /health_node battery_warn_voltage 10.8
```
