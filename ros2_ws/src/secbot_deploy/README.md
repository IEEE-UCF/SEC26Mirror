# secbot_deploy

IPC bridge between the MCU deploy button press and the host-side `deploy-orchestrator.py`. Watches for MCU trigger events, writes trigger files, polls status, and sends phase feedback to the MCU for OLED/LED display.

## Nodes

### deploy_node (C++)

Bridges MCU deploy trigger to orchestrator via shared filesystem IPC.

## ROS2 Interface

### Publishers

| Topic | Message Type | QoS | Rate | Description |
|-------|-------------|-----|------|-------------|
| `/mcu_robot/deploy/status` | `std_msgs/String` | best_effort | ~2 Hz | Phase updates: "phase:X" or "msg:Y" |
| `/mcu_robot/lcd/append` | `std_msgs/String` | best_effort | ~2 Hz | Human-readable messages for OLED |

### Subscribers

| Topic | Message Type | QoS | Description |
|-------|-------------|-----|-------------|
| `/mcu_robot/deploy/trigger` | `std_msgs/String` | best_effort | MCU button press: "start:robot", "start:*:force", "cancel" |

### Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `deploy_dir` | string | `/home/ubuntu/scripts/.deploy` | Shared IPC directory |

## Trigger Format (from MCU)

| Trigger | Description |
|---------|-------------|
| `start:robot` | Deploy main robot firmware from prod branch |
| `start:robot:force` | Force deploy (overwrite local changes) |
| `start:teensy-test-all-subsystems` | Deploy test firmware |
| `cancel` | Cancel pending deploy |

## Orchestrator Phases

| Phase | Description |
|-------|-------------|
| `pulling` | Git pulling prod branch |
| `building_mcu` | Building MCU firmware |
| `building_ros` | Building ROS2 workspace |
| `flashing_teensy` | Flashing Teensy via USB |
| `flashing_esp32` | OTA flashing ESP32 devices |
| `complete` | Deploy finished successfully |
| `error` | Deploy failed |

## Launch Files

| File | Description |
|------|-------------|
| `deploy.launch.py` | Start deploy_node with default IPC dir |

## Usage Examples

```bash
# Launch deploy bridge
ros2 launch secbot_deploy deploy.launch.py

# Monitor deploy status
ros2 topic echo /mcu_robot/deploy/status

# Simulate deploy trigger (for testing)
ros2 topic pub --once /mcu_robot/deploy/trigger std_msgs/String '{data: "start:robot"}'

# Cancel deploy
ros2 topic pub --once /mcu_robot/deploy/trigger std_msgs/String '{data: "cancel"}'

# View OLED feedback
ros2 topic echo /mcu_robot/lcd/append
```
