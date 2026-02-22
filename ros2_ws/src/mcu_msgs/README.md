# mcu_msgs

Custom ROS2 message and service definitions shared between the ROS2 workspace and the MCU firmware.

---

## OLEDControl service

Controls the SSD1306 128×64 OLED display on the robot over micro-ROS.
Service: `/mcu_robot/oled_control`
Type: `mcu_msgs/srv/OLEDControl`

### Zones

| Constant    | Value | Area              |
|-------------|-------|-------------------|
| ZONE_FULL   | 0     | Entire 128×64     |
| ZONE_TOP    | 1     | Upper half 128×32 |
| ZONE_BOTTOM | 2     | Lower half 128×32 |

### Modes

| Constant    | Value | Description                  |
|-------------|-------|------------------------------|
| MODE_TEXT   | 0     | UTF-8 text, default 5×7 font |
| MODE_BITMAP | 1     | Raw 1-bpp pixel data         |

---

### Calling from the ROS2 container

Enter the container:
```bash
docker compose exec devcontainer bash
```

**Write text to the bottom half:**
```bash
ros2 service call /mcu_robot/oled_control mcu_msgs/srv/OLEDControl \
  "{zone: 2, mode: 0, text: 'Hello\nfrom ROS2!'}"
```

**Write text to the top half:**
```bash
ros2 service call /mcu_robot/oled_control mcu_msgs/srv/OLEDControl \
  "{zone: 1, mode: 0, text: 'Status: OK'}"
```

**Write text to the full screen:**
```bash
ros2 service call /mcu_robot/oled_control mcu_msgs/srv/OLEDControl \
  "{zone: 0, mode: 0, text: 'SEC26 Robot\nAll systems go'}"
```

**Notes:**
- `\n` advances to the next line within the zone (4 lines per half, 21 chars wide).
- The micro-ROS agent must be running and the robot connected before calling.
- The top and bottom zones are independent; writing to one does not clear the other.
