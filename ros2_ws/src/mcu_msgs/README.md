# mcu_msgs

Custom ROS2 message and service definitions shared between the ROS2 workspace and the MCU firmware.

---

## OLED display — serial terminal interface

The SSD1306 128×64 OLED on the robot behaves like a serial terminal.
Text is appended to a 128-line ring buffer; the display shows 8 lines at a time.
One writer at a time — last write wins.

### Interface

| Name                      | Type                           | Description                                          |
|---------------------------|--------------------------------|------------------------------------------------------|
| `/mcu_robot/lcd/append`  | service `mcu_msgs/LCDAppend`   | Append text; `\n` splits into lines. Returns `accepted`. |
| `/mcu_robot/lcd/scroll`  | topic `std_msgs/Int8`          | `-1` = scroll up (older), `+1` = scroll down (newer) |

### LCDAppend.srv

```
string text      # text to append (\n splits into separate lines)
---
bool accepted
```

### Limits

| Parameter     | Value | Notes                                           |
|---------------|-------|-------------------------------------------------|
| Lines visible | 8     | 64 px ÷ 8 px/row (text size 1)                 |
| Max line len  | 21    | 128 px ÷ 6 px/char — longer input is truncated  |
| Ring depth    | 128   | Oldest lines are overwritten when full           |

---

### Calling from the ROS2 container

Enter the container:
```bash
docker compose exec devcontainer bash
```

**Append a line:**
```bash
ros2 service call /mcu_robot/lcd/append mcu_msgs/srv/LCDAppend "text: 'Hello from ROS2'"
```

**Competition — write antenna colors after VIEW_LED_COLORS:**
```bash
ros2 service call /mcu_robot/lcd/append mcu_msgs/srv/LCDAppend \
  "text: 'A1:RED A2:BLU\nA3:GRN A4:PUR'"
```
Result on display (2 lines appended at bottom):
```
A1:RED A2:BLU
A3:GRN A4:PUR
```

**Scroll up to see older lines:**
```bash
ros2 topic pub --once /mcu_robot/lcd/scroll std_msgs/msg/Int8 "data: -1"
```

**Scroll down (back to live view):**
```bash
ros2 topic pub --once /mcu_robot/lcd/scroll std_msgs/msg/Int8 "data: 1"
```

**Jump back to live in one shot:**
```bash
ros2 topic pub --once /mcu_robot/lcd/scroll std_msgs/msg/Int8 "data: 127"
```

### Notes
- The micro-ROS agent must be running and the robot connected before calling.
- Appending does not reset the scroll position. Publish `data: 127` to jump back to live.
- The ring buffer overwrites the oldest line when 128 lines are exceeded.
- After adding `LCDAppend.srv`, rebuild micro-ROS: `pio run -e robot -t clean_microros && pio run -e robot`
