# Calibration & Configuration Verification Report

This report verifies the existence, location, and contents of the configuration files used for rotational and encoder-based movement.

## 1. Centralized Configuration

The primary source of truth for motion parameters is:
**Path:** `ros2_ws/src/my_robot_description/config/motion_config.yaml`

| Parameter               | Value (SI Units) | Notes             |
| :---------------------- | :--------------- | :---------------- |
| `track_width`           | 0.303022 m       | ~11.93 inches     |
| `wheel_diameter`        | 0.08255 m        | ~3.25 inches      |
| `encoder_ticks_per_rev` | 104.0            |                   |
| `gear_ratio`            | 0.6              |                   |
| `max_velocity`          | 0.6096 m/s       | ~24 inches/second |
| `max_angular_velocity`  | 3.0 rad/s        |                   |

## 2. Parameter Application

### MCU Subsystem Simulator

**Launch File:** `secbot_sim/launch/mcu_sim_secbot.launch.py`
The launch file dynamically loads the YAML values and converts them to **inches** (internal MCU standard) before passing them to the `mcu_subsystem_sim` node.

### Calibration Scripts

**Script:** `secbot_sim/scripts/motion_calibration_test.py`
The script performs the following at startup:

1. Locates `my_robot_description` share directory.
2. Loads `config/motion_config.yaml`.
3. Falls back to hardcoded defaults (which match the YAML) if the file is missing or unreadable.

## 3. Movement Styles Verification

### Rotational Movement

- **Open-Loop (Timed):** Uses `default_turn_speed` and `settle_time` from the YAML to calculate command duration.
- **Closed-Loop (IMU):** If enabled via `--closed-loop`, it overrides timing and monitors the IMU yaw topic for the target threshold.

### Encoder-Based Movement (Feedback)

- **Measurement Logic:** The `MotionCalibrationNode` computes distance by averaging the 4-wheel encoder deltas from `/joint_states`.
- **Formula:** `dist = 0.5 * (dl + dr) * radius`, where `dl/dr` are rotations in radians extracted from the simulation.

---

> [!IMPORTANT]
> Both the simulator and the and calibration scripts are synchronized to the same YAML file, ensuring that discrepancies found during testing are likely due to physics/friction rather than configuration mismatches.
