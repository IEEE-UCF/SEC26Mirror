# secbot_bridge_i2c

ROS2-to-Teensy I2C bridge with packet codec. Currently scaffolding — micro-ROS serial is the primary communication mechanism.

## Status

**In progress.** Source files contain placeholder implementations. The package structure and configuration files are in place but no functional nodes exist yet.

## Intended Purpose

Encode/decode ROS2 messages into I2C packets for direct Teensy communication as an alternative to micro-ROS serial transport.

## Contents

| Directory | Contents |
|-----------|----------|
| `src/` | Placeholder source: `i2c_bridge.cpp`, `fake_teensy_node.cpp`, `packet_codec.cpp`, `bridge_watchdog.cpp`, `rate_limiter.cpp` |
| `config/` | `bridge.yaml` (real hardware), `bridge_sim.yaml` (simulation) |
| `launch/` | `bridge.launch.py`, `sim_bridge.launch.py` |

## Launch Files

| File | Description |
|------|-------------|
| `bridge.launch.py` | Real hardware I2C bridge |
| `sim_bridge.launch.py` | Simulation bridge |
