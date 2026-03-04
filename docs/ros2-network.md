# ROS2 Network Configuration

How to expose the robot's ROS2 graph to external devices (laptops, field computers) over WiFi or Ethernet.

## Problem

By default Docker uses bridge networking, which isolates the container from the Pi's physical interfaces (`eth0`, `wlan0`). External devices on the same network cannot discover ROS2 nodes because DDS multicast never reaches them.

## Solution

Two changes fix this:

1. **Host networking** — the container shares the Pi's actual network interfaces directly.
2. **FastDDS configured for all interfaces** — DDS multicasts on `eth0` and `wlan0` instead of only `lo`.

## Files

| File | Purpose |
|------|---------|
| `.env` | Sets `NETWORK_MODE_CONFIG=host` (default for Pi/Linux) |
| `docker-compose.yml` | Passes DDS env vars and mounts `config/` into the container |
| `config/dds/fastdds_robot.xml` | FastDDS profile: UDPv4 on all interfaces, large socket buffers |
| `scripts/container-entrypoint.sh` | Exports DDS env vars before any ROS2 process starts |
| `scripts/ros2-join-network.sh` | Helper for external devices to join the graph |

## Environment Variables

| Variable | Value | Effect |
|----------|-------|--------|
| `ROS_LOCALHOST_ONLY` | `0` | Allow inter-host discovery |
| `ROS_AUTOMATIC_DISCOVERY_RANGE` | `SUBNET` | Limit discovery to local subnet (not whole internet) |
| `FASTRTPS_DEFAULT_PROFILES_FILE` | `/home/ubuntu/config/dds/fastdds_robot.xml` | Load custom DDS profile |

## Network Topology

The Pi has two interfaces. External devices connect through a router on the same LAN — not directly to the Pi.

```
[Teensy / ESP32s]
       │ serial / UDP 8888
       ▼
   [Pi eth0] ──► [Router] ◄── [Laptop via Ethernet or WiFi]
   [Pi wlan0 AP] ◄────────── [Laptop via UCFIEEEBot WiFi]
```

| Interface | Address | Used for |
|-----------|---------|---------|
| `wlan0` | `192.168.4.1` | WiFi AP — laptops connect directly here |
| `eth0` | DHCP (router) | Wired LAN through router |

## Pi Setup

The `.env` file defaults to host networking. No manual change needed on the Pi.

```bash
# Restart the container to apply (run on host, not inside container)
docker compose down
docker compose up -d
```

Verify from inside the container:

```bash
docker compose exec devcontainer bash
ros2 topic list   # should show all /mcu_robot/... topics
```

## Joining from an External Device

### WiFi AP (192.168.4.x subnet)

Connect your laptop to the `UCFIEEEBot` WiFi network, then:

```bash
source /path/to/sec26/scripts/ros2-join-network.sh 192.168.4.1
ros2 topic list
ros2 topic echo /mcu_robot/heartbeat
```

If you don't have the repo, set the vars manually:

```bash
export ROS_LOCALHOST_ONLY=0
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
export ROS_STATIC_PEERS=192.168.4.1
ros2 topic list
```

### Router LAN (Ethernet or WiFi through a router)

If your device connects to the same router as the Pi (wired or wireless), you're on the same LAN subnet. DDS multicast usually works automatically:

```bash
source /path/to/sec26/scripts/ros2-join-network.sh <pi_eth_ip>
ros2 topic list
```

If the router blocks multicast (some do), `ROS_STATIC_PEERS` in the helper script falls back to unicast discovery directly to the Pi's IP, bypassing multicast entirely.

## Windows Dev Setup (VSCode + WSL2 + Docker)

**Prerequisites:**
- Docker Desktop with WSL2 backend enabled and integration turned on for your distro
- VSCode with the [Remote Development](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.vscode-remote-extensionpack) extension pack
- Host networking turned on in Docker Desktop

### Daily workflow

1. Open a WSL2 terminal and navigate to the repo:
   ```bash
   cd /mnt/c/Users/<username>/path/to/SEC26
   docker compose up -d
   ```
2. Confirm `.env` has:
   ```
   NETWORK_MODE_CONFIG=host
   DISPLAY_CONFIG=${DISPLAY}
   ```
3. VSCode → `Remote-WSL: Connect to WSL` → `Remote-Containers: Attach to Running Container` → select `sec26-devcontainer`

## FastDDS Profile

`config/dds/fastdds_robot.xml` replaces the default DDS transport with a custom UDPv4 descriptor:

- **Empty `interfaceWhiteList`** — binds to all interfaces (eth0 + wlan0)
- **`sendBufferSize=1MB`** — handles burst traffic from 6+ MCU devices
- **`receiveBufferSize=4MB`** — prevents packet drops under high topic load
- **`useBuiltinTransports=false`** — disables the default transport so only this profile is used

## Discovery Flow

```
Laptop on WiFi AP (192.168.4.x)
  └── DDS multicast → 239.255.0.1 on wlan0 subnet
      └── Pi wlan0 (192.168.4.1) receives it
          └── FastDDS responds → ROS2 graph joined ✓

Laptop on router LAN (eth0 or WiFi through router)
  └── DDS multicast → router forwards within subnet (usually)
      └── Pi eth0 receives it → ROS2 graph joined ✓
  └── OR: ROS_STATIC_PEERS → unicast directly to Pi IP
      └── Works even if router blocks multicast ✓
```
