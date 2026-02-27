# SEC26 Deployment System — Full Setup Guide

## Table of Contents

1. [Architecture Overview](#architecture-overview)
2. [Prerequisites](#prerequisites)
3. [Step 1: Pi WiFi Access Point Setup](#step-1-pi-wifi-access-point-setup)
4. [Step 2: Docker Container Setup](#step-2-docker-container-setup)
5. [Step 3: Install the Deploy Orchestrator Service](#step-3-install-the-deploy-orchestrator-service)
6. [Step 4: Remove the Old Button-Trigger Service](#step-4-remove-the-old-button-trigger-service)
7. [Step 5: Flash Teensy (First Time)](#step-5-flash-teensy-first-time)
8. [Step 6: Flash ESP32s via USB (First Time — OTA Bootstrap)](#step-6-flash-esp32s-via-usb-first-time--ota-bootstrap)
9. [Step 7: Verify OTA Connectivity](#step-7-verify-ota-connectivity)
10. [Step 8: Build the secbot_deploy ROS2 Package](#step-8-build-the-secbot_deploy-ros2-package)
11. [Step 9: Verify micro-ROS Agents](#step-9-verify-micro-ros-agents)
12. [Step 10: Test the Full Pipeline](#step-10-test-the-full-pipeline)
13. [Day-to-Day Usage](#day-to-day-usage)
14. [How Static IPs Work](#how-static-ips-work)
15. [How the Pipeline Works End-to-End](#how-the-pipeline-works-end-to-end)
16. [Troubleshooting](#troubleshooting)

---

## Architecture Overview

```
 Teensy (MCU)                     Pi Host                    Docker Container
 ────────────                     ───────                    ────────────────
 Button 4 held 1s
   → DeploySubsystem
   → publishes /mcu_robot/deploy/trigger ──micro-ROS──►  secbot_deploy node
                                                             writes trigger file
                                          ◄── file IPC ──  scripts/.deploy/trigger
                                  deploy-orchestrator.py
                                    reads trigger
                                    git pull
                                    flash ESP32s (OTA)
                                    flash Teensy (USB)
                                    docker restart
                                    colcon build
                                          ──► writes status to scripts/.deploy/status
                                                             secbot_deploy reads status
   ◄── /mcu_robot/lcd/append (OLED) ◄─────────────────────  publishes to OLED
   ◄── /mcu_robot/deploy/status (LED phases) ◄────────────  publishes phase info
```

The system has three layers:
- **MCU (Teensy):** Detects button hold, publishes trigger via micro-ROS, displays status on OLED/LEDs
- **Docker (secbot_deploy node):** Bridges micro-ROS topics to file-based IPC
- **Pi Host (deploy-orchestrator.py):** Runs the actual deployment pipeline (git, flash, docker, colcon)

---

## Prerequisites

Before starting, you need:
- Raspberry Pi with Raspbian/Ubuntu, connected to Teensy via USB (`/dev/ttyACM0`)
- Docker and Docker Compose installed on the Pi
- The SEC26 repo cloned at `/home/ieee/SEC26`
- All ESP32s powered (beacons, minibot, drone) with USB cables available for first-time flash
- The Teensy connected to the Pi via USB

---

## Step 1: Pi WiFi Access Point Setup

The Pi runs as a WiFi access point so all ESP32s can connect to it. The ESP32 static IPs are set in firmware (not DHCP), so the AP just needs to be on the right subnet.

If the AP is **already configured** (SSID `UCFIEEEBot`, password `goodlife`, Pi at `192.168.4.1`), skip to Step 2.

The AP is managed by **NetworkManager**. To set it up from scratch:

### Create the AP connection

```bash
# Create a WiFi AP hotspot using nmcli
sudo nmcli connection add \
  type wifi \
  ifname wlan0 \
  con-name SEC26-AP \
  autoconnect yes \
  ssid UCFIEEEBot \
  mode ap

# Set the WiFi security (WPA2-PSK)
sudo nmcli connection modify SEC26-AP \
  wifi-sec.key-mgmt wpa-psk \
  wifi-sec.psk "goodlife"

# Set the static IP for the AP interface
sudo nmcli connection modify SEC26-AP \
  ipv4.method shared \
  ipv4.addresses 192.168.4.1/24

# Set WiFi band/channel (optional — 2.4GHz channel 7)
sudo nmcli connection modify SEC26-AP \
  wifi.band bg \
  wifi.channel 7
```

### Activate the AP

```bash
# Bring it up
sudo nmcli connection up SEC26-AP

# Verify it's running
nmcli connection show --active
# Should show SEC26-AP as active on wlan0

# Verify the IP
ip addr show wlan0
# Should show 192.168.4.1/24
```

### Make it start on boot

NetworkManager auto-starts connections with `autoconnect yes` (set above). Verify:

```bash
nmcli connection show SEC26-AP | grep autoconnect
# Should show: connection.autoconnect: yes
```

### Configure the shared DHCP range (optional)

When using `ipv4.method shared`, NetworkManager runs a built-in DHCP server (dnsmasq). By default it assigns from `192.168.4.x`. The ESP32s use static IPs compiled into firmware so they bypass DHCP, but any other WiFi clients (laptops, phones) will get addresses via DHCP.

To customize the DHCP range and avoid conflicts with the ESP32 static IPs, create a dnsmasq override:

```bash
sudo mkdir -p /etc/NetworkManager/dnsmasq-shared.d
sudo tee /etc/NetworkManager/dnsmasq-shared.d/sec26.conf << 'EOF'
# Reserve 192.168.4.1-49 for static devices (Pi, ESP32s)
# DHCP range starts at .50 for dynamic clients
dhcp-range=192.168.4.50,192.168.4.150,255.255.255.0,24h
EOF

# Restart NetworkManager to pick up the change
sudo systemctl restart NetworkManager
sudo nmcli connection up SEC26-AP
```

### Verify from ESP32 perspective

After the AP is up and ESP32s are powered, they should auto-connect:
```bash
# List connected WiFi clients
iw dev wlan0 station dump | grep Station
```

---

## Step 2: Docker Container Setup

```bash
cd /home/ieee/SEC26

# Make sure .env is configured for production
# BUILD_TARGET=prod for robot, BUILD_TARGET=dev for development
cat .env

# Build and start the container
# The container now auto-starts micro-ROS agents via container-entrypoint.sh
docker compose up -d --build
```

The container-entrypoint.sh (configured in `docker-compose.yml`) automatically starts:
- A **serial micro-ROS agent** on `/dev/ttyACM0` for the Teensy (auto-reconnects if Teensy reboots)
- A **UDP micro-ROS agent** on port 8888 for all ESP32s

---

## Step 3: Install the Deploy Orchestrator Service

The deploy orchestrator is a Python daemon that runs **on the Pi host** (not in Docker). It watches for trigger files and runs the deployment pipeline.

```bash
cd /home/ieee/SEC26

# Create the IPC directory used for communication between Docker and host
mkdir -p scripts/.deploy

# Copy the systemd unit file
sudo cp scripts/secbot-deploy-orchestrator.service /etc/systemd/system/

# Reload systemd to pick up the new file
sudo systemctl daemon-reload

# Enable it to start on boot
sudo systemctl enable secbot-deploy-orchestrator

# Start it now
sudo systemctl start secbot-deploy-orchestrator

# Verify it's running
sudo systemctl status secbot-deploy-orchestrator
```

You should see `Active: active (running)`. Check the logs:

```bash
journalctl -u secbot-deploy-orchestrator -f
# Should show: "Deploy orchestrator watching /home/ieee/SEC26/scripts/.deploy/trigger"
```

---

## Step 4: Remove the Old Button-Trigger Service

The old system used a GPIO button on the Pi that called the Gitea API. This is replaced by the Teensy button + deploy orchestrator.

```bash
# Stop and disable the old service (ignore errors if it doesn't exist)
sudo systemctl stop sec26-button-dispatch 2>/dev/null || true
sudo systemctl disable sec26-button-dispatch 2>/dev/null || true
sudo rm -f /etc/systemd/system/sec26-button-dispatch.service
sudo systemctl daemon-reload
```

The old files `scripts/button-trigger-workflow.py` and `scripts/sec26-button-dispatch.service` have been deleted from the repo.

---

## Step 5: Flash Teensy (First Time)

The Teensy must be connected to the Pi via USB. Flash it with the robot firmware that includes the new DeploySubsystem:

```bash
# Enter the Docker container
docker compose exec devcontainer bash

# Navigate to the MCU workspace
cd /home/ubuntu/mcu_workspaces/sec26mcu

# Build and flash the robot firmware
pio run -e robot --target upload

# Or flash the test firmware instead:
# pio run -e teensy-test-all-subsystems --target upload

# Verify the Teensy is running (open serial monitor)
pio device monitor -e robot
# You should see "SEC26 Robot — TeensyThreads" and subsystem init messages
# Press Ctrl+C to exit

# Exit the container
exit
```

After flashing, the Teensy will:
- Connect to the micro-ROS serial agent automatically
- Start publishing on `/mcu_robot/deploy/trigger` when button 4 is held
- Listen on `/mcu_robot/deploy/status` for phase updates

---

## Step 6: Flash ESP32s via USB (First Time — OTA Bootstrap)

**This must be done via USB/serial the first time.** After this initial flash, all future updates go over WiFi via OTA.

Connect each ESP32 to the Pi (or your PC) via USB one at a time:

```bash
# Enter the Docker container
docker compose exec devcontainer bash
cd /home/ubuntu/mcu_workspaces/sec26mcu
```

### Beacons

Connect beacon 1 ESP32 via USB, then:
```bash
pio run -e beacon1 --target upload
pio device monitor -e beacon1
# Should see: "UWB Beacon 10 starting...", WiFi connect, "[OTA] Ready as sec26-beacon-10"
# Ctrl+C to exit
```

Disconnect beacon 1, connect beacon 2:
```bash
pio run -e beacon2 --target upload
# Monitor to verify: pio device monitor -e beacon2
```

Disconnect beacon 2, connect beacon 3:
```bash
pio run -e beacon3 --target upload
```

### Minibot

Connect the minibot ESP32 via USB:
```bash
pio run -e minibot --target upload
pio device monitor -e minibot
# Should see: "Minibot starting...", WiFi connect, "[OTA] Ready as sec26-minibot"
```

### Drone

Connect the drone ESP32 via USB:
```bash
pio run -e drone --target upload
pio device monitor -e drone
# Should see: "Drone starting...", WiFi connect, "[OTA] Ready as sec26-drone"
```

```bash
# Exit the container when done
exit
```

---

## Step 7: Verify OTA Connectivity

After flashing all ESP32s and powering them on (they should connect to the `UCFIEEEBot` WiFi automatically), verify they're reachable from the Pi:

```bash
# From the Pi host:
ping -c 1 192.168.4.20 && echo "beacon1 OK"  || echo "beacon1 UNREACHABLE"
ping -c 1 192.168.4.21 && echo "beacon2 OK"  || echo "beacon2 UNREACHABLE"
ping -c 1 192.168.4.22 && echo "beacon3 OK"  || echo "beacon3 UNREACHABLE"
ping -c 1 192.168.4.24 && echo "minibot OK"  || echo "minibot UNREACHABLE"
ping -c 1 192.168.4.25 && echo "drone OK"    || echo "drone UNREACHABLE"
```

If a device is unreachable:
- Check it's powered on
- Check the serial monitor for WiFi connection errors
- Verify the AP is running: `sudo systemctl status hostapd`
- The ESP32 may need a power cycle after first flash

### Test OTA manually (optional)

```bash
# Build firmware without flashing (inside Docker)
docker compose exec devcontainer bash -c \
  "cd /home/ubuntu/mcu_workspaces/sec26mcu && pio run -e beacon1"

# Flash via OTA from Pi host
bash scripts/flash_esp32_ota.sh 192.168.4.20 mcu_ws/.pio/build/beacon1/firmware.bin

# If successful, you'll see "SUCCESS: OTA flash complete for 192.168.4.20"
```

---

## Step 8: Build the secbot_deploy ROS2 Package

```bash
docker compose exec devcontainer bash -c \
  "source /opt/ros/jazzy/setup.bash && \
   cd /home/ubuntu/ros2_workspaces && \
   colcon build --packages-select secbot_deploy"
```

This builds the deploy_node that bridges micro-ROS deploy topics to the file-based IPC.

To launch it (this will be part of the normal robot launch eventually):

```bash
docker compose exec devcontainer bash -c \
  "source /opt/ros/jazzy/setup.bash && \
   source /home/ubuntu/ros2_workspaces/install/setup.bash && \
   ros2 launch secbot_deploy deploy.launch.py"
```

---

## Step 9: Verify micro-ROS Agents

The container-entrypoint.sh starts the micro-ROS agents automatically. Verify they're running:

```bash
docker compose exec devcontainer bash -c "ps aux | grep micro_ros_agent"
```

You should see two processes:
- `micro_ros_agent serial --dev /dev/ttyACM0 -b 921600` (Teensy)
- `micro_ros_agent udp4 --port 8888` (ESP32s)

Verify ROS2 topics are flowing:

```bash
docker compose exec devcontainer bash -c \
  "source /opt/ros/jazzy/setup.bash && ros2 topic list"
# Should include /mcu_robot/heartbeat, /mcu_robot/deploy/trigger, etc.
```

---

## Step 10: Test the Full Pipeline

### Test 1: Verify button trigger

1. Make sure secbot_deploy node is running (Step 8)
2. Hold **button 4** on the robot for 1 second
3. Watch the OLED — should show "Deploy triggered: robot" (or "teensy-test-all-subsystems" if DIP 8 is off)
4. Check the trigger file was created:
   ```bash
   cat scripts/.deploy/trigger
   ```

### Test 2: Manual trigger (without button)

You can trigger a deploy by writing the trigger file directly:

```bash
# From the Pi host:
mkdir -p scripts/.deploy
cat > scripts/.deploy/trigger << 'EOF'
target=robot
branch=prod
online=false
timestamp=0
EOF
```

Watch the orchestrator logs:
```bash
journalctl -u secbot-deploy-orchestrator -f
```

### Test 3: Cancel a deploy

During deployment, hold button 4 for 1 second again. LEDs should turn yellow and the pipeline should abort.

---

## Day-to-Day Usage

### Button Deploy (most common)

1. Set DIP switch 8: **ON** = `robot`, **OFF** = `teensy-test-all-subsystems`
2. Hold button 4 for 1 second
3. Watch LEDs: blue = active phase, green = done, red = failed
4. Cancel: hold button 4 again for 1 second

### Gitea CI Deploy

Just push to `prod` (or `master` which auto-syncs to `prod`). The CI workflow builds all MCU environments and triggers the same orchestrator pipeline on the Pi.

### Offline Deploy with Pre-Built Binaries

1. Build on your PC: `pio run -e robot` (produces `.pio/build/robot/firmware.hex`)
2. Copy the hex/bin files to `/home/ieee/SEC26/` on the Pi, named by environment:
   - `robot.hex`, `teensy-test-all-subsystems.hex`
   - `beacon1.bin`, `beacon2.bin`, `beacon3.bin`
   - `minibot.bin`, `drone.bin`
3. Hold button 4 for 1 second — orchestrator finds and uses the local binaries

---

## How Static IPs Work

The ESP32 static IPs are **not** assigned by DHCP. They are compiled into the firmware via build flags in `mcu_ws/platformio.ini`:

```ini
# Example from [env:beacon1]:
build_flags =
    ${esp32_microros_wifi.build_flags}
    -D BEACON_ID=10
    '-DLOCAL_MAC={ 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0x10 }'
    '-DLOCAL_IP={ 192, 168, 4, 20 }'
```

The `esp32_microros_wifi` base sets the shared network config:
```ini
'-DAGENT_IP={ 192, 168, 4, 1 }'    # Pi's IP (micro-ROS agent)
'-DAGENT_PORT=8888'                  # UDP port
'-DWIFI_SSID="UCFIEEEBot"'
'-DWIFI_PASSWORD="goodlife"'
```

The micro-ROS WiFi transport library reads these defines at boot and configures the ESP32's WiFi with the static IP + MAC. No AP-side configuration is needed to assign these addresses — the ESP32 tells the AP "I am this IP" and the AP accepts it.

The dnsmasq reservations in Step 1 are optional but prevent the DHCP server from accidentally giving another device one of these addresses.

**IP assignment table:**

| Device | `LOCAL_IP` | `LOCAL_MAC` | PIO Environment |
|--------|-----------|-------------|-----------------|
| beacon1 | 192.168.4.20 | AA:BB:CC:DD:EE:10 | `beacon1` |
| beacon2 | 192.168.4.21 | AA:BB:CC:DD:EE:11 | `beacon2` |
| beacon3 | 192.168.4.22 | AA:BB:CC:DD:EE:12 | `beacon3` |
| minibot | 192.168.4.24 | AA:BB:CC:DD:EE:14 | `minibot` |
| drone   | 192.168.4.25 | DE:AD:BE:EF:05:25 | `drone` |

To change an IP, edit the corresponding `[env:...]` section in `mcu_ws/platformio.ini` and reflash.

---

## How the Pipeline Works End-to-End

When button 4 is held for 1 second:

```
1. Teensy DeploySubsystem detects button hold (50Hz polling)
2. Reads DIP switch 8 to determine target ("robot" or "teensy-test-all-subsystems")
3. Publishes "start:robot" to /mcu_robot/deploy/trigger via micro-ROS
4. Sets all 5 LEDs to blue (starting)

5. Docker: secbot_deploy node receives the trigger
6. Writes scripts/.deploy/trigger file (target=robot, branch=prod, online=false)
7. Publishes "Deploy triggered: robot" to /mcu_robot/lcd/append (OLED)

8. Pi host: deploy-orchestrator.py detects trigger file (polling every 2s)
9. Runs pipeline:
   a. Git pull (if online) → writes status phase=git_pull
   b. ESP32 OTA flash (5 devices, 3 retries each) → phase=esp32_ota
   c. Teensy USB flash (3 retries) → phase=mcu_flash
      NOTE: This reboots the MCU, so OLED/LED feedback stops here
   d. Docker restart (if ROS changes) → phase=docker
   e. Colcon build → phase=colcon_build
   f. Done → phase=done

10. Docker: secbot_deploy reads status file (polling every 500ms)
11. Publishes "phase:git_pull" etc. to /mcu_robot/deploy/status
12. Publishes human-readable messages to /mcu_robot/lcd/append (OLED)

13. Teensy DeploySubsystem receives phase updates
14. Updates LEDs: previous phases green, current phase blue, future phases off
15. On "done": all LEDs green
16. On "failed": current phase LED red
```

---

## Troubleshooting

### Service issues

```bash
# Check orchestrator status and logs
sudo systemctl status secbot-deploy-orchestrator
journalctl -u secbot-deploy-orchestrator -f

# Restart the orchestrator
sudo systemctl restart secbot-deploy-orchestrator

# Check if the IPC directory exists
ls -la /home/ieee/SEC26/scripts/.deploy/
```

### micro-ROS agent issues

```bash
# Check agents are running inside container
docker compose exec devcontainer bash -c "ps aux | grep micro_ros_agent"

# If serial agent isn't running, check Teensy USB
ls /dev/ttyACM*

# Restart the container (this restarts the agents)
docker compose restart devcontainer

# Check ROS2 topics are flowing
docker compose exec devcontainer bash -c \
  "source /opt/ros/jazzy/setup.bash && ros2 topic list"
```

### ESP32 OTA issues

```bash
# Check if ESP32 is reachable
ping -c 3 192.168.4.20

# Check AP is running
sudo systemctl status hostapd

# Check connected WiFi clients
iw dev wlan0 station dump

# If OTA fails, reflash via USB (connect ESP32 to Pi, then inside Docker):
docker compose exec devcontainer bash -c \
  "cd /home/ubuntu/mcu_workspaces/sec26mcu && pio run -e beacon1 --target upload"
```

### Teensy flash issues

```bash
# Check Teensy is visible
ls -la /dev/ttyACM*

# If /dev/ttyACM0 doesn't exist:
# - Unplug and replug the Teensy USB cable
# - Check dmesg for USB errors: dmesg | tail -20

# Flash manually inside Docker:
docker compose exec devcontainer bash -c \
  "cd /home/ubuntu/mcu_workspaces/sec26mcu && pio run -e robot --target upload"
```

### Button not working

```bash
# Verify the deploy topic exists
docker compose exec devcontainer bash -c \
  "source /opt/ros/jazzy/setup.bash && ros2 topic echo /mcu_robot/deploy/trigger --once"
# Then hold button 4 for 1 second — should print the trigger message

# Check button subsystem is publishing
docker compose exec devcontainer bash -c \
  "source /opt/ros/jazzy/setup.bash && ros2 topic echo /mcu_robot/buttons --once"
# Press button 4 — bit 3 should toggle in the bitmask
```

### Clean rebuild after mcu_msgs changes

If you modified any `.msg` or `.srv` files:

```bash
docker compose exec devcontainer bash -c \
  "cd /home/ubuntu/mcu_workspaces/sec26mcu && \
   pio run -e robot -t clean_microros && \
   pio run -e robot"
```
